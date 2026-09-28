// ak_motor_driver_node.cpp
//
// ROS 2 wrapper around cubemars::CubeMarsMotor with:
//   * an explicit, logged, per-motor startup sequence (like a drone's pre-arm checks),
//   * a fixed-rate "send setpoint -> read reply" control loop,
//   * command-timeout and feedback-loss watchdogs that move the motor to SAFE_STOPPED,
//   * application-level safety limits (separate from the protocol's encoding limits),
//   * a 1 Hz status line per motor with counters, so "why doesn't it move?" is answerable.
//
// Per-motor phases:
//   DISCONNECTED -> ENTERING_MIT -> AWAITING_FEEDBACK -> ACTIVE <-> SAFE_STOPPED
//                                                          \-> FAULT (latched)
// Every phase change is logged with a reason.

#include "rclcpp/rclcpp.hpp"
#include "ak_motor_driver/msg/motor_command_array.hpp"
#include "ak_motor_driver/msg/motor_state_array.hpp"
#include "ak_motor_driver/srv/enter_mit_mode.hpp"
#include "cubemars.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

using ak_motor_driver::msg::MotorCommandArray;
using ak_motor_driver::msg::MotorStateArray;
using ak_motor_driver::srv::EnterMitMode;

namespace
{
using Clock = std::chrono::steady_clock;

constexpr std::uint8_t kModePosition = 0;
constexpr std::uint8_t kModeVelocity = 1;
constexpr std::uint8_t kModeTorque = 2;

enum class Phase
{
  kDisconnected,      // CAN socket not open yet
  kEnteringMit,       // socket open; sending the "enter MIT mode" frame
  kAwaitingFeedback,  // pinging the motor until the first valid state frame arrives
  kActive,            // feedback verified; commands are accepted and forwarded
  kSafeStopped,       // watchdog tripped; braking (velocity 0 + damping) until a new command
  kFault              // motor reported a fault code; latched until re-initialised
};

enum class TxKind
{
  kPing,      // harmless "enter MIT mode" frame; the motor answers with its state
  kSetpoint,  // the latest accepted user command
  kSafeStop   // velocity 0 with damping (kd)
};

const char * phaseName(Phase phase)
{
  switch (phase) {
    case Phase::kDisconnected: return "DISCONNECTED";
    case Phase::kEnteringMit: return "ENTERING_MIT";
    case Phase::kAwaitingFeedback: return "AWAITING_FEEDBACK";
    case Phase::kActive: return "ACTIVE";
    case Phase::kSafeStopped: return "SAFE_STOPPED";
    case Phase::kFault: return "FAULT";
  }
  return "UNKNOWN";
}

double secondsSince(Clock::time_point since)
{
  return std::chrono::duration<double>(Clock::now() - since).count();
}

// Everything the node knows about one motor.
struct MotorContext
{
  std::unique_ptr<cubemars::CubeMarsMotor> driver;
  Phase phase{Phase::kDisconnected};
  Clock::time_point phase_since{Clock::now()};
  Clock::time_point last_attempt{};       // paces startup retries
  Clock::time_point last_command_time{};  // heartbeat for the command watchdog
  bool has_command{false};
  cubemars::MotorCommand setpoint{};      // latest accepted command, re-sent every tick
  int consecutive_failures{0};
  bool fresh_state{false};                // a new valid state is waiting to be published
  cubemars::MotorState last_state{};

  // Counters shown in the status line.
  std::uint64_t commands_received{0};
  std::uint64_t commands_rejected{0};
  std::uint64_t frames_sent{0};
  std::uint64_t send_errors{0};
  std::uint64_t reads_ok{0};
  std::uint64_t reads_failed{0};
};
}  // namespace

class AkMotorDriverNode : public rclcpp::Node
{
public:
  explicit AkMotorDriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("ak_motor_driver_node", options)
  {
    this->declare_parameter("can_interface", "can0");
    this->declare_parameter("motor_ids", std::vector<int64_t>{1});
    this->declare_parameter("publish_rate_hz", 100.0);  // control loop rate: TX + RX + publish
    this->declare_parameter("read_timeout_ms", 5);      // max wait for a reply per motor per tick
    this->declare_parameter("kd", 1.0);                 // damping used by the safe stop
    this->declare_parameter("command_timeout_s", 0.5);
    this->declare_parameter("max_consecutive_failures", 5);
    this->declare_parameter("startup_timeout_s", 3.0);
    this->declare_parameter("zero_on_start", false);
    this->declare_parameter("max_velocity_rad_per_s", 5.0);
    this->declare_parameter("max_torque_nm", 1.0);
    this->declare_parameter("max_kp", 10.0);
    this->declare_parameter("max_kd", 5.0);

    const auto can_interface = this->get_parameter("can_interface").as_string();
    const auto motor_ids = this->get_parameter("motor_ids").as_integer_array();
    publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();
    if (publish_rate_hz_ <= 0.0) {
      publish_rate_hz_ = 50.0;
    }
    const int read_timeout_ms = static_cast<int>(this->get_parameter("read_timeout_ms").as_int());
    safe_stop_kd_ = this->get_parameter("kd").as_double();
    command_timeout_s_ = this->get_parameter("command_timeout_s").as_double();
    max_consecutive_failures_ =
      static_cast<int>(this->get_parameter("max_consecutive_failures").as_int());
    startup_timeout_s_ = this->get_parameter("startup_timeout_s").as_double();
    zero_on_start_ = this->get_parameter("zero_on_start").as_bool();
    max_velocity_ = this->get_parameter("max_velocity_rad_per_s").as_double();
    max_torque_ = this->get_parameter("max_torque_nm").as_double();
    max_kp_ = this->get_parameter("max_kp").as_double();
    max_kd_ = this->get_parameter("max_kd").as_double();

    for (const auto & raw_id : motor_ids) {
      const auto id = static_cast<std::uint8_t>(raw_id);
      auto & ctx = motors_.try_emplace(id).first->second;
      ctx.driver = std::make_unique<cubemars::CubeMarsMotor>(
        can_interface, id, cubemars::MitLimits{}, read_timeout_ms);
      RCLCPP_INFO(
        this->get_logger(), "[motor %u] created on %s (startup sequence runs on the control loop)",
        static_cast<unsigned>(id), can_interface.c_str());
    }

    command_sub_ = this->create_subscription<MotorCommandArray>(
      "motor_commands",
      10,
      [this](const MotorCommandArray::SharedPtr msg) {
        this->commandCallback(msg);
      });

    state_pub_ = this->create_publisher<MotorStateArray>("motor_states", 10);

    control_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz_),
      [this]() { this->controlTick(); });

    status_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() { this->statusTick(); });

    // Re-runs the startup sequence for one motor (also the way to clear a latched FAULT).
    enter_mit_service_ = this->create_service<EnterMitMode>(
      "enter_mit_mode",
      [this](const std::shared_ptr<EnterMitMode::Request> request,
             const std::shared_ptr<EnterMitMode::Response> response) {
        const auto motor_id = static_cast<std::uint8_t>(request->motor_id);
        auto it = motors_.find(motor_id);
        if (it == motors_.end()) {
          response->success = false;
          response->error_code = 255;
          response->message = "motor_id not configured";
          return;
        }
        auto & ctx = it->second;
        ctx.has_command = false;
        ctx.consecutive_failures = 0;
        const Phase next = ctx.driver->connected() ? Phase::kEnteringMit : Phase::kDisconnected;
        setPhase(motor_id, ctx, next, "re-initialise requested via enter_mit_mode service");
        response->success = true;
        response->error_code = 0;
        response->message = "startup sequence restarted; watch the node log for phase changes";
      });

    RCLCPP_INFO(
      this->get_logger(),
      "Limits: |v|<=%.2f rad/s, |torque|<=%.2f Nm, kp<=%.1f, kd<=%.1f | command timeout %.2fs | "
      "control loop %.0f Hz, read timeout %d ms | zero_on_start=%s",
      max_velocity_, max_torque_, max_kp_, max_kd_, command_timeout_s_, publish_rate_hz_,
      read_timeout_ms, zero_on_start_ ? "true" : "false");
  }

private:
  // ---------------------------------------------------------------- phase handling
  void setPhase(std::uint8_t id, MotorContext & ctx, Phase next, const std::string & reason)
  {
    if (ctx.phase == next) {
      return;
    }
    const auto from = phaseName(ctx.phase);
    const auto to = phaseName(next);
    if (next == Phase::kFault) {
      RCLCPP_ERROR(
        this->get_logger(), "[motor %u] %s -> %s : %s", static_cast<unsigned>(id), from, to,
        reason.c_str());
    } else if (next == Phase::kSafeStopped) {
      RCLCPP_WARN(
        this->get_logger(), "[motor %u] %s -> %s : %s", static_cast<unsigned>(id), from, to,
        reason.c_str());
    } else {
      RCLCPP_INFO(
        this->get_logger(), "[motor %u] %s -> %s : %s", static_cast<unsigned>(id), from, to,
        reason.c_str());
    }
    ctx.phase = next;
    ctx.phase_since = Clock::now();
  }

  bool retryDue(MotorContext & ctx)
  {
    if (secondsSince(ctx.last_attempt) < 1.0) {
      return false;
    }
    ctx.last_attempt = Clock::now();
    return true;
  }

  // ---------------------------------------------------------------- command path
  bool withinLimits(const ak_motor_driver::msg::MotorCommand & c) const
  {
    return std::abs(c.velocity_rad_per_s) <= max_velocity_ &&
           std::abs(c.torque_nm) <= max_torque_ &&
           c.kp >= 0.0 && c.kp <= max_kp_ &&
           c.kd >= 0.0 && c.kd <= max_kd_;
  }

  // The callback only VALIDATES and STORES the newest setpoint.
  // The control loop is what actually transmits it, at a fixed rate.
  void commandCallback(const MotorCommandArray::SharedPtr msg)
  {
    for (const auto & command : msg->commands) {
      const auto id = static_cast<std::uint8_t>(command.motor_id);
      auto it = motors_.find(id);
      if (it == motors_.end()) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Command for unconfigured motor_id %u ignored (check the motor_ids parameter)",
          static_cast<unsigned>(id));
        continue;
      }
      auto & ctx = it->second;
      ++ctx.commands_received;

      if (ctx.phase == Phase::kFault) {
        ++ctx.commands_rejected;
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "[motor %u] command rejected: FAULT latched (call the enter_mit_mode service to re-initialise)",
          static_cast<unsigned>(id));
        continue;
      }
      if (ctx.phase != Phase::kActive && ctx.phase != Phase::kSafeStopped) {
        ++ctx.commands_rejected;
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "[motor %u] command rejected: not ready (phase=%s)", static_cast<unsigned>(id),
          phaseName(ctx.phase));
        continue;
      }
      if (!withinLimits(command)) {
        ++ctx.commands_rejected;
        RCLCPP_WARN(
          this->get_logger(),
          "[motor %u] command rejected: outside safety limits (v=%.3f torque=%.3f kp=%.2f kd=%.2f)",
          static_cast<unsigned>(id), command.velocity_rad_per_s, command.torque_nm, command.kp,
          command.kd);
        continue;
      }

      // Every mode fills the same five slots; the mode decides which slots carry real values.
      cubemars::MotorCommand setpoint{};
      switch (command.mode) {
        case kModePosition:
          setpoint.position_rad = command.position_rad;
          setpoint.kp = command.kp;
          setpoint.kd = command.kd;
          break;
        case kModeVelocity:
          setpoint.velocity_rad_per_s = command.velocity_rad_per_s;
          setpoint.kd = command.kd;
          break;
        case kModeTorque:
          setpoint.torque_nm = command.torque_nm;
          break;
        default:
          ++ctx.commands_rejected;
          RCLCPP_WARN(
            this->get_logger(), "[motor %u] command rejected: unknown mode %u",
            static_cast<unsigned>(id), static_cast<unsigned>(command.mode));
          continue;
      }

      ctx.setpoint = setpoint;
      ctx.has_command = true;
      ctx.last_command_time = Clock::now();
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[motor %u] command accepted: mode=%u p=%.3f v=%.3f kp=%.2f kd=%.2f t=%.3f",
        static_cast<unsigned>(id), static_cast<unsigned>(command.mode), setpoint.position_rad,
        setpoint.velocity_rad_per_s, setpoint.kp, setpoint.kd, setpoint.torque_nm);

      if (ctx.phase == Phase::kSafeStopped) {
        setPhase(id, ctx, Phase::kActive, "new valid command received");
      }
    }
  }

  // ---------------------------------------------------------------- control loop
  void controlTick()
  {
    const auto tick_start = Clock::now();

    for (auto & entry : motors_) {
      const std::uint8_t id = entry.first;
      MotorContext & ctx = entry.second;
      switch (ctx.phase) {
        case Phase::kDisconnected: stepDisconnected(id, ctx); break;
        case Phase::kEnteringMit: stepEnteringMit(id, ctx); break;
        case Phase::kAwaitingFeedback: stepAwaitingFeedback(id, ctx); break;
        case Phase::kActive: stepActive(id, ctx); break;
        case Phase::kSafeStopped: stepHold(id, ctx); break;
        case Phase::kFault: stepHold(id, ctx); break;
      }
    }

    publishStates();

    const double tick_s = secondsSince(tick_start);
    if (tick_s > 1.0 / publish_rate_hz_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Control tick took %.1f ms but the period is %.1f ms: lower publish_rate_hz, "
        "read_timeout_ms, or the number of motors",
        tick_s * 1000.0, 1000.0 / publish_rate_hz_);
    }
  }

  void stepDisconnected(std::uint8_t id, MotorContext & ctx)
  {
    if (!retryDue(ctx)) {
      return;
    }
    const auto err = ctx.driver->connect();
    if (err == cubemars::Error::none) {
      setPhase(id, ctx, Phase::kEnteringMit, "CAN socket opened");
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "[motor %u] cannot open CAN socket: %s (does the interface exist? try: ip -details link show can0)",
        static_cast<unsigned>(id), ctx.driver->lastErrorMessage().c_str());
    }
  }

  void stepEnteringMit(std::uint8_t id, MotorContext & ctx)
  {
    if (!retryDue(ctx)) {
      return;
    }
    const auto err = ctx.driver->enterMitMode();
    if (err != cubemars::Error::none) {
      RCLCPP_ERROR(
        this->get_logger(),
        "[motor %u] enter-MIT frame could not be sent: %s (is the interface UP at 1 Mbps? try: ip -details link show can0)",
        static_cast<unsigned>(id), ctx.driver->lastErrorMessage().c_str());
      return;
    }
    if (zero_on_start_) {
      const auto zero_err = ctx.driver->setZero();
      RCLCPP_WARN(
        this->get_logger(), "[motor %u] zero_on_start=true: current position set as zero (%s)",
        static_cast<unsigned>(id),
        zero_err == cubemars::Error::none ? "sent" : ctx.driver->lastErrorMessage().c_str());
    }
    setPhase(id, ctx, Phase::kAwaitingFeedback, "enter-MIT frame sent; waiting for first reply");
  }

  void stepAwaitingFeedback(std::uint8_t id, MotorContext & ctx)
  {
    transmit(id, ctx, TxKind::kPing);
    if (receive(id, ctx)) {
      char text[160];
      std::snprintf(
        text, sizeof(text), "feedback verified: pos=%.3f rad vel=%.3f rad/s tau=%.3f Nm T=%.1f C",
        ctx.last_state.position_rad, ctx.last_state.velocity_rad_per_s, ctx.last_state.torque_nm,
        ctx.last_state.temperature_c);
      setPhase(id, ctx, Phase::kActive, text);
      return;
    }
    if (ctx.phase == Phase::kAwaitingFeedback && secondsSince(ctx.phase_since) > startup_timeout_s_) {
      RCLCPP_WARN(
        this->get_logger(),
        "[motor %u] no valid feedback after %.1f s. Check: motor power, CAN wiring/termination, "
        "motor in MIT (not servo) mode, motor ID. Last driver error: %s",
        static_cast<unsigned>(id), startup_timeout_s_, ctx.driver->lastErrorMessage().c_str());
      setPhase(id, ctx, Phase::kEnteringMit, "retrying startup");
    }
  }

  void stepActive(std::uint8_t id, MotorContext & ctx)
  {
    if (ctx.has_command && secondsSince(ctx.last_command_time) > command_timeout_s_) {
      char text[96];
      std::snprintf(text, sizeof(text), "no command for more than %.2f s", command_timeout_s_);
      setPhase(id, ctx, Phase::kSafeStopped, text);
      stepHold(id, ctx);
      return;
    }
    // Before the first command, only ping: the motor stays passive but keeps answering.
    transmit(id, ctx, ctx.has_command ? TxKind::kSetpoint : TxKind::kPing);
    receive(id, ctx);
  }

  void stepHold(std::uint8_t id, MotorContext & ctx)
  {
    transmit(id, ctx, TxKind::kSafeStop);
    receive(id, ctx);
  }

  void transmit(std::uint8_t id, MotorContext & ctx, TxKind kind)
  {
    cubemars::Error err = cubemars::Error::none;
    switch (kind) {
      case TxKind::kPing:
        err = ctx.driver->enterMitMode();
        break;
      case TxKind::kSetpoint:
        err = ctx.driver->setMitCommand(ctx.setpoint);
        break;
      case TxKind::kSafeStop: {
        cubemars::MotorCommand stop{};
        stop.kd = safe_stop_kd_;
        err = ctx.driver->setMitCommand(stop);
        break;
      }
    }
    ++ctx.frames_sent;
    if (err != cubemars::Error::none) {
      ++ctx.send_errors;
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000, "[motor %u] TX failed: %s",
        static_cast<unsigned>(id), ctx.driver->lastErrorMessage().c_str());
    }
    RCLCPP_DEBUG(
      this->get_logger(), "[motor %u] TX kind=%d result=%d", static_cast<unsigned>(id),
      static_cast<int>(kind), static_cast<int>(err));
  }

  // Reads one reply. Returns true only for a valid frame with no fault code.
  bool receive(std::uint8_t id, MotorContext & ctx)
  {
    cubemars::MotorState state;
    const auto err = ctx.driver->readState(state);

    if (state.valid && state.error != 0) {
      ++ctx.reads_ok;
      ctx.last_state = state;
      ctx.fresh_state = true;  // publish it: faults must stay visible
      char text[96];
      std::snprintf(text, sizeof(text), "motor reported fault code %u", static_cast<unsigned>(state.error));
      setPhase(id, ctx, Phase::kFault, text);
      return false;
    }

    if (err != cubemars::Error::none || !state.valid) {
      ++ctx.reads_failed;
      ++ctx.consecutive_failures;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000, "[motor %u] read failed (%d in a row): %s",
        static_cast<unsigned>(id), ctx.consecutive_failures, ctx.driver->lastErrorMessage().c_str());
      if (ctx.phase == Phase::kActive && ctx.consecutive_failures >= max_consecutive_failures_) {
        char text[128];
        std::snprintf(
          text, sizeof(text), "feedback lost: %d consecutive failed reads", ctx.consecutive_failures);
        setPhase(id, ctx, Phase::kSafeStopped, text);
      }
      return false;
    }

    ++ctx.reads_ok;
    ctx.consecutive_failures = 0;
    ctx.last_state = state;
    ctx.fresh_state = true;
    RCLCPP_DEBUG(
      this->get_logger(), "[motor %u] RX pos=%.3f vel=%.3f tau=%.3f T=%.1f", static_cast<unsigned>(id),
      state.position_rad, state.velocity_rad_per_s, state.torque_nm, state.temperature_c);
    return true;
  }

  void publishStates()
  {
    MotorStateArray msg;
    for (auto & entry : motors_) {
      MotorContext & ctx = entry.second;
      if (!ctx.fresh_state) {
        continue;
      }
      ctx.fresh_state = false;
      ak_motor_driver::msg::MotorState motor_state;
      motor_state.motor_id = ctx.last_state.motor_id;
      motor_state.position_rad = ctx.last_state.position_rad;
      motor_state.velocity_rad_per_s = ctx.last_state.velocity_rad_per_s;
      motor_state.torque_nm = ctx.last_state.torque_nm;
      motor_state.temperature_c = ctx.last_state.temperature_c;
      motor_state.error = ctx.last_state.error;
      motor_state.valid = ctx.last_state.valid;
      msg.states.push_back(motor_state);
    }
    if (!msg.states.empty()) {
      state_pub_->publish(msg);
    }
  }

  // ---------------------------------------------------------------- status line
  void statusTick()
  {
    for (const auto & entry : motors_) {
      const std::uint8_t id = entry.first;
      const MotorContext & ctx = entry.second;
      const double cmd_age = ctx.has_command ? secondsSince(ctx.last_command_time) : -1.0;
      RCLCPP_INFO(
        this->get_logger(),
        "[motor %u] %-17s | cmds rx=%llu rejected=%llu | TX frames=%llu errors=%llu | "
        "RX ok=%llu fail=%llu (streak %d) | cmd age=%.2fs | setpoint p=%.2f v=%.2f kp=%.1f kd=%.1f t=%.2f | "
        "state pos=%.3f vel=%.3f tau=%.3f T=%.1fC err=%u",
        static_cast<unsigned>(id), phaseName(ctx.phase),
        static_cast<unsigned long long>(ctx.commands_received),
        static_cast<unsigned long long>(ctx.commands_rejected),
        static_cast<unsigned long long>(ctx.frames_sent),
        static_cast<unsigned long long>(ctx.send_errors),
        static_cast<unsigned long long>(ctx.reads_ok),
        static_cast<unsigned long long>(ctx.reads_failed), ctx.consecutive_failures, cmd_age,
        ctx.setpoint.position_rad, ctx.setpoint.velocity_rad_per_s, ctx.setpoint.kp,
        ctx.setpoint.kd, ctx.setpoint.torque_nm, ctx.last_state.position_rad,
        ctx.last_state.velocity_rad_per_s, ctx.last_state.torque_nm,
        ctx.last_state.temperature_c, static_cast<unsigned>(ctx.last_state.error));
    }
  }

  // ---------------------------------------------------------------- members
  std::unordered_map<std::uint8_t, MotorContext> motors_;
  rclcpp::Subscription<MotorCommandArray>::SharedPtr command_sub_;
  rclcpp::Publisher<MotorStateArray>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::Service<EnterMitMode>::SharedPtr enter_mit_service_;

  double publish_rate_hz_{100.0};
  double safe_stop_kd_{1.0};
  double command_timeout_s_{0.5};
  int max_consecutive_failures_{5};
  double startup_timeout_s_{3.0};
  bool zero_on_start_{false};
  double max_velocity_{5.0};
  double max_torque_{1.0};
  double max_kp_{10.0};
  double max_kd_{5.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AkMotorDriverNode>());
  rclcpp::shutdown();
  return 0;
}