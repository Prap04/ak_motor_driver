#include "rclcpp/rclcpp.hpp"
#include "ak_motor_driver/msg/motor_command_array.hpp"
#include "ak_motor_driver/msg/motor_state_array.hpp"
#include "ak_motor_driver/srv/enter_mit_mode.hpp"
#include "cubemars.hpp"

#include <chrono>
#include <cerrno>
#include <cstring>
#include <memory>
#include <net/if.h>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

using ak_motor_driver::msg::MotorCommandArray;
using ak_motor_driver::msg::MotorStateArray;
using ak_motor_driver::srv::EnterMitMode;

namespace
{
constexpr std::uint8_t kModePosition = 0;
constexpr std::uint8_t kModeVelocity = 1;
constexpr std::uint8_t kModeTorque = 2;
}

class AkMotorDriverNode : public rclcpp::Node
{
public:
  explicit AkMotorDriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("ak_motor_driver_node", options)
  {
    this->declare_parameter("can_interface", "can0");
    this->declare_parameter("motor_ids", std::vector<int64_t>{1});
    this->declare_parameter("publish_rate_hz", 100.0);
    this->declare_parameter("kd", 1.0);
    this->declare_parameter("command_timeout_s", 0.5);
    this->declare_parameter("max_consecutive_failures", 5);

    const auto can_interface = this->get_parameter("can_interface").as_string();
    const auto motor_ids = this->get_parameter("motor_ids").as_integer_array();
    const auto publish_rate_hz = this->get_parameter("publish_rate_hz").as_double();
    const double kd = this->get_parameter("kd").as_double();
    command_timeout_s_ = this->get_parameter("command_timeout_s").as_double();
    max_consecutive_failures_ = this->get_parameter("max_consecutive_failures").as_int();
    last_command_time_ = this->now();

    // Phase 1: CAN interface identified
    const unsigned int if_index = if_nametoindex(can_interface.c_str());
    if (if_index == 0) {
      RCLCPP_ERROR(
        this->get_logger(),
        "CAN interface %s: NOT found (%s)",
        can_interface.c_str(), std::strerror(errno));
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "CAN interface %s: identified (index %u)",
        can_interface.c_str(), if_index);
    }

    // Initialize each motor through the phases
    for (const auto & motor_id : motor_ids) {
      const auto id = static_cast<std::uint8_t>(motor_id);
      motors_[id] = std::make_unique<cubemars::CubeMarsMotor>(can_interface, id);
      auto & motor = *motors_[id];
      last_kd_[id] = kd;

      // Phase 2: Connected to CAN
      RCLCPP_INFO(this->get_logger(), "Motor %u: connecting to CAN...", id);
      const auto conn_err = motor.connect();
      if (conn_err != cubemars::Error::none) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Motor %u: CAN connection FAILED (%s)",
          id, motor.lastErrorMessage().c_str());
        continue;
      }
      RCLCPP_INFO(this->get_logger(), "Motor %u: CAN connection OK", id);

      // Phase 3: Entering MIT mode
      RCLCPP_INFO(this->get_logger(), "Motor %u: entering MIT mode...", id);
      const auto mit_err = motor.enterMitMode();
      if (mit_err != cubemars::Error::none) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Motor %u: entering MIT mode FAILED (%s)",
          id, motor.lastErrorMessage().c_str());
        continue;
      }
      RCLCPP_INFO(this->get_logger(), "Motor %u: MIT mode active", id);

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      motor.setZero();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

      // Phase 4: Sending the first command
      cubemars::MotorCommand initial_cmd{};
      initial_cmd.position_rad = 0.0;
      initial_cmd.velocity_rad_per_s = 0.0;
      initial_cmd.kp = 0.0;
      initial_cmd.kd = kd;
      initial_cmd.torque_nm = 0.0;

      RCLCPP_INFO(this->get_logger(), "Motor %u: sending first command...", id);
      const auto cmd_err = motor.setMitCommand(initial_cmd);
      if (cmd_err != cubemars::Error::none) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Motor %u: sending first command FAILED (%s)",
          id, motor.lastErrorMessage().c_str());
      }

      // Phase 5: Received command / response
      cubemars::MotorState initial_state{};
      const auto read_err = motor.readState(initial_state);
      if (read_err != cubemars::Error::none || !initial_state.valid) {
        RCLCPP_WARN(
          this->get_logger(),
          "Motor %u: response not received (%s)",
          id, motor.lastErrorMessage().c_str());
      } else {
        RCLCPP_INFO(
          this->get_logger(),
          "Motor %u: response received (pos=%.3f rad, vel=%.3f rad/s, torque=%.3f Nm, temp=%.1f C)",
          id, initial_state.position_rad, initial_state.velocity_rad_per_s,
          initial_state.torque_nm, initial_state.temperature_c);
      }

      RCLCPP_INFO(this->get_logger(), "Motor %u: READY", id);
    }
  
    // Phase 6: Publishing on motor
    command_sub_ = this->create_subscription<MotorCommandArray>(
      "motor_commands",
      10,
      [this](const MotorCommandArray::SharedPtr msg) {
        this->commandCallback(msg);
      });

    state_pub_ = this->create_publisher<MotorStateArray>("motor_states", 10);
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz),
      std::bind(&AkMotorDriverNode::publishState, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Publishing on motor: 'motor_states' ready at %.1f Hz",
      publish_rate_hz);

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
        const auto result = it->second->enterMitMode();
        response->success = result == cubemars::Error::none;
        response->error_code = static_cast<uint8_t>(result);
        response->message = result == cubemars::Error::none ? "ok" : it->second->lastErrorMessage();
      });
  }

private:
  void commandCallback(const MotorCommandArray::SharedPtr msg)
  {
    last_command_time_ = this->now();
    if (command_watchdog_triggered_) {
      RCLCPP_INFO(this->get_logger(), "Command watchdog cleared");
      command_watchdog_triggered_ = false;
    }

    for (const auto & command : msg->commands) {
      const auto motor_id = static_cast<std::uint8_t>(command.motor_id);
      auto it = motors_.find(motor_id);
      if (it == motors_.end()) {
        RCLCPP_WARN(this->get_logger(), "Received command for unconfigured motor %u", motor_id);
        continue;
      }

      switch (command.mode) {
        case kModePosition:
          it->second->setPositionCommand(command.position_rad, command.kp, command.kd);
          break;
        case kModeVelocity:
          it->second->setVelocityCommand(command.velocity_rad_per_s, command.kd);
          break;
        case kModeTorque:
          it->second->setTorque(command.torque_nm);
          break;
        default:
          RCLCPP_WARN(this->get_logger(), "Unknown mode %u for motor %u", command.mode, motor_id);
          break;
      }
    }
  }

  void safeStop(cubemars::CubeMarsMotor & motor, std::uint8_t motor_id, double kd)
  {
    motor.setVelocityCommand(0.0, kd);
    RCLCPP_WARN(this->get_logger(), "Safety stop issued to motor %u", motor_id);
  }

  void publishState()
  {
    MotorStateArray msg;
    const double since_last_command = (this->now() - last_command_time_).seconds();

    if (since_last_command > command_timeout_s_ && !command_watchdog_triggered_) {
      command_watchdog_triggered_ = true;
      for (auto & [motor_id, motor] : motors_) {
        safeStop(*motor, motor_id, last_kd_[motor_id]);
      }
    }

    for (const auto & [motor_id, motor] : motors_) {
      cubemars::MotorState state;
      const auto error = motor->readState(state);

      if (error != cubemars::Error::none || !state.valid) {
        consecutive_failures_[motor_id]++;
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Motor %u read failed (%d in a row): %s",
          motor_id, consecutive_failures_[motor_id], motor->lastErrorMessage().c_str());

        if (consecutive_failures_[motor_id] == max_consecutive_failures_) {
          safeStop(*motor, motor_id, last_kd_[motor_id]);
        }
        continue;
      }
      consecutive_failures_[motor_id] = 0;

      RCLCPP_INFO(
        this->get_logger(),
        "Motor %u | Position: %.3f rad | Velocity: %.3f rad/s | Torque: %.3f Nm | Temp: %.1f C",
        motor_id,
        state.position_rad,
        state.velocity_rad_per_s,
        state.torque_nm,
        state.temperature_c);

      ak_motor_driver::msg::MotorState motor_state;
      motor_state.motor_id = state.motor_id;
      motor_state.position_rad = state.position_rad;
      motor_state.velocity_rad_per_s = state.velocity_rad_per_s;
      motor_state.torque_nm = state.torque_nm;
      motor_state.temperature_c = state.temperature_c;
      motor_state.error = state.error;
      motor_state.valid = state.valid;

      msg.states.push_back(motor_state);
    }

    if (!msg.states.empty()) {
      state_pub_->publish(msg);
    }
  }

  std::unordered_map<std::uint8_t, std::unique_ptr<cubemars::CubeMarsMotor>> motors_;
  std::unordered_map<std::uint8_t, double> last_kd_;
  std::unordered_map<std::uint8_t, int> consecutive_failures_;
  rclcpp::Subscription<MotorCommandArray>::SharedPtr command_sub_;
  rclcpp::Publisher<MotorStateArray>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<EnterMitMode>::SharedPtr enter_mit_service_;
  double command_timeout_s_;
  int max_consecutive_failures_;
  rclcpp::Time last_command_time_;
  bool command_watchdog_triggered_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AkMotorDriverNode>());
  rclcpp::shutdown();
  return 0;
}