#include "rclcpp/rclcpp.hpp"
#include "ak_motor_driver/msg/motor_command_array.hpp"
#include "ak_motor_driver/msg/motor_state_array.hpp"
#include "ak_motor_driver/srv/enter_mit_mode.hpp"
#include "cubemars.hpp"

#include <chrono>
#include <memory>
#include <string>
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
    this->declare_parameter("publish_rate_hz", 50.0);
    this->declare_parameter("kd", 1.0);

    const auto can_interface = this->get_parameter("can_interface").as_string();
    const auto motor_ids = this->get_parameter("motor_ids").as_integer_array();
    const auto publish_rate_hz = this->get_parameter("publish_rate_hz").as_double();
    const double kd = this->get_parameter("kd").as_double();

    for (const auto & motor_id : motor_ids) {
      const auto id = static_cast<std::uint8_t>(motor_id);
      motors_[id] = std::make_unique<cubemars::CubeMarsMotor>(can_interface, id);
      auto & motor = *motors_[id];
      if (motor.connect() != cubemars::Error::none) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect motor %u on %s", id, can_interface.c_str());
      }
      if (motor.enterMitMode() != cubemars::Error::none) {
        RCLCPP_ERROR(this->get_logger(), "Failed to enter MIT mode for motor %u", id);
      }
      motor.setZero();
      last_kd_[id] = kd;
    }

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
void publishState()
{
  MotorStateArray msg;

  for (const auto & [motor_id, motor] : motors_) {

    // Send a very small velocity command: 2 RPM
    constexpr double test_velocity_rad_s = 0.209;
    constexpr double test_kd = 1.0;

    const auto command_error =
      motor->setVelocityCommand(test_velocity_rad_s, test_kd);

    if (command_error != cubemars::Error::none) {
      RCLCPP_WARN(
        this->get_logger(),
        "Motor %u command failed: %s",
        motor_id,
        motor->lastErrorMessage().c_str());
      continue;
    }

    cubemars::MotorState state;

    const auto error = motor->readState(state);

    if (error != cubemars::Error::none || !state.valid) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Motor %u read failed: %s",
        motor_id,
        motor->lastErrorMessage().c_str());
      continue;
    }

    // Print successful feedback
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
  rclcpp::Subscription<MotorCommandArray>::SharedPtr command_sub_;
  rclcpp::Publisher<MotorStateArray>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<EnterMitMode>::SharedPtr enter_mit_service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AkMotorDriverNode>());
  rclcpp::shutdown();
  return 0;
}
