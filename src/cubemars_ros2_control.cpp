#include "cubemars.hpp"

#include "hardware_interface/system_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <memory>
#include <string>
#include <vector>

namespace cubemars
{

class CubeMarsSystem : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS || info.joints.size() != 1) {
      return CallbackReturn::ERROR;
    }
    const auto & joint = info.joints.front();
    if (joint.command_interfaces.size() != 3 || joint.state_interfaces.size() < 3) {
      return CallbackReturn::ERROR;
    }
    std::string can_interface = "can0";
    std::uint8_t motor_id = 1;
    kp_ = 0.0;
    kd_ = 0.0;
    if (info.hardware_parameters.count("can_interface")) {
      can_interface = info.hardware_parameters.at("can_interface");
    }
    if (info.hardware_parameters.count("motor_id")) {
      motor_id = static_cast<std::uint8_t>(std::stoi(info.hardware_parameters.at("motor_id")));
    }
    if (info.hardware_parameters.count("kp")) {
      kp_ = std::stod(info.hardware_parameters.at("kp"));
    }
    if (info.hardware_parameters.count("kd")) {
      kd_ = std::stod(info.hardware_parameters.at("kd"));
    }
    motor_ = std::make_unique<CubeMarsMotor>(can_interface, motor_id);
    position_command_ = 0.0;
    velocity_command_ = 0.0;
    torque_command_ = 0.0;
    position_state_ = 0.0;
    velocity_state_ = 0.0;
    effort_state_ = 0.0;
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back(
      info_.joints[0].name, hardware_interface::HW_IF_POSITION, &position_state_);
    state_interfaces.emplace_back(
      info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &velocity_state_);
    state_interfaces.emplace_back(
      info_.joints[0].name, hardware_interface::HW_IF_EFFORT, &effort_state_);
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(
      info_.joints[0].name, hardware_interface::HW_IF_POSITION, &position_command_);
    command_interfaces.emplace_back(
      info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &velocity_command_);
    command_interfaces.emplace_back(
      info_.joints[0].name, hardware_interface::HW_IF_EFFORT, &torque_command_);
    return command_interfaces;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    return motor_->connect() == Error::none ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    motor_->disconnect();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    // Mode entry is explicit: activation does not command or move the motor.
    return motor_->enterMitMode() == Error::none ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    return motor_->exitMitMode() == Error::none ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
  }

  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    MotorState state;
    if (motor_->readState(state) != Error::none || !state.valid) {
      return hardware_interface::return_type::ERROR;
    }
    position_state_ = state.position_rad;
    velocity_state_ = state.velocity_rad_per_s;
    effort_state_ = state.torque_nm;
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    MotorCommand command;
    command.position_rad = position_command_;
    command.velocity_rad_per_s = velocity_command_;
    command.kp = kp_;
    command.kd = kd_;
    command.torque_nm = torque_command_;
    return motor_->setMitCommand(command) == Error::none ?
      hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
  }

private:
  std::unique_ptr<CubeMarsMotor> motor_;
  double position_command_{0.0};
  double velocity_command_{0.0};
  double torque_command_{0.0};
  double kp_{0.0};
  double kd_{0.0};
  double position_state_{0.0};
  double velocity_state_{0.0};
  double effort_state_{0.0};
};

}  // namespace cubemars

PLUGINLIB_EXPORT_CLASS(cubemars::CubeMarsSystem, hardware_interface::SystemInterface)