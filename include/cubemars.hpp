#pragma once //Must be included only once per compilation unit since it contains declarations of the CubeMarsMotor class and related functions.

#include <array>// used for the excat 8 byte CAN frame data representation
#include <cstdint>// used for fixed-width integer types
#include <string>// used for string handling

namespace cubemars// all the drivers code is encapsulated in the cubemars namespace to avoid name collisions with other libraries or code
{

struct MitLimits// structure to hold the limits for the motor's position, velocity, kp, kd, and torque
{
  double position_min_rad{-12.5};
  double position_max_rad{12.5};
  double velocity_min_rad_per_s{-50.0};
  double velocity_max_rad_per_s{50.0};
  double kp_min{0.0};
  double kp_max{500.0};
  double kd_min{0.0};
  double kd_max{5.0};
  double torque_min_nm{-65.0};
  double torque_max_nm{65.0};
};

struct MotorCommand// structure to hold the command values for the motor just a c++ object 
{
  double position_rad{0.0};
  double velocity_rad_per_s{0.0};
  double kp{0.0};
  double kd{0.0};
  double torque_nm{0.0};
};

struct MotorState// structure to hold the state values for the motor  
{
  double position_rad{0.0};
  double velocity_rad_per_s{0.0};
  double torque_nm{0.0};
  double temperature_c{0.0};
  std::uint8_t motor_id{0};
  std::uint8_t error{0};
  bool valid{false};
};

enum class Error //it is a type created by the project to represent different error states that can occur during the operation of the class MotorState.
{
  none,
  already_connected,
  not_connected,
  invalid_range,
  socket_create_failed,
  interface_lookup_failed,
  bind_failed,
  send_failed,
  receive_failed,
  timeout,
  malformed_frame,
  unexpected_can_id,
  unexpected_dlc,
  motor_fault
};

// These helpers are protocol-only and make hardware-free round-trip tests possible.
std::uint16_t floatToUint(double value, double minimum, double maximum, unsigned bits);
double uintToFloat(std::uint16_t value, double minimum, double maximum, unsigned bits);
std::array<std::uint8_t, 8> encodeMitCommand(const MotorCommand & command, const MitLimits & limits);
bool decodeMitFeedback(
  const std::array<std::uint8_t, 8> & data, std::uint8_t expected_id,
  const MitLimits & limits, MotorState & state);

class CubeMarsMotor
{
  public:
  CubeMarsMotor(
    std::string can_interface ="can0",
    std::uint8_t motor_id=1,
    MitLimits limits=MitLimits{},
    int receive_timeout_ms=20);
  // make sure to define the public member functions and variables of the CubeMarsMotor class
  Error setMitCommand(const MotorCommand & command);
  Error setPositionCommand(double position_rad,double kp,double kd);
  Error setVelocityCommand(double velocity_rad_per_s,double kd);
  Error setTorque(double torque_nm);
  ~CubeMarsMotor();

  CubeMarsMotor(const CubeMarsMotor &) = delete;
  CubeMarsMotor & operator=(const CubeMarsMotor &) = delete;

  Error connect();
  Error disconnect();
  Error enterMitMode();
  Error exitMitMode();
  Error setZero();
  Error readState(MotorState & state);

  Error lastError() const noexcept;
  const std::string & lastErrorMessage() const noexcept;
  bool connected() const noexcept;
  std::uint8_t motorId() const noexcept;

private:
  Error sendSpecialCommand(std::uint8_t final_byte);
  Error sendData(const std::array<std::uint8_t, 8> & data);
  Error fail(Error error, const std::string & message);

  std::string can_interface_;
  std::uint8_t motor_id_;
  MitLimits limits_;
  int receive_timeout_ms_;
  int socket_fd_{-1};
  Error last_error_{Error::none};
  std::string last_error_message_;
};

}  // namespace cubemars