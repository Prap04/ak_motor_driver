#include "cubemars.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <sstream>

namespace cubemars
{
namespace
{
constexpr std::uint8_t kEnterMit = 0xFC;//constexpr is used to define a constant value when the compiler is able to evaluate the value at compile time. It is used here to define the command byte for entering the Mit mode of the motor.
constexpr std::uint8_t kExitMit = 0xFD;//constexpr is used to define a constant value when the compiler is able to evaluate the value at compile time. It is used here to define the command byte for exiting the Mit mode of the motor.
constexpr std::uint8_t kSetZero = 0xFE;//constexpr is used to define a constant value when the compiler is able to evaluate the value at compile time. It is used here to define the command byte for setting the motor position to zero.
constexpr std::uint8_t kDlc = 8;

bool inRange(double value, double minimum, double maximum)
{
  return std::isfinite(value) && value >= minimum && value <= maximum;
}

std::uint16_t maskForBits(unsigned bits)
{
  return static_cast<std::uint16_t>((1u << bits) - 1u);
}
}  // namespace

std::uint16_t floatToUint(double value, double minimum, double maximum, unsigned bits)
{
  if (bits == 0 || bits > 16 || !std::isfinite(value) || !(maximum > minimum)) {
    return 0;
  }
  // The manual's float_to_uint() uses 2^bits/span and truncates the result.
  const double scale = static_cast<double>(1u << bits) / (maximum - minimum);
  const double scaled = (value - minimum) * scale;
  return static_cast<std::uint16_t>(std::clamp(std::floor(scaled), 0.0,
    static_cast<double>(maskForBits(bits))));
}

double uintToFloat(std::uint16_t value, double minimum, double maximum, unsigned bits)
{
  if (bits == 0 || bits > 16 || !(maximum > minimum)) {
    return minimum;
  }
  const auto maximum_integer = static_cast<double>(maskForBits(bits));
  return static_cast<double>(value) * (maximum - minimum) / maximum_integer + minimum;
}

std::array<std::uint8_t, 8> encodeMitCommand(const MotorCommand & command, const MitLimits & limits)
{
  const auto position = floatToUint(command.position_rad, limits.position_min_rad,
    limits.position_max_rad, 16);
  const auto velocity = floatToUint(command.velocity_rad_per_s, limits.velocity_min_rad_per_s,
    limits.velocity_max_rad_per_s, 12);
  const auto kp = floatToUint(command.kp, limits.kp_min, limits.kp_max, 12);
  const auto kd = floatToUint(command.kd, limits.kd_min, limits.kd_max, 12);
  const auto torque = floatToUint(command.torque_nm, limits.torque_min_nm,
    limits.torque_max_nm, 12);

  return {
    static_cast<std::uint8_t>(position >> 8), static_cast<std::uint8_t>(position & 0xFF),
    static_cast<std::uint8_t>(velocity >> 4),
    static_cast<std::uint8_t>(((velocity & 0x0F) << 4) | (kp >> 8)),
    static_cast<std::uint8_t>(kp & 0xFF), static_cast<std::uint8_t>(kd >> 4),
    static_cast<std::uint8_t>(((kd & 0x0F) << 4) | (torque >> 8)),
    static_cast<std::uint8_t>(torque & 0xFF)};
}

bool decodeMitFeedback(
  const std::array<std::uint8_t, 8> & data, std::uint8_t expected_id,
  const MitLimits & limits, MotorState & state)
{
  // The manual's executable unpack_reply() uses DATA[0] as ID. Its preceding
  // table conflicts with this and appears to omit the ID byte.
  if (data[0] != expected_id) {
    return false;
  }
  const auto position = static_cast<std::uint16_t>((data[1] << 8) | data[2]);
  const auto velocity = static_cast<std::uint16_t>((data[3] << 4) | (data[4] >> 4));
  const auto torque = static_cast<std::uint16_t>(((data[4] & 0x0F) << 8) | data[5]);

  state.motor_id = data[0];
  state.position_rad = uintToFloat(position, limits.position_min_rad, limits.position_max_rad, 16);
  state.velocity_rad_per_s = uintToFloat(
    velocity, limits.velocity_min_rad_per_s, limits.velocity_max_rad_per_s, 12);
  state.torque_nm = uintToFloat(torque, limits.torque_min_nm, limits.torque_max_nm, 12);
  state.temperature_c = static_cast<double>(data[6]) - 40.0;
  state.error = data[7];
  state.valid = true;
  return true;
}

CubeMarsMotor::CubeMarsMotor(
  std::string can_interface, 
  std::uint8_t motor_id, 
  MitLimits limits, 
  int receive_timeout_ms)
: can_interface_(std::move(can_interface)),
  motor_id_(motor_id),
  limits_(limits),
  receive_timeout_ms_(receive_timeout_ms)
{
}

CubeMarsMotor::~CubeMarsMotor()
{
  disconnect();
}

Error CubeMarsMotor::setPositionCommand(double position_rad,double kp,double kd)
{
  MotorCommand cmd{};
  cmd.position_rad=position_rad;
  cmd.velocity_rad_per_s=0.0;
  cmd.kp=kp;
  cmd.kd=kd;
  cmd.torque_nm=0.0;
  return setMitCommand(cmd);
}
Error CubeMarsMotor::setVelocityCommand(double velocity_rad_per_s,double kd)
{
  MotorCommand cmd{};
  cmd.position_rad=0.0;
  cmd.velocity_rad_per_s=velocity_rad_per_s;
  cmd.kp=0.0;
  cmd.kd=kd;
  cmd.torque_nm=0.0;
  return setMitCommand(cmd);
}
Error CubeMarsMotor::setTorque(double torque_nm)
{
  MotorCommand cmd{};
  cmd.position_rad=0.0;
  cmd.velocity_rad_per_s=0.0;
  cmd.kp=0.0; //Zero Kp
  cmd.kd=0.0; //Zero Kd
  cmd.torque_nm=torque_nm;
  return setMitCommand(cmd);
}

Error CubeMarsMotor::fail(Error error, const std::string & message)
{
  last_error_ = error;
  last_error_message_ = message;
  return error;
}

Error CubeMarsMotor::connect()
{
  if (connected()) {
    return fail(Error::already_connected, "CAN socket is already open");
  }
  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    return fail(Error::socket_create_failed, std::strerror(errno));
  }

  ifreq interface_request{};
  std::strncpy(interface_request.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
  if (ioctl(socket_fd_, SIOCGIFINDEX, &interface_request) < 0) {
    close(socket_fd_);
    socket_fd_ = -1;
    return fail(Error::interface_lookup_failed, std::strerror(errno));
  }

  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = interface_request.ifr_ifindex;
  if (bind(socket_fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
    close(socket_fd_);
    socket_fd_ = -1;
    return fail(Error::bind_failed, std::strerror(errno));
  }
  last_error_ = Error::none;
  last_error_message_.clear();
  return Error::none;
}
Error CubeMarsMotor::disconnect()
{
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
  return Error::none;
}

Error CubeMarsMotor::sendData(const std::array<std::uint8_t, 8> & data)
{
  if (!connected()) {
    return fail(Error::not_connected, "CAN socket is not open");
  }
  can_frame frame{};
  frame.can_id = motor_id_;  // MIT uses a standard identifier equal to motor ID.
  frame.can_dlc = kDlc;
  std::copy(data.begin(), data.end(), frame.data);
  if (write(socket_fd_, &frame, sizeof(frame)) != static_cast<ssize_t>(sizeof(frame))) {
    return fail(Error::send_failed, std::strerror(errno));
  }
  return Error::none;
}

Error CubeMarsMotor::sendSpecialCommand(std::uint8_t final_byte)
{
  std::array<std::uint8_t, 8> data{};
  data.fill(0xFF);
  data.back() = final_byte;
  return sendData(data);
}

Error CubeMarsMotor::enterMitMode() { return sendSpecialCommand(kEnterMit); }
Error CubeMarsMotor::exitMitMode() { return sendSpecialCommand(kExitMit); }
Error CubeMarsMotor::setZero() { return sendSpecialCommand(kSetZero); }

Error CubeMarsMotor::setMitCommand(const MotorCommand & command)
{
  if (!inRange(command.position_rad, limits_.position_min_rad, limits_.position_max_rad) ||
    !inRange(command.velocity_rad_per_s, limits_.velocity_min_rad_per_s,
      limits_.velocity_max_rad_per_s) || !inRange(command.kp, limits_.kp_min, limits_.kp_max) ||
    !inRange(command.kd, limits_.kd_min, limits_.kd_max) ||
    !inRange(command.torque_nm, limits_.torque_min_nm, limits_.torque_max_nm)) {
    return fail(Error::invalid_range, "MIT command is outside configured limits");
  }
  return sendData(encodeMitCommand(command, limits_));
}

Error CubeMarsMotor::readState(MotorState & state)
{
  if (!connected()) {
    state.valid = false;
    return fail(Error::not_connected, "CAN socket is not open");
  }
  pollfd descriptor{socket_fd_, POLLIN, 0};
  const int poll_result = poll(&descriptor, 1, receive_timeout_ms_);
  if (poll_result == 0) {
    state.valid = false;
    return fail(Error::timeout, "Timed out waiting for MIT feedback");
  }
  if (poll_result < 0) {
    state.valid = false;
    return fail(Error::receive_failed, std::strerror(errno));
  }

  can_frame frame{};
  if (read(socket_fd_, &frame, sizeof(frame)) != static_cast<ssize_t>(sizeof(frame))) {
    state.valid = false;
    return fail(Error::receive_failed, std::strerror(errno));
  }
  if ((frame.can_id & CAN_EFF_FLAG) != 0) {
    state.valid = false;
    return fail(Error::malformed_frame, "MIT feedback must use a standard CAN frame");
  }
  if ((frame.can_id & CAN_SFF_MASK) != motor_id_) {
    state.valid = false;
    return fail(Error::unexpected_can_id, "Feedback CAN ID does not match motor ID");
  }
  if (frame.can_dlc != kDlc) {
    state.valid = false;
    return fail(Error::unexpected_dlc, "MIT feedback DLC is not 8");
  }

  std::array<std::uint8_t, 8> data{};
  std::copy(frame.data, frame.data + kDlc, data.begin());
  if (!decodeMitFeedback(data, motor_id_, limits_, state)) {
    state.valid = false;
    return fail(Error::malformed_frame, "Feedback motor ID does not match configured motor");
  }
  if (state.error != 0) {
    return fail(Error::motor_fault, "Motor reported a non-zero fault code");
  }
  last_error_ = Error::none;
  last_error_message_.clear();
  return Error::none;
}

Error CubeMarsMotor::lastError() const noexcept { return last_error_; }
const std::string & CubeMarsMotor::lastErrorMessage() const noexcept { return last_error_message_; }
bool CubeMarsMotor::connected() const noexcept { return socket_fd_ >= 0; }
std::uint8_t CubeMarsMotor::motorId() const noexcept { return motor_id_; }

}  // namespace cubemars