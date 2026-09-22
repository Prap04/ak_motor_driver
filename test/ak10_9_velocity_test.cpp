#include "cubemars.hpp"

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

namespace
{
volatile std::sig_atomic_t stop_requested = 0;

void requestStop(int)
{
  stop_requested = 1;
}

void printUsage(const char * program)
{
  std::cerr << "Usage: " << program
            << " [can_interface] [motor_id] [speed_rpm] [duration_s] [kd]\n"
            << "Defaults: can0 1 0.9549 3.0 1.0\n"
            << "Example for 60 RPM: " << program << " can0 1 60 3 1.0\n";
}

bool parseDouble(const char * text, double & value)
{
  char * end = nullptr;
  value = std::strtod(text, &end);
  return end != text && *end == '\0' && std::isfinite(value);
}

bool parseUnsigned(const char * text, unsigned & value)
{
  char * end = nullptr;
  const auto parsed = std::strtoul(text, &end, 10);
  if (end == text || *end != '\0' || parsed > 255) {
    return false;
  }
  value = static_cast<unsigned>(parsed);
  return true;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc == 2 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) {
    printUsage(argv[0]);
    return 0;
  }
  if (argc > 6) {
    printUsage(argv[0]);
    return 2;
  }

  const std::string can_interface = argc > 1 ? argv[1] : "can0";
  unsigned motor_id = 1;
  double speed_rpm = 0.9549296586;
  double duration_s = 3.0;
  double kd = 1.0;

  if ((argc > 2 && !parseUnsigned(argv[2], motor_id)) ||
    (argc > 3 && !parseDouble(argv[3], speed_rpm)) ||
    (argc > 4 && !parseDouble(argv[4], duration_s)) ||
    (argc > 5 && !parseDouble(argv[5], kd)) ||
    duration_s <= 0.0 || kd < 0.0 || kd > 5.0)
  {
    printUsage(argv[0]);
    return 2;
  }

  constexpr double pi = 3.14159265358979323846;
  const double velocity_rad_per_s = speed_rpm * 2.0 * pi / 60.0;
  const cubemars::MitLimits limits;
  if (velocity_rad_per_s < limits.velocity_min_rad_per_s ||
    velocity_rad_per_s > limits.velocity_max_rad_per_s)
  {
    std::cerr << "Requested speed is outside the configured AK10-9 velocity range.\n";
    return 2;
  }

  std::signal(SIGINT, requestStop);
  std::signal(SIGTERM, requestStop);

  cubemars::CubeMarsMotor motor(can_interface, static_cast<std::uint8_t>(motor_id));
  auto reportError = [&motor](const char * operation, cubemars::Error error) {
    if (error != cubemars::Error::none) {
      std::cerr << operation << " failed: " << motor.lastErrorMessage() << "\n";
      return false;
    }
    return true;
  };

  if (!reportError("connect", motor.connect())) {
    return 1;
  }
  if (!reportError("enter MIT mode", motor.enterMitMode())) {
    motor.disconnect();
    return 1;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  if (!reportError("set 0", motor.setZero())) {
    motor.disconnect();
    return 1;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  const cubemars::MotorCommand command{0.0, velocity_rad_per_s, 0.0, kd, 0.0};
  const cubemars::MotorCommand stop_command{0.0, 0.0, 0.0, kd, 0.0};
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(duration_s);

  std::cout << std::fixed << std::setprecision(3)
            << "Running motor " << motor_id << " on " << can_interface
            << " at " << speed_rpm << " RPM (" << velocity_rad_per_s
            << " rad/s) for " << duration_s << " seconds.\n";

  bool feedback_received = false;
  unsigned feedback_timeouts = 0;
  while (!stop_requested && std::chrono::steady_clock::now() < deadline) {
    if (!reportError("send command", motor.setMitCommand(command))) {
      break;
    }

    cubemars::MotorState state;
    const auto read_error = motor.readState(state);
    if (read_error == cubemars::Error::none) {
      feedback_received = true;
      const double measured_rpm = state.velocity_rad_per_s * 60.0 / (2.0 * pi);
      std::cout << "measured=" << measured_rpm << " RPM, position="
                << state.position_rad << " rad, torque=" << state.torque_nm
                << " Nm, temperature=" << state.temperature_c
                << " C, error=" << static_cast<unsigned>(state.error) << '\n';
    } else if (read_error == cubemars::Error::timeout) {
      ++feedback_timeouts;
      if (feedback_timeouts == 1) {
        std::cerr << "No valid feedback received yet; checking CAN connection.\n";
      }
    } else {
      reportError("read feedback", read_error);
      break;
    }

    // Keep command traffic bounded even when feedback arrives immediately.
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  std::cout << "Sending zero-velocity stop command.\n";
  for (int count = 0; count < 20; ++count) {
    if (!reportError("stop", motor.setMitCommand(stop_command))) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  const bool exited = reportError("exit MIT mode", motor.exitMitMode());
  motor.disconnect();
  if (!feedback_received) {
    std::cerr << "Test failed: no valid motor feedback was decoded.\n";
  }
  return exited && feedback_received ? 0 : 1;
}
