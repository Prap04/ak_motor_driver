#include "cubemars.hpp"

#include <array>
#include <cassert>
#include <cmath>

int main()
{
  const cubemars::MitLimits limits;
  const cubemars::MotorCommand command{0.75, -4.0, 120.0, 1.2, 10.0};
  const auto bytes = cubemars::encodeMitCommand(command, limits);

  // Build the feedback layout documented by unpack_reply(), independently of the encoder.
  std::array<std::uint8_t, 8> feedback{};
  const auto position = cubemars::floatToUint(command.position_rad,
    limits.position_min_rad, limits.position_max_rad, 16);
  const auto velocity = cubemars::floatToUint(command.velocity_rad_per_s,
    limits.velocity_min_rad_per_s, limits.velocity_max_rad_per_s, 12);
  const auto torque = cubemars::floatToUint(command.torque_nm,
    limits.torque_min_nm, limits.torque_max_nm, 12);
  feedback[0] = 1;
  feedback[1] = static_cast<std::uint8_t>(position >> 8);
  feedback[2] = static_cast<std::uint8_t>(position);
  feedback[3] = static_cast<std::uint8_t>(velocity >> 4);
  feedback[4] = static_cast<std::uint8_t>(((velocity & 0x0F) << 4) | (torque >> 8));
  feedback[5] = static_cast<std::uint8_t>(torque);
  feedback[6] = 40;

  cubemars::MotorState state;
  assert(cubemars::decodeMitFeedback(feedback, 1, limits, state));
  assert(state.valid);
  assert(std::abs(state.position_rad - command.position_rad) < 0.001);
  assert(std::abs(state.velocity_rad_per_s - command.velocity_rad_per_s) < 0.03);
  assert(std::abs(state.torque_nm - command.torque_nm) < 0.04);
  assert(std::abs(state.temperature_c) < 1e-12);
  assert(bytes.size() == 8);
  return 0;
}