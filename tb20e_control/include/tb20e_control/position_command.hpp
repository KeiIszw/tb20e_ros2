#ifndef TB20E_CONTROL__POSITION_COMMAND_HPP_
#define TB20E_CONTROL__POSITION_COMMAND_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

#include "tb20e_control/math_utils.hpp"

namespace tb20e_control
{
namespace position_command
{

constexpr std::size_t kAxisCount = 4;

struct Config
{
  std::array<double, kAxisCount> full_speed_rad_s{
    math::degrees_to_radians(50.0),
    math::degrees_to_radians(50.0),
    math::degrees_to_radians(50.0),
    math::degrees_to_radians(50.0)};
  std::array<double, kAxisCount> lower_rad{
    -math::kPi,
    math::degrees_to_radians(-83.0),
    math::degrees_to_radians(32.0),
    math::degrees_to_radians(-31.0)};
  std::array<double, kAxisCount> upper_rad{
    math::kPi,
    math::degrees_to_radians(48.0),
    math::degrees_to_radians(155.0),
    math::degrees_to_radians(159.0)};
};

inline std::array<double, kAxisCount> advance(
  const std::array<double, kAxisCount> & current_target,
  const std::array<double, kAxisCount> & lever_percent,
  const Config & config,
  const double period_sec)
{
  if (!std::isfinite(period_sec) || period_sec <= 0.0) {
    return current_target;
  }

  auto result = current_target;
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    if (!std::isfinite(result[axis]) ||
      !std::isfinite(lever_percent[axis]) ||
      !std::isfinite(config.full_speed_rad_s[axis]))
    {
      continue;
    }
    const double bounded_lever = std::clamp(lever_percent[axis], -100.0, 100.0);
    result[axis] +=
      bounded_lever / 100.0 * config.full_speed_rad_s[axis] * period_sec;
    if (axis == 0) {
      result[axis] = math::wrap_to_pi(result[axis]);
    } else {
      result[axis] = std::clamp(
        result[axis], config.lower_rad[axis], config.upper_rad[axis]);
    }
  }
  return result;
}

}  // namespace position_command
}  // namespace tb20e_control

#endif  // TB20E_CONTROL__POSITION_COMMAND_HPP_
