// Copyright 2026 tb20e_ros2 contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TB20E_CONTROL__MATH_UTILS_HPP_
#define TB20E_CONTROL__MATH_UTILS_HPP_

#include <algorithm>
#include <cmath>

namespace tb20e_control
{
namespace math
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kDegreesToRadians = kPi / 180.0;

inline double degrees_to_radians(const double degrees)
{
  return degrees * kDegreesToRadians;
}

inline double wrap_to_pi(const double radians)
{
  return std::remainder(radians, kTwoPi);
}

inline double shortest_angular_delta(const double current, const double previous)
{
  return std::remainder(current - previous, kTwoPi);
}

inline double bounded_lever_command(
  const double controller_command,
  const double lever_sign,
  const double lever_min,
  const double lever_max)
{
  if (!std::isfinite(controller_command) || !std::isfinite(lever_sign) ||
    !std::isfinite(lever_min) || !std::isfinite(lever_max))
  {
    return 0.0;
  }
  return std::clamp(lever_sign * controller_command, lever_min, lever_max);
}

inline bool position_outside_limits(
  const double position,
  const double position_min,
  const double position_max,
  const double tolerance)
{
  return position < position_min - tolerance || position > position_max + tolerance;
}

inline bool command_points_outside_limit(
  const double position,
  const double controller_command,
  const double position_min,
  const double position_max)
{
  return (position <= position_min && controller_command < 0.0) ||
         (position >= position_max && controller_command > 0.0);
}

inline bool velocity_exceeds_limit(
  const double position_delta,
  const double period_sec,
  const double velocity_limit)
{
  if (!std::isfinite(position_delta) || !std::isfinite(period_sec) ||
    !std::isfinite(velocity_limit) || period_sec <= 0.0 || velocity_limit <= 0.0)
  {
    return true;
  }
  return std::abs(position_delta / period_sec) > velocity_limit;
}

}  // namespace math
}  // namespace tb20e_control

#endif  // TB20E_CONTROL__MATH_UTILS_HPP_
