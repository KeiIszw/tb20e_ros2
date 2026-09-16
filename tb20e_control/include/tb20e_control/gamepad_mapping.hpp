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

#ifndef TB20E_CONTROL__GAMEPAD_MAPPING_HPP_
#define TB20E_CONTROL__GAMEPAD_MAPPING_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

namespace tb20e_control
{
namespace gamepad
{

// Command order must match the ForwardCommandController joint order:
// swing, boom, arm, bucket.
constexpr std::size_t kCommandCount = 4;

struct Mapping
{
  int swing_axis{0};
  int arm_axis{1};
  int bucket_axis{2};
  int boom_axis{3};
  double swing_scale{100.0};
  double arm_scale{-100.0};
  double bucket_scale{100.0};
  double boom_scale{100.0};
  double deadzone{0.10};
};

inline double apply_deadzone(const double value, const double deadzone)
{
  const double bounded = std::clamp(value, -1.0, 1.0);
  const double magnitude = std::abs(bounded);
  if (magnitude <= deadzone) {
    return 0.0;
  }
  const double rescaled = (magnitude - deadzone) / (1.0 - deadzone);
  return std::copysign(rescaled, bounded);
}

inline bool axis_indices_are_valid(
  const std::vector<float> & axes, const Mapping & mapping)
{
  const std::array<int, kCommandCount> indices{
    mapping.swing_axis, mapping.boom_axis, mapping.arm_axis, mapping.bucket_axis};
  return std::all_of(
    indices.begin(), indices.end(), [&axes](const int index) {
      return index >= 0 && static_cast<std::size_t>(index) < axes.size();
    });
}

inline std::array<double, kCommandCount> map_axes(
  const std::vector<float> & axes, const Mapping & mapping)
{
  if (!axis_indices_are_valid(axes, mapping)) {
    return {};
  }

  return {
    apply_deadzone(axes[mapping.swing_axis], mapping.deadzone) * mapping.swing_scale,
    apply_deadzone(axes[mapping.boom_axis], mapping.deadzone) * mapping.boom_scale,
    apply_deadzone(axes[mapping.arm_axis], mapping.deadzone) * mapping.arm_scale,
    apply_deadzone(axes[mapping.bucket_axis], mapping.deadzone) * mapping.bucket_scale,
  };
}

}  // namespace gamepad
}  // namespace tb20e_control

#endif  // TB20E_CONTROL__GAMEPAD_MAPPING_HPP_
