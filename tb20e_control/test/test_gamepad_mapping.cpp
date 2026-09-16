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

#include <vector>

#include "gtest/gtest.h"
#include "tb20e_control/gamepad_mapping.hpp"

namespace
{

TEST(GamepadMapping, MapsSticksToControllerJointOrder)
{
  const std::vector<float> axes{0.5F, -0.5F, -1.0F, 1.0F};
  tb20e_control::gamepad::Mapping mapping;
  mapping.deadzone = 0.0;

  const auto command = tb20e_control::gamepad::map_axes(axes, mapping);

  EXPECT_DOUBLE_EQ(command[0], 50.0);    // left X -> swing
  EXPECT_DOUBLE_EQ(command[1], 100.0);   // right Y -> boom
  EXPECT_DOUBLE_EQ(command[2], 50.0);    // inverted left Y -> arm
  EXPECT_DOUBLE_EQ(command[3], -100.0);  // right X -> bucket
}

TEST(GamepadMapping, AppliesAndRescalesDeadzone)
{
  EXPECT_DOUBLE_EQ(tb20e_control::gamepad::apply_deadzone(0.05, 0.1), 0.0);
  EXPECT_NEAR(
    tb20e_control::gamepad::apply_deadzone(0.55, 0.1), 0.5, 1e-12);
  EXPECT_NEAR(
    tb20e_control::gamepad::apply_deadzone(-0.55, 0.1), -0.5, 1e-12);
}

TEST(GamepadMapping, ReturnsZeroWhenAnAxisIsMissing)
{
  const std::vector<float> axes{1.0F, 1.0F};
  const auto command = tb20e_control::gamepad::map_axes(
    axes, tb20e_control::gamepad::Mapping{});

  for (const double value : command) {
    EXPECT_DOUBLE_EQ(value, 0.0);
  }
}

TEST(GamepadMapping, SupportsDirectionOverrideWithScaleSign)
{
  const std::vector<float> axes{1.0F, 1.0F, 1.0F, 1.0F};
  tb20e_control::gamepad::Mapping mapping;
  mapping.deadzone = 0.0;
  mapping.arm_scale = 25.0;

  const auto command = tb20e_control::gamepad::map_axes(axes, mapping);
  EXPECT_DOUBLE_EQ(command[2], 25.0);
}

}  // namespace
