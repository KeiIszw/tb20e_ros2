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
