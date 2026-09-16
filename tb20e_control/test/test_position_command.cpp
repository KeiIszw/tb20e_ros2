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

#include <array>

#include "gtest/gtest.h"
#include "tb20e_control/position_command.hpp"

namespace
{

TEST(PositionCommand, IntegratesLeverPercentAtConfiguredSpeed)
{
  tb20e_control::position_command::Config config;
  config.full_speed_rad_s.fill(1.0);
  const std::array<double, 4> target{0.0, 0.0, 1.0, 0.0};
  const std::array<double, 4> lever{50.0, -50.0, 25.0, 0.0};

  const auto result = tb20e_control::position_command::advance(
    target, lever, config, 0.2);

  EXPECT_NEAR(result[0], 0.1, 1e-12);
  EXPECT_NEAR(result[1], -0.1, 1e-12);
  EXPECT_NEAR(result[2], 1.05, 1e-12);
  EXPECT_NEAR(result[3], 0.0, 1e-12);
}

TEST(PositionCommand, ClampsFiniteJointsAndWrapsSwing)
{
  tb20e_control::position_command::Config config;
  config.full_speed_rad_s.fill(1.0);
  const std::array<double, 4> target{
    tb20e_control::math::kPi - 0.01,
    config.lower_rad[1], config.upper_rad[2], config.lower_rad[3]};
  const std::array<double, 4> lever{100.0, -100.0, 100.0, -100.0};

  const auto result = tb20e_control::position_command::advance(
    target, lever, config, 0.1);

  EXPECT_LT(result[0], -3.0);
  EXPECT_DOUBLE_EQ(result[1], config.lower_rad[1]);
  EXPECT_DOUBLE_EQ(result[2], config.upper_rad[2]);
  EXPECT_DOUBLE_EQ(result[3], config.lower_rad[3]);
}

TEST(PositionCommand, InvalidPeriodHoldsTarget)
{
  const std::array<double, 4> target{0.1, 0.2, 0.3, 0.4};
  const std::array<double, 4> lever{100.0, 100.0, 100.0, 100.0};
  const auto result = tb20e_control::position_command::advance(
    target, lever, tb20e_control::position_command::Config{}, 0.0);
  EXPECT_EQ(result, target);
}

}  // namespace
