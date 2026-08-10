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

#include <limits>

#include "gtest/gtest.h"
#include "tb20e_control/math_utils.hpp"

namespace
{

using tb20e_control::math::bounded_lever_command;
using tb20e_control::math::command_points_outside_limit;
using tb20e_control::math::degrees_to_radians;
using tb20e_control::math::kPi;
using tb20e_control::math::position_outside_limits;
using tb20e_control::math::shortest_angular_delta;
using tb20e_control::math::velocity_exceeds_limit;

TEST(MathUtils, ConvertsDegreesToRadians)
{
  EXPECT_DOUBLE_EQ(degrees_to_radians(180.0), kPi);
  EXPECT_DOUBLE_EQ(degrees_to_radians(-90.0), -0.5 * kPi);
}

TEST(MathUtils, SwingDeltaCrossesWrapBoundaryByShortestPath)
{
  const double positive_crossing =
    shortest_angular_delta(degrees_to_radians(-179.0), degrees_to_radians(179.0));
  const double negative_crossing =
    shortest_angular_delta(degrees_to_radians(179.0), degrees_to_radians(-179.0));

  EXPECT_NEAR(positive_crossing, degrees_to_radians(2.0), 1e-12);
  EXPECT_NEAR(negative_crossing, degrees_to_radians(-2.0), 1e-12);
}

TEST(MathUtils, AppliesLeverSignAndClamp)
{
  EXPECT_DOUBLE_EQ(bounded_lever_command(25.0, -1.0, -100.0, 100.0), -25.0);
  EXPECT_DOUBLE_EQ(bounded_lever_command(200.0, 1.0, -100.0, 100.0), 100.0);
  EXPECT_DOUBLE_EQ(bounded_lever_command(-200.0, 1.0, -100.0, 100.0), -100.0);
  EXPECT_DOUBLE_EQ(
    bounded_lever_command(
      std::numeric_limits<double>::quiet_NaN(), 1.0, -100.0, 100.0),
    0.0);
}

TEST(MathUtils, RejectsFeedbackBeyondTolerance)
{
  const double minimum = degrees_to_radians(-83.0);
  const double maximum = degrees_to_radians(48.0);
  const double tolerance = degrees_to_radians(2.0);

  EXPECT_FALSE(position_outside_limits(minimum - tolerance, minimum, maximum, tolerance));
  EXPECT_FALSE(position_outside_limits(maximum + tolerance, minimum, maximum, tolerance));
  EXPECT_TRUE(
    position_outside_limits(
      minimum - tolerance - degrees_to_radians(0.1), minimum, maximum, tolerance));
  EXPECT_TRUE(
    position_outside_limits(
      maximum + tolerance + degrees_to_radians(0.1), minimum, maximum, tolerance));
}

TEST(MathUtils, BlocksOnlyCommandsPointingPastFiniteJointEndpoints)
{
  EXPECT_TRUE(command_points_outside_limit(-1.0, -2.0, -1.0, 1.0));
  EXPECT_FALSE(command_points_outside_limit(-1.0, 2.0, -1.0, 1.0));
  EXPECT_TRUE(command_points_outside_limit(1.0, 2.0, -1.0, 1.0));
  EXPECT_FALSE(command_points_outside_limit(1.0, -2.0, -1.0, 1.0));
}

TEST(MathUtils, DetectsImplausibleFeedbackVelocity)
{
  EXPECT_FALSE(
    velocity_exceeds_limit(
      degrees_to_radians(2.5), 0.05, degrees_to_radians(180.0)));
  EXPECT_TRUE(
    velocity_exceeds_limit(
      degrees_to_radians(10.0), 0.05, degrees_to_radians(180.0)));
  EXPECT_TRUE(
    velocity_exceeds_limit(
      degrees_to_radians(1.0), 0.0, degrees_to_radians(180.0)));
}

}  // namespace
