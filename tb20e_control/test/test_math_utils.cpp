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
#include "tb20e_control/feedback_velocity.hpp"

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

TEST(FeedbackVelocity, FirstReadUsesSampleIntervalInsteadOfTinyControllerPeriod)
{
  tb20e_control::FeedbackVelocity estimator;
  const auto t = std::chrono::steady_clock::time_point{};
  estimator.reset(0.0, t);
  // 0.2 degrees in 5 ms = 40 deg/s; a first-loop 1 us period would falsely trip.
  EXPECT_FALSE(
    estimator.update(
      degrees_to_radians(0.2), t + std::chrono::milliseconds(5), false,
      degrees_to_radians(180.0)));
  EXPECT_NEAR(estimator.velocity(), degrees_to_radians(40.0), 1e-12);
  EXPECT_FALSE(
    estimator.update(
      degrees_to_radians(0.2), t + std::chrono::milliseconds(5), false,
      degrees_to_radians(180.0)));
  EXPECT_NEAR(estimator.velocity(), degrees_to_radians(40.0), 1e-12);
}

TEST(FeedbackVelocity, RealOverspeedStillTripsAndReactivationResetsBaseline)
{
  tb20e_control::FeedbackVelocity estimator;
  const auto t = std::chrono::steady_clock::time_point{};
  estimator.reset(0.0, t);
  EXPECT_TRUE(
    estimator.update(
      degrees_to_radians(20.0), t + std::chrono::milliseconds(50), false,
      degrees_to_radians(180.0)));
  EXPECT_DOUBLE_EQ(estimator.velocity(), 0.0);
  estimator.reset(degrees_to_radians(90.0), t + std::chrono::seconds(1));
  EXPECT_FALSE(
    estimator.update(
      degrees_to_radians(90.2), t + std::chrono::milliseconds(1005), false,
      degrees_to_radians(180.0)));
  EXPECT_NEAR(estimator.velocity(), degrees_to_radians(40.0), 1e-10);
}

TEST(FeedbackVelocity, SwingWrapUsesShortestDistance)
{
  tb20e_control::FeedbackVelocity estimator;
  const auto t = std::chrono::steady_clock::time_point{};
  estimator.reset(degrees_to_radians(179.0), t);
  EXPECT_FALSE(
    estimator.update(
      degrees_to_radians(-179.0), t + std::chrono::milliseconds(50), true,
      degrees_to_radians(180.0)));
  EXPECT_NEAR(estimator.velocity(), degrees_to_radians(40.0), 1e-10);
}

TEST(FeedbackVelocity, CompressedUnityDeliveryDoesNotTrip)
{
  tb20e_control::FeedbackVelocity estimator;
  const auto t = std::chrono::steady_clock::time_point{};
  estimator.reset(0.0, t);
  // 100 deg/s source, delivered at alternating 39 ms / 1 ms intervals.
  for (int i = 1; i <= 100; ++i) {
    const int receipt_ms = i * 20 + (i % 2 ? 19 : 0);
    EXPECT_FALSE(
      estimator.update(
        degrees_to_radians(i * 2.0), t + std::chrono::milliseconds(receipt_ms),
        false, degrees_to_radians(180.0)));
  }
}

TEST(FeedbackVelocity, SustainedOverspeedAccumulatesEvenWithReversals)
{
  for (const bool reverse : {false, true}) {
    tb20e_control::FeedbackVelocity estimator;
    const auto t = std::chrono::steady_clock::time_point{};
    estimator.reset(0.0, t);
    // 240 deg/s exceeds the 5.4 degree jitter budget after five 20 ms samples.
    for (int i = 1; i <= 5; ++i) {
      const double position = reverse ? (i % 2) * 4.8 : i * 4.8;
      EXPECT_EQ(
        estimator.update(
          degrees_to_radians(position), t + std::chrono::milliseconds(i * 20),
          false, degrees_to_radians(180.0)), i == 5);
    }
  }
}

TEST(FeedbackVelocity, ZeroTolerancePreservesStrictCheckAndOldStampsFault)
{
  tb20e_control::FeedbackVelocity estimator;
  const auto t = std::chrono::steady_clock::time_point{};
  estimator.reset(0.0, t);
  EXPECT_TRUE(
    estimator.update(
      degrees_to_radians(2.0), t + std::chrono::milliseconds(1), false,
      degrees_to_radians(180.0), 0.0));
  EXPECT_TRUE(estimator.update(0.0, t, false, degrees_to_radians(180.0)));
}

TEST(FeedbackVelocity, IdleTimeDoesNotBankCreditForLaterJump)
{
  tb20e_control::FeedbackVelocity estimator;
  const auto t = std::chrono::steady_clock::time_point{};
  estimator.reset(0.0, t);
  EXPECT_FALSE(
    estimator.update(
      0.0, t + std::chrono::seconds(10), false,
      degrees_to_radians(180.0)));
  EXPECT_TRUE(
    estimator.update(
      degrees_to_radians(10.0), t + std::chrono::milliseconds(10001), false,
      degrees_to_radians(180.0)));
}

TEST(LeverCompensation, PositiveOnlyAndHysteresis)
{
  using tb20e_control::math::compensated_lever_command;
  int direction = 0;
  auto output = [&](double command) {
      return compensated_lever_command(command, 45.0, 0.0, 2.0, 1.0, direction);
    };
  EXPECT_DOUBLE_EQ(output(0.0), 0.0);
  EXPECT_DOUBLE_EQ(output(1.5), 0.0);
  EXPECT_DOUBLE_EQ(output(2.0), 45.0);
  EXPECT_DOUBLE_EQ(output(1.5), 45.0);
  EXPECT_DOUBLE_EQ(output(1.0), 0.0);
  EXPECT_DOUBLE_EQ(output(1.5), 0.0);
  EXPECT_DOUBLE_EQ(output(70.0), 70.0);
  EXPECT_DOUBLE_EQ(output(-0.5), -0.5);
  EXPECT_DOUBLE_EQ(output(1.5), 0.0);
  EXPECT_DOUBLE_EQ(output(0.0), 0.0);
}

TEST(LeverCompensation, SignBoundsReversalAndInvalidInput)
{
  using tb20e_control::math::compensated_lever_command;
  using tb20e_control::math::bounded_lever_command;
  int direction = 0;
  auto output = [&](double command) {
      return compensated_lever_command(
        bounded_lever_command(command, -1.0, -80.0, 90.0),
        45.0, 50.0, 2.0, 1.0, direction);
    };
  EXPECT_DOUBLE_EQ(output(3.0), -50.0);
  EXPECT_DOUBLE_EQ(output(-1.5), 0.0);
  EXPECT_DOUBLE_EQ(output(-2.0), 45.0);
  EXPECT_DOUBLE_EQ(output(-200.0), 90.0);
  EXPECT_DOUBLE_EQ(output(200.0), -80.0);
  EXPECT_DOUBLE_EQ(output(0.0), 0.0);
  EXPECT_EQ(direction, 0);
  EXPECT_DOUBLE_EQ(
    compensated_lever_command(
      std::numeric_limits<double>::quiet_NaN(), 45.0, 50.0, 2.0, 1.0, direction), 0.0);
  EXPECT_DOUBLE_EQ(compensated_lever_command(5.0, 45.0, 50.0, 1.0, 2.0, direction), 0.0);
  EXPECT_DOUBLE_EQ(compensated_lever_command(0.5, 0.0, 0.0, 2.0, 1.0, direction), 0.5);
}
