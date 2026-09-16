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

#ifndef TB20E_CONTROL__FEEDBACK_VELOCITY_HPP_
#define TB20E_CONTROL__FEEDBACK_VELOCITY_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include "tb20e_control/math_utils.hpp"

namespace tb20e_control
{

// Float64 feedback has no source stamp; use steady-clock receipt times of
// the two samples, never the controller-manager period (especially at startup).
class FeedbackVelocity
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;

  void reset(double position, TimePoint stamp)
  {
    position_ = position;
    stamp_ = stamp;
    velocity_ = 0.0;
    excess_distance_ = 0.0;
  }

  bool update(
    double position, TimePoint stamp, bool continuous, double limit,
    double jitter_tolerance_sec = 0.03)
  {
    if (stamp == stamp_) {
      return false;  // Same sample: retain velocity until fresh feedback arrives.
    }
    const double dt = std::chrono::duration<double>(stamp - stamp_).count();
    const double delta = continuous ? math::shortest_angular_delta(position, position_) :
      position - position_;
    if (!std::isfinite(delta) || dt <= 0.0 || !std::isfinite(limit) || limit <= 0.0 ||
      !std::isfinite(jitter_tolerance_sec) || jitter_tolerance_sec < 0.0)
    {
      velocity_ = 0.0;
      return true;
    }
    // Unstamped TCP feedback can arrive in bursts. Carry excess travel across
    // samples, allowing only a bounded amount of receipt-time compression.
    // Slow/stationary periods repay excess but never bank unlimited credit.
    // Absolute travel prevents reversals from cancelling an overspeed.
    excess_distance_ = std::max(0.0, excess_distance_ + std::abs(delta) - limit * dt);
    const bool fault = excess_distance_ > limit * jitter_tolerance_sec;
    position_ = position;
    stamp_ = stamp;
    velocity_ = fault ? 0.0 : delta / dt;
    return fault;
  }

  double velocity() const {return velocity_;}

private:
  double position_{0.0};
  TimePoint stamp_{};
  double velocity_{0.0};
  double excess_distance_{0.0};
};

}  // namespace tb20e_control

#endif  // TB20E_CONTROL__FEEDBACK_VELOCITY_HPP_
