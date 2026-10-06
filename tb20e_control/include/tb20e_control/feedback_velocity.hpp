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

// Float64のフィードバックには送信元時刻がないため、2つのサンプルの
// 定常時計による受信時刻を使う。特に起動時はコントローラ管理側の周期を使わない。
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
    double jitter_tolerance_sec = 0.03, bool limit_check_enabled = true)
  {
    if (stamp == stamp_) {
      return false;  // 同じサンプルの場合、新しいフィードバックが届くまで速度を保持する。
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
    if (!limit_check_enabled) {
      position_ = position;
      stamp_ = stamp;
      velocity_ = delta / dt;
      excess_distance_ = 0.0;
      return false;
    }
    // 時刻情報のないTCPフィードバックは集中して届く場合がある。移動量の超過分を
    // サンプル間で持ち越し、受信間隔の短縮は一定範囲だけ許容する。
    // 低速・停止期間で超過分を減らすが、無制限の余裕は蓄積しない。
    // 移動量の絶対値を使い、反転によって速度超過が相殺されるのを防ぐ。
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

}  // tb20e_control名前空間

#endif  // TB20E_CONTROL__FEEDBACK_VELOCITY_HPP_
