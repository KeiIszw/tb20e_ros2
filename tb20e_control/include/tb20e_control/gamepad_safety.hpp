// Copyright 2026 tb20e_ros2 contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef TB20E_CONTROL__GAMEPAD_SAFETY_HPP_
#define TB20E_CONTROL__GAMEPAD_SAFETY_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include "tb20e_control/gamepad_mapping.hpp"

namespace tb20e_control
{
namespace gamepad
{

// Arming requires a neutral interval and a fresh button press. A second press
// stops immediately, including when a stick is away from neutral.
class NeutralToggleGate
{
public:
  explicit NeutralToggleGate(double hold_sec, double timeout_sec)
  : hold_sec_(hold_sec), timeout_sec_(timeout_sec)
  {}

  void reset()
  {
    state_ = State::WaitingNeutral;
    neutral_since_ = std::numeric_limits<double>::quiet_NaN();
    last_sample_time_ = std::numeric_limits<double>::quiet_NaN();
    previous_button_pressed_ = false;
  }

  // Returns true only when the current sample is allowed to command motion.
  bool update(
    const std::vector<float> & axes, const Mapping & mapping,
    bool button_pressed, double now_sec)
  {
    if (!std::isfinite(now_sec) || !axis_indices_are_valid(axes, mapping)) {
      reset();
      return false;
    }
    const std::array<int, kCommandCount> indices{
      mapping.swing_axis, mapping.boom_axis, mapping.arm_axis, mapping.bucket_axis};
    for (const int index : indices) {
      if (!std::isfinite(axes[index])) {
        reset();
        return false;
      }
    }
    if (std::isfinite(last_sample_time_) &&
      (now_sec < last_sample_time_ || now_sec - last_sample_time_ > timeout_sec_))
    {
      reset();
    }
    last_sample_time_ = now_sec;
    const bool button_rising = button_pressed && !previous_button_pressed_;
    previous_button_pressed_ = button_pressed;

    const bool neutral = std::all_of(
      indices.begin(), indices.end(), [&](int index) {
        return std::abs(axes[index]) <= mapping.deadzone;
      });

    if (state_ == State::Active) {
      if (button_rising) {
        reset();
        last_sample_time_ = now_sec;
        previous_button_pressed_ = button_pressed;
        return false;
      }
      return true;
    }

    if (!neutral || (button_pressed && state_ == State::WaitingNeutral))
    {
      state_ = State::WaitingNeutral;
      neutral_since_ = std::numeric_limits<double>::quiet_NaN();
      return false;
    }

    if (state_ == State::WaitingNeutral) {
      if (!std::isfinite(neutral_since_)) {
        neutral_since_ = now_sec;
      }
      if (now_sec - neutral_since_ >= hold_sec_) {
        state_ = State::WaitingPress;
      }
      return false;
    }

    if (button_rising) {
      state_ = State::Active;
    }
    return false;
  }

  bool armed() const {return state_ == State::Active;}

private:
  enum class State {WaitingNeutral, WaitingPress, Active};
  State state_{State::WaitingNeutral};
  double hold_sec_;
  double timeout_sec_;
  double neutral_since_{std::numeric_limits<double>::quiet_NaN()};
  double last_sample_time_{std::numeric_limits<double>::quiet_NaN()};
  bool previous_button_pressed_{false};
};

}  // namespace gamepad
}  // namespace tb20e_control

#endif  // TB20E_CONTROL__GAMEPAD_SAFETY_HPP_
