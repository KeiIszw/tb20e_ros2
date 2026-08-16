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
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tb20e_control/gamepad_mapping.hpp"
#include "tb20e_control/math_utils.hpp"
#include "tb20e_control/position_command.hpp"

namespace tb20e_control
{

class Tb20eGamepadNode final : public rclcpp::Node
{
public:
  Tb20eGamepadNode()
  : Node("tb20e_gamepad"), last_joy_time_(std::chrono::steady_clock::now()),
    last_position_update_(std::chrono::steady_clock::now())
  {
    mapping_.swing_axis = declare_parameter<int>("swing_axis", 0);
    mapping_.arm_axis = declare_parameter<int>("arm_axis", 1);
    mapping_.bucket_axis = declare_parameter<int>("bucket_axis", 2);
    mapping_.boom_axis = declare_parameter<int>("boom_axis", 3);
    mapping_.swing_scale = declare_parameter<double>("swing_scale", 100.0);
    mapping_.arm_scale = declare_parameter<double>("arm_scale", -100.0);
    mapping_.bucket_scale = declare_parameter<double>("bucket_scale", 100.0);
    mapping_.boom_scale = declare_parameter<double>("boom_scale", 100.0);
    mapping_.deadzone = declare_parameter<double>("deadzone", 0.10);
    joy_timeout_sec_ = declare_parameter<double>("joy_timeout_sec", 0.25);
    publish_rate_ = declare_parameter<double>("publish_rate", 20.0);
    deadman_button_ = declare_parameter<int>("deadman_button", -1);
    unity_position_output_enabled_ =
      declare_parameter<bool>("unity_position_output_enabled", false);
    sim_feedback_timeout_sec_ =
      declare_parameter<double>("sim_feedback_timeout_sec", 0.25);

    const std::array<std::string, gamepad::kCommandCount> axis_names{
      "swing", "boom", "arm", "bucket"};
    for (std::size_t axis = 0; axis < gamepad::kCommandCount; ++axis) {
      const auto & name = axis_names[axis];
      sim_state_topics_[axis] = declare_parameter<std::string>(
        name + "_sim_state_topic", "/sim/tb20e/current_" + name + "_angle");
      unity_command_topics_[axis] = declare_parameter<std::string>(
        name + "_unity_command_topic", "/tb20e/" + name + "/cmd");
      if (!unity_command_topics_[axis].empty() &&
        unity_command_topics_[axis].front() != '/')
      {
        unity_command_topics_[axis].insert(0, "/");
      }
      unity_position_signs_[axis] = declare_parameter<double>(
        name + "_unity_position_sign", axis == 0 ? -1.0 : 1.0);
      const double speed_deg_s = declare_parameter<double>(
        name + "_unity_speed_deg_s", 50.0);
      position_config_.full_speed_rad_s[axis] =
        math::degrees_to_radians(speed_deg_s);
    }

    validate_parameters();

    command_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/tb20e_gamepad_controller/commands", rclcpp::QoS(10).reliable());
    joy_subscription_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", rclcpp::SensorDataQoS().keep_last(10),
      std::bind(&Tb20eGamepadNode::joy_callback, this, std::placeholders::_1));

    if (unity_position_output_enabled_) {
      const auto feedback_qos = rclcpp::SensorDataQoS().keep_last(10);
      for (std::size_t axis = 0; axis < gamepad::kCommandCount; ++axis) {
        unity_position_publishers_[axis] =
          create_publisher<std_msgs::msg::Float64>(
          unity_command_topics_[axis], rclcpp::QoS(10).reliable());
        sim_state_subscriptions_[axis] =
          create_subscription<std_msgs::msg::Float64>(
          sim_state_topics_[axis], feedback_qos,
          [this, axis](const std_msgs::msg::Float64::ConstSharedPtr message) {
            sim_feedback_callback(axis, message);
          });
      }
    }

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&Tb20eGamepadNode::publish_command, this));

    RCLCPP_INFO(
      get_logger(),
      "Gamepad mapping ready: left X=swing, left Y=arm, right X=bucket, "
      "right Y=boom (timeout %.3f s, Unity position output %s)",
      joy_timeout_sec_, unity_position_output_enabled_ ? "enabled" : "disabled");
  }

private:
  void validate_parameters()
  {
    const std::array<int, gamepad::kCommandCount> indices{
      mapping_.swing_axis, mapping_.boom_axis,
      mapping_.arm_axis, mapping_.bucket_axis};
    for (const int index : indices) {
      if (index < 0) {
        throw std::invalid_argument("gamepad axis indices must not be negative");
      }
    }
    const std::array<double, gamepad::kCommandCount> scales{
      mapping_.swing_scale, mapping_.boom_scale,
      mapping_.arm_scale, mapping_.bucket_scale};
    for (const double scale : scales) {
      if (!std::isfinite(scale) || std::abs(scale) > 100.0) {
        throw std::invalid_argument("axis scales must be finite and within [-100, 100]");
      }
    }
    if (!std::isfinite(mapping_.deadzone) ||
      mapping_.deadzone < 0.0 || mapping_.deadzone >= 1.0)
    {
      throw std::invalid_argument("deadzone must satisfy 0 <= deadzone < 1");
    }
    if (!std::isfinite(joy_timeout_sec_) || joy_timeout_sec_ <= 0.0) {
      throw std::invalid_argument("joy_timeout_sec must be greater than zero");
    }
    if (!std::isfinite(publish_rate_) || publish_rate_ <= 0.0) {
      throw std::invalid_argument("publish_rate must be greater than zero");
    }
    if (deadman_button_ < -1) {
      throw std::invalid_argument("deadman_button must be -1 or a non-negative index");
    }
    if (!std::isfinite(sim_feedback_timeout_sec_) ||
      sim_feedback_timeout_sec_ <= 0.0)
    {
      throw std::invalid_argument("sim_feedback_timeout_sec must be greater than zero");
    }
    for (std::size_t axis = 0; axis < gamepad::kCommandCount; ++axis) {
      if (sim_state_topics_[axis].empty() || unity_command_topics_[axis].empty()) {
        throw std::invalid_argument("Unity state and command topics must not be empty");
      }
      if (!std::isfinite(position_config_.full_speed_rad_s[axis]) ||
        position_config_.full_speed_rad_s[axis] <= 0.0)
      {
        throw std::invalid_argument(
                "Unity position command speeds must be greater than zero");
      }
      if (!std::isfinite(unity_position_signs_[axis]) ||
        std::abs(unity_position_signs_[axis]) != 1.0)
      {
        throw std::invalid_argument("Unity position signs must be either -1 or 1");
      }
    }
  }

  void sim_feedback_callback(
    const std::size_t axis,
    const std_msgs::msg::Float64::ConstSharedPtr message)
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!std::isfinite(message->data)) {
      sim_feedback_received_[axis] = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Unity angle feedback is non-finite; position output is paused");
      return;
    }
    sim_feedback_positions_[axis] =
      unity_position_signs_[axis] * math::degrees_to_radians(message->data);
    sim_feedback_received_[axis] = true;
    sim_feedback_times_[axis] = std::chrono::steady_clock::now();
  }

  void joy_callback(const sensor_msgs::msg::Joy::ConstSharedPtr message)
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    const bool axes_valid = gamepad::axis_indices_are_valid(message->axes, mapping_);
    const bool deadman_valid = deadman_button_ < 0 ||
      static_cast<std::size_t>(deadman_button_) < message->buttons.size();

    if (!axes_valid || !deadman_valid) {
      latest_command_.fill(0.0);
      have_valid_joy_ = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Joy message does not contain every configured axis/button; commanding zero");
      return;
    }

    const bool deadman_pressed = deadman_button_ < 0 ||
      message->buttons[deadman_button_] != 0;
    latest_command_ = deadman_pressed ?
      gamepad::map_axes(message->axes, mapping_) :
      std::array<double, gamepad::kCommandCount>{};
    last_joy_time_ = std::chrono::steady_clock::now();
    have_valid_joy_ = true;
  }

  void publish_command()
  {
    std::array<double, gamepad::kCommandCount> command{};
    std::array<double, gamepad::kCommandCount> unity_target{};
    bool timed_out = false;
    bool publish_unity_target = false;
    bool sim_feedback_fresh = false;
    const auto now = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      const double age = std::chrono::duration<double>(
        now - last_joy_time_).count();
      timed_out = !have_valid_joy_ || age > joy_timeout_sec_;
      if (!timed_out) {
        command = latest_command_;
      }

      if (unity_position_output_enabled_) {
        sim_feedback_fresh = true;
        for (std::size_t axis = 0; axis < gamepad::kCommandCount; ++axis) {
          const double feedback_age = std::chrono::duration<double>(
            now - sim_feedback_times_[axis]).count();
          sim_feedback_fresh &= sim_feedback_received_[axis] &&
            feedback_age <= sim_feedback_timeout_sec_;
        }

        if (sim_feedback_fresh) {
          if (!position_targets_initialized_ || !sim_feedback_was_fresh_) {
            position_targets_ = sim_feedback_positions_;
            position_targets_[0] = math::wrap_to_pi(position_targets_[0]);
            position_targets_initialized_ = true;
          }
          const double period_sec = std::chrono::duration<double>(
            now - last_position_update_).count();
          if (!timed_out) {
            position_targets_ = position_command::advance(
              position_targets_, command, position_config_, period_sec);
          }
          unity_target = position_targets_;
          publish_unity_target = position_targets_initialized_;
        }
        sim_feedback_was_fresh_ = sim_feedback_fresh;
        last_position_update_ = now;
      }
    }

    if (timed_out) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No recent valid Joy input; commanding all axes to zero");
    }

    std_msgs::msg::Float64MultiArray message;
    message.data.assign(command.begin(), command.end());
    command_publisher_->publish(message);

    if (unity_position_output_enabled_ && !sim_feedback_fresh) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Unity angle feedback is missing or stale; position integration is paused");
    }
    if (publish_unity_target) {
      for (std::size_t axis = 0; axis < gamepad::kCommandCount; ++axis) {
        std_msgs::msg::Float64 target_message;
        target_message.data = unity_position_signs_[axis] * unity_target[axis];
        unity_position_publishers_[axis]->publish(target_message);
      }
    }
  }

  gamepad::Mapping mapping_;
  double joy_timeout_sec_{0.25};
  double publish_rate_{20.0};
  int deadman_button_{-1};
  bool unity_position_output_enabled_{false};
  double sim_feedback_timeout_sec_{0.25};
  position_command::Config position_config_;
  std::array<std::string, gamepad::kCommandCount> sim_state_topics_{};
  std::array<std::string, gamepad::kCommandCount> unity_command_topics_{};
  std::array<double, gamepad::kCommandCount> unity_position_signs_{};

  std::mutex command_mutex_;
  std::array<double, gamepad::kCommandCount> latest_command_{};
  std::chrono::steady_clock::time_point last_joy_time_;
  bool have_valid_joy_{false};
  std::array<double, gamepad::kCommandCount> sim_feedback_positions_{};
  std::array<bool, gamepad::kCommandCount> sim_feedback_received_{};
  std::array<std::chrono::steady_clock::time_point, gamepad::kCommandCount>
  sim_feedback_times_{};
  std::array<double, gamepad::kCommandCount> position_targets_{};
  bool position_targets_initialized_{false};
  bool sim_feedback_was_fresh_{false};
  std::chrono::steady_clock::time_point last_position_update_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr,
    gamepad::kCommandCount> unity_position_publishers_{};
  std::array<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr,
    gamepad::kCommandCount> sim_state_subscriptions_{};
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace tb20e_control

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<tb20e_control::Tb20eGamepadNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("tb20e_gamepad"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
