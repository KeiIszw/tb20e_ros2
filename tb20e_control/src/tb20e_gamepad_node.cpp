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
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tb20e_control/gamepad_mapping.hpp"

namespace tb20e_control
{

class Tb20eGamepadNode final : public rclcpp::Node
{
public:
  Tb20eGamepadNode()
  : Node("tb20e_gamepad"), last_joy_time_(std::chrono::steady_clock::now())
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

    validate_parameters();

    command_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/tb20e_gamepad_controller/commands", rclcpp::QoS(10).reliable());
    joy_subscription_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", rclcpp::SensorDataQoS().keep_last(10),
      std::bind(&Tb20eGamepadNode::joy_callback, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&Tb20eGamepadNode::publish_command, this));

    RCLCPP_INFO(
      get_logger(),
      "Gamepad mapping ready: left X=swing, left Y=arm, right X=bucket, "
      "right Y=boom (timeout %.3f s)",
      joy_timeout_sec_);
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
    bool timed_out = false;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      const double age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_joy_time_).count();
      timed_out = !have_valid_joy_ || age > joy_timeout_sec_;
      if (!timed_out) {
        command = latest_command_;
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
  }

  gamepad::Mapping mapping_;
  double joy_timeout_sec_{0.25};
  double publish_rate_{20.0};
  int deadman_button_{-1};

  std::mutex command_mutex_;
  std::array<double, gamepad::kCommandCount> latest_command_{};
  std::chrono::steady_clock::time_point last_joy_time_;
  bool have_valid_joy_{false};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
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
