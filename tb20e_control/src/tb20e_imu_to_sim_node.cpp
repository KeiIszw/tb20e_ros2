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
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tb20e_control/math_utils.hpp"

namespace tb20e_control
{

class Tb20eImuToSimNode final : public rclcpp::Node
{
public:
  static constexpr std::size_t kAxisCount = 4;

  Tb20eImuToSimNode()
  : Node("tb20e_imu_to_sim")
  {
    const std::array<std::string, kAxisCount> axis_names{
      "swing", "boom", "arm", "bucket"};

    for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
      const auto & name = axis_names[axis];
      state_topics_[axis] = declare_parameter<std::string>(
        name + "_state_topic", "/TB20e_0/current_" + name + "_angle");
      command_topics_[axis] = declare_parameter<std::string>(
        name + "_sim_command_topic", "/TB20e/" + name + "/cmd");
      position_signs_[axis] = declare_parameter<double>(
        name + "_sim_position_sign", axis == 0 ? -1.0 : 1.0);

      if (state_topics_[axis].empty() || command_topics_[axis].empty()) {
        throw std::invalid_argument("IMU state and simulator command topics must not be empty");
      }
      if (!std::isfinite(position_signs_[axis]) ||
        std::abs(position_signs_[axis]) != 1.0)
      {
        throw std::invalid_argument("simulator position signs must be either -1 or 1");
      }
    }

    const auto feedback_qos = rclcpp::SensorDataQoS().keep_last(10);
    for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
      publishers_[axis] = create_publisher<std_msgs::msg::Float64>(
        command_topics_[axis], rclcpp::QoS(10).reliable());
      subscriptions_[axis] = create_subscription<std_msgs::msg::Float64>(
        state_topics_[axis], feedback_qos,
        [this, axis](const std_msgs::msg::Float64::ConstSharedPtr message) {
          publish_sim_command(axis, message);
        });
      RCLCPP_INFO(
        get_logger(), "%s (degree) -> %s (radian), sign %.0f",
        state_topics_[axis].c_str(), command_topics_[axis].c_str(),
        position_signs_[axis]);
    }
  }

private:
  void publish_sim_command(
    const std::size_t axis,
    const std_msgs::msg::Float64::ConstSharedPtr message)
  {
    if (!std::isfinite(message->data)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring a non-finite hardware IMU angle");
      return;
    }

    std_msgs::msg::Float64 command;
    command.data = position_signs_[axis] * math::degrees_to_radians(message->data);
    publishers_[axis]->publish(command);
  }

  std::array<std::string, kAxisCount> state_topics_{};
  std::array<std::string, kAxisCount> command_topics_{};
  std::array<double, kAxisCount> position_signs_{};
  std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, kAxisCount>
  publishers_{};
  std::array<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr, kAxisCount>
  subscriptions_{};
};

}  // namespace tb20e_control

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<tb20e_control::Tb20eImuToSimNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("tb20e_imu_to_sim"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
