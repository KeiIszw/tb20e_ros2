// Copyright 2026 tb20e_ros2 contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef TB20E_CONTROL__TB20E_LEVER_HARDWARE_HPP_
#define TB20E_CONTROL__TB20E_LEVER_HARDWARE_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64.hpp"

namespace tb20e_control
{

class Tb20eLeverHardware final : public hardware_interface::SystemInterface
{
public:
  Tb20eLeverHardware() = default;
  ~Tb20eLeverHardware() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static constexpr std::size_t kAxisCount = 4;

  struct AxisConfig
  {
    std::string name;
    std::string joint_name;
    std::string state_topic;
    std::string command_topic;
    double lever_sign{1.0};
    double lever_min{-100.0};
    double lever_max{100.0};
    bool continuous{false};
    double position_min_rad{0.0};
    double position_max_rad{0.0};
  };

  struct FeedbackSample
  {
    double position_rad{0.0};
    bool received{false};
    std::chrono::steady_clock::time_point received_at{};
  };

  bool load_hardware_parameters();
  bool validate_joint_interfaces();
  bool start_executor();
  void stop_executor();
  void reset_feedback();
  bool all_feedback_is_fresh_locked(
    const std::chrono::steady_clock::time_point & now) const;
  void copy_feedback_to_states_locked();
  void invalidate_feedback(std::size_t axis);
  void latch_fault_and_publish_zero();
  void publish_zero_to_all_axes();
  void publish_zero_to_all_axes_locked();
  void feedback_callback(
    std::size_t axis, const std_msgs::msg::Float64::ConstSharedPtr & message);
  bool feedback_is_fresh(
    const FeedbackSample & sample,
    const std::chrono::steady_clock::time_point & now) const;

  std::array<AxisConfig, kAxisCount> axis_configs_{};
  std::array<FeedbackSample, kAxisCount> feedback_{};
  std::array<double, kAxisCount> position_states_{};
  std::array<double, kAxisCount> velocity_states_{};
  std::array<double, kAxisCount> effort_commands_{};
  std::array<bool, kAxisCount> stale_reported_{};
  std::vector<std::size_t> joint_axis_indices_;

  double state_timeout_sec_{0.1};
  double initial_feedback_wait_sec_{2.0};
  double feedback_limit_tolerance_rad_{0.03490658503988659};
  double max_feedback_velocity_rad_s_{3.14159265358979323846};

  std::mutex feedback_mutex_;
  std::condition_variable feedback_condition_;
  std::mutex command_publish_mutex_;
  std::atomic<bool> active_{false};
  std::atomic<bool> feedback_fault_latched_{false};

  rclcpp::Node::SharedPtr node_;
  std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, kAxisCount>
    command_publishers_{};
  std::array<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr, kAxisCount>
    state_subscriptions_{};
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
};

}  // namespace tb20e_control

#endif  // TB20E_CONTROL__TB20E_LEVER_HARDWARE_HPP_
