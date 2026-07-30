// Copyright 2026 tb20e_ros2 contributors
// SPDX-License-Identifier: Apache-2.0

#include "tb20e_control/tb20e_lever_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "tb20e_control/math_utils.hpp"

namespace
{

constexpr double kEndStopGuardRadians =
  0.5 * tb20e_control::math::kDegreesToRadians;
constexpr char kLoggerName[] = "tb20e_lever_hardware";

bool parse_finite_double(
  const hardware_interface::HardwareInfo & info,
  const std::string & key,
  const double default_value,
  double & result)
{
  const auto parameter = info.hardware_parameters.find(key);
  if (parameter == info.hardware_parameters.end()) {
    result = default_value;
    return true;
  }

  try {
    std::size_t parsed_characters = 0;
    result = std::stod(parameter->second, &parsed_characters);
    if (parsed_characters != parameter->second.size() || !std::isfinite(result)) {
      throw std::invalid_argument("not a finite number");
    }
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Hardware parameter '%s' has invalid value '%s': %s",
      key.c_str(), parameter->second.c_str(), exception.what());
    return false;
  }

  return true;
}

std::string string_parameter(
  const hardware_interface::HardwareInfo & info,
  const std::string & key,
  const std::string & default_value)
{
  const auto parameter = info.hardware_parameters.find(key);
  return parameter == info.hardware_parameters.end() ? default_value : parameter->second;
}

}  // namespace

namespace tb20e_control
{

Tb20eLeverHardware::~Tb20eLeverHardware()
{
  active_.store(false);
  publish_zero_to_all_axes();
  stop_executor();
}

hardware_interface::CallbackReturn Tb20eLeverHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  axis_configs_[0] = {
    "swing", "swing_joint", "/current_swing_angle", "/manipulated_swing_lever",
    1.0, -100.0, 100.0, true, 0.0, 0.0};
  axis_configs_[1] = {
    "boom", "boom_joint", "/current_boom_angle", "/manipulated_boom_lever",
    -1.0, -100.0, 100.0, false,
    math::degrees_to_radians(-83.0), math::degrees_to_radians(48.0)};
  axis_configs_[2] = {
    "arm", "arm_joint", "/current_arm_angle", "/manipulated_arm_lever",
    1.0, -100.0, 100.0, false,
    math::degrees_to_radians(32.0), math::degrees_to_radians(155.0)};
  axis_configs_[3] = {
    "bucket", "bucket_joint", "/current_bucket_angle", "/manipulated_bucket_lever",
    1.0, -100.0, 100.0, false,
    math::degrees_to_radians(-31.0), math::degrees_to_radians(159.0)};

  if (!load_hardware_parameters() || !validate_joint_interfaces()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  position_states_.fill(0.0);
  velocity_states_.fill(0.0);
  effort_commands_.fill(0.0);
  stale_reported_.fill(false);
  feedback_fault_latched_.store(false);

  node_ = std::make_shared<rclcpp::Node>("tb20e_lever_hardware");
  const auto state_qos = rclcpp::SensorDataQoS().keep_last(10);
  const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    command_publishers_[axis] = node_->create_publisher<std_msgs::msg::Float64>(
      axis_configs_[axis].command_topic, command_qos);
    state_subscriptions_[axis] = node_->create_subscription<std_msgs::msg::Float64>(
      axis_configs_[axis].state_topic,
      state_qos,
      [this, axis](const std_msgs::msg::Float64::ConstSharedPtr message) {
        feedback_callback(axis, message);
      });
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "Initialized four-axis lever hardware (state timeout %.3f s)", state_timeout_sec_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Tb20eLeverHardware::on_configure(
  const rclcpp_lifecycle::State &)
{
  active_.store(false);
  feedback_fault_latched_.store(false);
  effort_commands_.fill(0.0);
  stale_reported_.fill(false);
  reset_feedback();

  if (!start_executor()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(node_->get_logger(), "Configured and listening for Unity angle feedback");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Tb20eLeverHardware::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  active_.store(false);
  effort_commands_.fill(0.0);
  publish_zero_to_all_axes();
  stop_executor();
  reset_feedback();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Tb20eLeverHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  active_.store(false);
  effort_commands_.fill(0.0);
  stale_reported_.fill(false);
  publish_zero_to_all_axes();

  bool feedback_ready = false;
  {
    std::unique_lock<std::mutex> lock(feedback_mutex_);
    feedback_ready = feedback_condition_.wait_for(
      lock,
      std::chrono::duration<double>(initial_feedback_wait_sec_),
      [this]() {
        return all_feedback_is_fresh_locked(std::chrono::steady_clock::now());
      });
    if (feedback_ready) {
      copy_feedback_to_states_locked();
      feedback_fault_latched_.store(false);
      active_.store(true);
    }
  }

  if (!feedback_ready) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Activation refused: all four angle topics must provide fresh feedback "
      "within %.3f s",
      initial_feedback_wait_sec_);
    publish_zero_to_all_axes();
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(node_->get_logger(), "Activated with fresh feedback on all four axes");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Tb20eLeverHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_.store(false);
  effort_commands_.fill(0.0);
  publish_zero_to_all_axes();
  RCLCPP_INFO(node_->get_logger(), "Deactivated; published zero to all lever topics");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Tb20eLeverHardware::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  active_.store(false);
  effort_commands_.fill(0.0);
  publish_zero_to_all_axes();
  stop_executor();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Tb20eLeverHardware::on_error(
  const rclcpp_lifecycle::State &)
{
  active_.store(false);
  effort_commands_.fill(0.0);
  publish_zero_to_all_axes();
  stop_executor();
  reset_feedback();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
Tb20eLeverHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(info_.joints.size() * 2);

  for (std::size_t joint = 0; joint < info_.joints.size(); ++joint) {
    const std::size_t axis = joint_axis_indices_[joint];
    interfaces.emplace_back(
      info_.joints[joint].name,
      hardware_interface::HW_IF_POSITION,
      &position_states_[axis]);
    interfaces.emplace_back(
      info_.joints[joint].name,
      hardware_interface::HW_IF_VELOCITY,
      &velocity_states_[axis]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
Tb20eLeverHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(info_.joints.size());

  for (std::size_t joint = 0; joint < info_.joints.size(); ++joint) {
    const std::size_t axis = joint_axis_indices_[joint];
    interfaces.emplace_back(
      info_.joints[joint].name,
      hardware_interface::HW_IF_EFFORT,
      &effort_commands_[axis]);
  }
  return interfaces;
}

hardware_interface::return_type Tb20eLeverHardware::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  const auto now = std::chrono::steady_clock::now();
  const double period_sec = period.seconds();
  const bool calculate_velocity =
    active_.load() && std::isfinite(period_sec) &&
    period_sec > std::numeric_limits<double>::epsilon();
  std::array<bool, kAxisCount> velocity_fault{};

  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
      if (!feedback_[axis].received) {
        velocity_states_[axis] = 0.0;
        continue;
      }

      const double previous_position = position_states_[axis];
      const double current_position = feedback_[axis].position_rad;
      position_states_[axis] = current_position;
      velocity_states_[axis] = 0.0;

      if (calculate_velocity && feedback_is_fresh(feedback_[axis], now)) {
        const double position_delta = axis_configs_[axis].continuous ?
          math::shortest_angular_delta(current_position, previous_position) :
          current_position - previous_position;
        const double velocity = position_delta / period_sec;
        if (!math::velocity_exceeds_limit(
            position_delta, period_sec, max_feedback_velocity_rad_s_))
        {
          velocity_states_[axis] = velocity;
        } else {
          velocity_fault[axis] = true;
        }
      }
    }
  }

  if (active_.load() &&
    std::any_of(
      velocity_fault.begin(), velocity_fault.end(),
      [](const bool fault) {return fault;}))
  {
    for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
      if (velocity_fault[axis]) {
        RCLCPP_ERROR(
          node_->get_logger(),
          "%s feedback changed faster than the configured %.3f deg/s limit",
          axis_configs_[axis].name.c_str(),
          max_feedback_velocity_rad_s_ / math::kDegreesToRadians);
      }
    }
    latch_fault_and_publish_zero();
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Tb20eLeverHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  const auto now = std::chrono::steady_clock::now();
  std::array<bool, kAxisCount> fresh{};
  std::array<double, kAxisCount> feedback_positions{};
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
      fresh[axis] = feedback_is_fresh(feedback_[axis], now);
      feedback_positions[axis] = feedback_[axis].position_rad;
    }
  }

  const bool is_active = active_.load();
  const bool all_feedback_fresh =
    std::all_of(fresh.begin(), fresh.end(), [](const bool value) {return value;});
  const bool all_commands_finite = std::all_of(
    effort_commands_.begin(), effort_commands_.end(),
    [](const double command) {return std::isfinite(command);});

  if (is_active && (!all_feedback_fresh || !all_commands_finite)) {
    if (!feedback_fault_latched_.exchange(true)) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Safety fault latched: feedback is stale/missing or a controller command "
        "is non-finite. All lever outputs remain zero until hardware reactivation.");
    }
  }

  const bool output_enabled =
    is_active && all_feedback_fresh && all_commands_finite &&
    !feedback_fault_latched_.load();

  std::lock_guard<std::mutex> publish_lock(command_publish_mutex_);
  bool publish_failed = false;
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    double lever_command = 0.0;
    const double controller_command = effort_commands_[axis];
    if (output_enabled && !feedback_fault_latched_.load()) {
      const auto & config = axis_configs_[axis];
      bool moves_outside_end_stop = false;
      if (!config.continuous) {
        moves_outside_end_stop = math::command_points_outside_limit(
          feedback_positions[axis],
          controller_command,
          config.position_min_rad + kEndStopGuardRadians,
          config.position_max_rad - kEndStopGuardRadians);
      }

      if (!moves_outside_end_stop) {
        lever_command = math::bounded_lever_command(
          controller_command,
          config.lever_sign,
          config.lever_min,
          config.lever_max);
      } else {
        RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 2000,
          "Suppressing %s command at its configured joint limit",
          config.name.c_str());
      }
    }

    if (is_active && !fresh[axis] && !stale_reported_[axis]) {
      RCLCPP_WARN(
        node_->get_logger(),
        "%s feedback is missing or stale",
        axis_configs_[axis].name.c_str());
      stale_reported_[axis] = true;
    } else if (!feedback_fault_latched_.load() && fresh[axis]) {
      stale_reported_[axis] = false;
    }

    std_msgs::msg::Float64 message;
    message.data = std::isfinite(lever_command) ? lever_command : 0.0;
    try {
      command_publishers_[axis]->publish(message);
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        node_->get_logger(), "Failed to publish %s lever command: %s",
        axis_configs_[axis].name.c_str(), exception.what());
      publish_failed = true;
    }
  }

  if (publish_failed) {
    feedback_fault_latched_.store(true);
    publish_zero_to_all_axes_locked();
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

bool Tb20eLeverHardware::load_hardware_parameters()
{
  if (!parse_finite_double(info_, "state_timeout_sec", 0.1, state_timeout_sec_) ||
    state_timeout_sec_ <= 0.0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Hardware parameter 'state_timeout_sec' must be greater than zero");
    return false;
  }

  if (!parse_finite_double(
      info_, "initial_feedback_wait_sec", 2.0, initial_feedback_wait_sec_) ||
    initial_feedback_wait_sec_ <= 0.0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Hardware parameter 'initial_feedback_wait_sec' must be greater than zero");
    return false;
  }

  double feedback_limit_tolerance_deg = 2.0;
  if (!parse_finite_double(
      info_, "feedback_limit_tolerance_deg", 2.0,
      feedback_limit_tolerance_deg) ||
    feedback_limit_tolerance_deg < 0.0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Hardware parameter 'feedback_limit_tolerance_deg' must not be negative");
    return false;
  }
  feedback_limit_tolerance_rad_ =
    math::degrees_to_radians(feedback_limit_tolerance_deg);

  double max_feedback_velocity_deg_s = 180.0;
  if (!parse_finite_double(
      info_, "max_feedback_velocity_deg_s", 180.0,
      max_feedback_velocity_deg_s) ||
    max_feedback_velocity_deg_s <= 0.0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Hardware parameter 'max_feedback_velocity_deg_s' must be greater than zero");
    return false;
  }
  max_feedback_velocity_rad_s_ =
    math::degrees_to_radians(max_feedback_velocity_deg_s);

  for (auto & axis : axis_configs_) {
    const std::string prefix = axis.name + "_";
    axis.state_topic = string_parameter(
      info_, prefix + "state_topic", axis.state_topic);
    axis.command_topic = string_parameter(
      info_, prefix + "command_topic", axis.command_topic);

    if (!parse_finite_double(
        info_, prefix + "lever_sign", axis.lever_sign, axis.lever_sign) ||
      !parse_finite_double(
        info_, prefix + "lever_min", axis.lever_min, axis.lever_min) ||
      !parse_finite_double(
        info_, prefix + "lever_max", axis.lever_max, axis.lever_max))
    {
      return false;
    }

    if (!axis.continuous) {
      double position_min_deg =
        axis.position_min_rad / math::kDegreesToRadians;
      double position_max_deg =
        axis.position_max_rad / math::kDegreesToRadians;
      if (!parse_finite_double(
          info_, prefix + "position_min_deg",
          position_min_deg, position_min_deg) ||
        !parse_finite_double(
          info_, prefix + "position_max_deg",
          position_max_deg, position_max_deg))
      {
        return false;
      }
      axis.position_min_rad = math::degrees_to_radians(position_min_deg);
      axis.position_max_rad = math::degrees_to_radians(position_max_deg);
      if (axis.position_min_rad >= axis.position_max_rad) {
        RCLCPP_ERROR(
          rclcpp::get_logger(kLoggerName),
          "Position limits for axis '%s' must satisfy min < max",
          axis.name.c_str());
        return false;
      }
    }

    if (axis.state_topic.empty() || axis.command_topic.empty()) {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "Topics for axis '%s' must not be empty", axis.name.c_str());
      return false;
    }
    if (std::abs(std::abs(axis.lever_sign) - 1.0) >
      std::numeric_limits<double>::epsilon())
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "Hardware parameter '%slever_sign' must be either -1.0 or 1.0",
        prefix.c_str());
      return false;
    }
    if (axis.lever_min > 0.0 || axis.lever_max < 0.0 ||
      axis.lever_min >= axis.lever_max)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "Lever limits for axis '%s' must satisfy min <= 0 <= max and min < max",
        axis.name.c_str());
      return false;
    }
  }

  return true;
}

bool Tb20eLeverHardware::validate_joint_interfaces()
{
  if (info_.joints.size() != kAxisCount) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Expected %zu joints, but the URDF declares %zu",
      kAxisCount, info_.joints.size());
    return false;
  }

  joint_axis_indices_.resize(info_.joints.size());
  std::array<bool, kAxisCount> axis_seen{};

  for (std::size_t joint_index = 0; joint_index < info_.joints.size(); ++joint_index) {
    const auto & joint = info_.joints[joint_index];
    auto axis_it = std::find_if(
      axis_configs_.begin(), axis_configs_.end(),
      [&joint](const AxisConfig & axis) {return axis.joint_name == joint.name;});
    if (axis_it == axis_configs_.end()) {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "Unexpected joint '%s' in ros2_control definition", joint.name.c_str());
      return false;
    }

    const std::size_t axis = static_cast<std::size_t>(
      std::distance(axis_configs_.begin(), axis_it));
    if (axis_seen[axis]) {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "Joint '%s' is declared more than once", joint.name.c_str());
      return false;
    }
    axis_seen[axis] = true;
    joint_axis_indices_[joint_index] = axis;

    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces.front().name != hardware_interface::HW_IF_EFFORT)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "Joint '%s' must have exactly one 'effort' command interface",
        joint.name.c_str());
      return false;
    }

    bool has_position = false;
    bool has_velocity = false;
    for (const auto & state_interface : joint.state_interfaces) {
      has_position |= state_interface.name == hardware_interface::HW_IF_POSITION;
      has_velocity |= state_interface.name == hardware_interface::HW_IF_VELOCITY;
    }
    if (joint.state_interfaces.size() != 2 || !has_position || !has_velocity) {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "Joint '%s' must have exactly 'position' and 'velocity' state interfaces",
        joint.name.c_str());
      return false;
    }
  }

  return true;
}

bool Tb20eLeverHardware::start_executor()
{
  if (executor_thread_.joinable()) {
    return true;
  }
  if (!node_) {
    RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "Cannot start executor without a node");
    return false;
  }

  try {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    const auto executor = executor_;
    executor_thread_ = std::thread(
      [executor, logger = node_->get_logger()]() {
        try {
          executor->spin();
        } catch (const std::exception & exception) {
          RCLCPP_ERROR(logger, "Feedback executor stopped with an exception: %s", exception.what());
        }
      });
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      node_->get_logger(), "Failed to start feedback executor: %s", exception.what());
    executor_.reset();
    return false;
  }

  return true;
}

void Tb20eLeverHardware::stop_executor()
{
  const auto executor = executor_;
  if (executor) {
    executor->cancel();
  }
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
  if (executor && node_) {
    try {
      executor->remove_node(node_);
    } catch (const std::exception &) {
      // The context may already be shut down. There is nothing else to release here.
    }
  }
  executor_.reset();
}

bool Tb20eLeverHardware::all_feedback_is_fresh_locked(
  const std::chrono::steady_clock::time_point & now) const
{
  return std::all_of(
    feedback_.begin(), feedback_.end(),
    [this, &now](const FeedbackSample & sample) {
      return feedback_is_fresh(sample, now);
    });
}

void Tb20eLeverHardware::copy_feedback_to_states_locked()
{
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    if (!feedback_[axis].received) {
      continue;
    }
    position_states_[axis] = feedback_[axis].position_rad;
    velocity_states_[axis] = 0.0;
  }
}

void Tb20eLeverHardware::latch_fault_and_publish_zero()
{
  bool was_active = false;
  {
    std::lock_guard<std::mutex> publish_lock(command_publish_mutex_);
    {
      std::lock_guard<std::mutex> feedback_lock(feedback_mutex_);
      was_active = active_.load();
      if (was_active) {
        feedback_fault_latched_.store(true);
      }
    }
    if (was_active) {
      publish_zero_to_all_axes_locked();
    }
  }
}

void Tb20eLeverHardware::invalidate_feedback(const std::size_t axis)
{
  if (axis >= kAxisCount) {
    return;
  }

  bool was_active = false;
  {
    std::lock_guard<std::mutex> publish_lock(command_publish_mutex_);
    {
      std::lock_guard<std::mutex> feedback_lock(feedback_mutex_);
      feedback_[axis] = FeedbackSample{};
      was_active = active_.load();
      if (was_active) {
        feedback_fault_latched_.store(true);
      }
    }
    if (was_active) {
      publish_zero_to_all_axes_locked();
    }
  }
  feedback_condition_.notify_all();
}

void Tb20eLeverHardware::reset_feedback()
{
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    feedback_.fill(FeedbackSample{});
    position_states_.fill(0.0);
    velocity_states_.fill(0.0);
  }
  feedback_condition_.notify_all();
}

void Tb20eLeverHardware::publish_zero_to_all_axes()
{
  std::lock_guard<std::mutex> publish_lock(command_publish_mutex_);
  publish_zero_to_all_axes_locked();
}

void Tb20eLeverHardware::publish_zero_to_all_axes_locked()
{
  std_msgs::msg::Float64 message;
  message.data = 0.0;
  for (const auto & publisher : command_publishers_) {
    if (publisher) {
      try {
        publisher->publish(message);
      } catch (const std::exception & exception) {
        if (node_) {
          RCLCPP_WARN(
            node_->get_logger(), "Could not publish shutdown zero: %s", exception.what());
        }
      }
    }
  }
}

void Tb20eLeverHardware::feedback_callback(
  const std::size_t axis,
  const std_msgs::msg::Float64::ConstSharedPtr & message)
{
  if (axis >= kAxisCount) {
    if (node_) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Ignoring angle feedback for an unknown axis");
    }
    return;
  }

  if (!message || !std::isfinite(message->data)) {
    if (node_) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Invalid %s angle feedback; safety output is latched at zero while active",
        axis_configs_[axis].name.c_str());
    }
    invalidate_feedback(axis);
    return;
  }

  const auto & config = axis_configs_[axis];
  double position_rad = math::degrees_to_radians(message->data);
  if (config.continuous) {
    position_rad = math::wrap_to_pi(position_rad);
  } else {
    if (math::position_outside_limits(
        position_rad,
        config.position_min_rad,
        config.position_max_rad,
        feedback_limit_tolerance_rad_))
    {
      if (node_) {
        RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 2000,
          "%s feedback %.3f deg is outside the configured range; "
          "safety output is latched at zero while active",
          config.name.c_str(), message->data);
      }
      invalidate_feedback(axis);
      return;
    }
    position_rad = std::clamp(
      position_rad, config.position_min_rad, config.position_max_rad);
  }
  const auto now = std::chrono::steady_clock::now();

  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    auto & sample = feedback_[axis];
    sample.position_rad = position_rad;
    sample.received = true;
    sample.received_at = now;
  }
  feedback_condition_.notify_all();
}

bool Tb20eLeverHardware::feedback_is_fresh(
  const FeedbackSample & sample,
  const std::chrono::steady_clock::time_point & now) const
{
  if (!sample.received) {
    return false;
  }
  const double age_sec =
    std::chrono::duration<double>(now - sample.received_at).count();
  return std::isfinite(age_sec) && age_sec >= 0.0 && age_sec <= state_timeout_sec_;
}

}  // namespace tb20e_control

PLUGINLIB_EXPORT_CLASS(
  tb20e_control::Tb20eLeverHardware,
  hardware_interface::SystemInterface)
