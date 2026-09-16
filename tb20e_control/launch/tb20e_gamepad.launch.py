#!/usr/bin/env python3
# Copyright 2026 tb20e_ros2 contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _integer_parameter(name):
    return ParameterValue(LaunchConfiguration(name), value_type=int)


def _double_parameter(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _boolean_parameter(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def generate_launch_description():
    package_share = FindPackageShare("tb20e_control")

    arguments = [
        DeclareLaunchArgument("joy_device_id", default_value="0"),
        DeclareLaunchArgument("swing_axis", default_value="0"),
        DeclareLaunchArgument("arm_axis", default_value="1"),
        DeclareLaunchArgument("bucket_axis", default_value="2"),
        DeclareLaunchArgument("boom_axis", default_value="3"),
        DeclareLaunchArgument("swing_scale", default_value="100.0"),
        DeclareLaunchArgument("arm_scale", default_value="-100.0"),
        DeclareLaunchArgument("bucket_scale", default_value="100.0"),
        DeclareLaunchArgument("boom_scale", default_value="100.0"),
        DeclareLaunchArgument("deadzone", default_value="0.10"),
        DeclareLaunchArgument("joy_timeout_sec", default_value="0.25"),
        DeclareLaunchArgument("deadman_button", default_value="-1"),
        DeclareLaunchArgument("command_output_enabled", default_value="true"),
        DeclareLaunchArgument(
            "unity_position_output_enabled", default_value="false"
        ),
        DeclareLaunchArgument("sim_feedback_timeout_sec", default_value="0.25"),
        DeclareLaunchArgument(
            "swing_sim_state_topic",
            default_value="/sim/tb20e/current_swing_angle",
        ),
        DeclareLaunchArgument(
            "boom_sim_state_topic",
            default_value="/sim/tb20e/current_boom_angle",
        ),
        DeclareLaunchArgument(
            "arm_sim_state_topic",
            default_value="/sim/tb20e/current_arm_angle",
        ),
        DeclareLaunchArgument(
            "bucket_sim_state_topic",
            default_value="/sim/tb20e/current_bucket_angle",
        ),
        DeclareLaunchArgument("swing_unity_speed_deg_s", default_value="50.0"),
        DeclareLaunchArgument("boom_unity_speed_deg_s", default_value="50.0"),
        DeclareLaunchArgument("arm_unity_speed_deg_s", default_value="50.0"),
        DeclareLaunchArgument("bucket_unity_speed_deg_s", default_value="50.0"),
        DeclareLaunchArgument("swing_unity_position_sign", default_value="-1.0"),
        DeclareLaunchArgument("boom_unity_position_sign", default_value="1.0"),
        DeclareLaunchArgument("arm_unity_position_sign", default_value="1.0"),
        DeclareLaunchArgument("bucket_unity_position_sign", default_value="1.0"),
    ]

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [package_share, "launch", "tb20e_control.launch.py"]
            )
        ),
        launch_arguments={
            "controller_name": "tb20e_gamepad_controller",
            "command_output_enabled": LaunchConfiguration(
                "command_output_enabled"
            ),
        }.items(),
    )

    joy = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[{
            "device_id": _integer_parameter("joy_device_id"),
            "deadzone": 0.0,
            "autorepeat_rate": 20.0,
        }],
    )

    gamepad = Node(
        package="tb20e_control",
        executable="tb20e_gamepad_node",
        output="screen",
        parameters=[{
            "swing_axis": _integer_parameter("swing_axis"),
            "arm_axis": _integer_parameter("arm_axis"),
            "bucket_axis": _integer_parameter("bucket_axis"),
            "boom_axis": _integer_parameter("boom_axis"),
            "swing_scale": _double_parameter("swing_scale"),
            "arm_scale": _double_parameter("arm_scale"),
            "bucket_scale": _double_parameter("bucket_scale"),
            "boom_scale": _double_parameter("boom_scale"),
            "deadzone": _double_parameter("deadzone"),
            "joy_timeout_sec": _double_parameter("joy_timeout_sec"),
            "deadman_button": _integer_parameter("deadman_button"),
            "publish_rate": 20.0,
            "unity_position_output_enabled": _boolean_parameter(
                "unity_position_output_enabled"
            ),
            "sim_feedback_timeout_sec": _double_parameter(
                "sim_feedback_timeout_sec"
            ),
            "swing_sim_state_topic": LaunchConfiguration(
                "swing_sim_state_topic"
            ),
            "boom_sim_state_topic": LaunchConfiguration("boom_sim_state_topic"),
            "arm_sim_state_topic": LaunchConfiguration("arm_sim_state_topic"),
            "bucket_sim_state_topic": LaunchConfiguration(
                "bucket_sim_state_topic"
            ),
            "swing_unity_speed_deg_s": _double_parameter(
                "swing_unity_speed_deg_s"
            ),
            "boom_unity_speed_deg_s": _double_parameter(
                "boom_unity_speed_deg_s"
            ),
            "arm_unity_speed_deg_s": _double_parameter(
                "arm_unity_speed_deg_s"
            ),
            "bucket_unity_speed_deg_s": _double_parameter(
                "bucket_unity_speed_deg_s"
            ),
            "swing_unity_position_sign": _double_parameter(
                "swing_unity_position_sign"
            ),
            "boom_unity_position_sign": _double_parameter(
                "boom_unity_position_sign"
            ),
            "arm_unity_position_sign": _double_parameter(
                "arm_unity_position_sign"
            ),
            "bucket_unity_position_sign": _double_parameter(
                "bucket_unity_position_sign"
            ),
        }],
    )

    return LaunchDescription(arguments + [gamepad, joy, control])
