#!/usr/bin/env python3
# Copyright 2026 tb20e_bringup contributors
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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare


def _is_source(name):
    return IfCondition(
        PythonExpression(
            ["'", LaunchConfiguration("input_source"), "' == '", name, "'"]
        )
    )


def generate_launch_description():
    control_share = FindPackageShare("tb20e_control")
    scratch_share = FindPackageShare("scratch_hci_bridge")

    arguments = [
        DeclareLaunchArgument(
            "input_source",
            default_value="gamepad",
            choices=["gamepad", "http"],
            description="Command source; the two modes are mutually exclusive.",
        ),
        DeclareLaunchArgument("real_output_enabled", default_value="false"),
        DeclareLaunchArgument("unity_position_output_enabled", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("joy_device_id", default_value="0"),
        DeclareLaunchArgument("deadman_button", default_value="-1"),
        DeclareLaunchArgument("http_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("http_port", default_value="8899"),
        DeclareLaunchArgument("swing_state_topic", default_value="/current_swing_angle"),
        DeclareLaunchArgument("boom_state_topic", default_value="/current_boom_angle"),
        DeclareLaunchArgument("arm_state_topic", default_value="/current_arm_angle"),
        DeclareLaunchArgument("bucket_state_topic", default_value="/current_bucket_angle"),
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
        DeclareLaunchArgument("sim_feedback_timeout_sec", default_value="0.25"),
        DeclareLaunchArgument("swing_unity_speed_deg_s", default_value="50.0"),
        DeclareLaunchArgument("boom_unity_speed_deg_s", default_value="50.0"),
        DeclareLaunchArgument("arm_unity_speed_deg_s", default_value="50.0"),
        DeclareLaunchArgument("bucket_unity_speed_deg_s", default_value="50.0"),
        DeclareLaunchArgument("swing_unity_position_sign", default_value="-1.0"),
        DeclareLaunchArgument("boom_unity_position_sign", default_value="1.0"),
        DeclareLaunchArgument("arm_unity_position_sign", default_value="1.0"),
        DeclareLaunchArgument("bucket_unity_position_sign", default_value="1.0"),
    ]

    common_hardware_arguments = {
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "command_output_enabled": LaunchConfiguration("real_output_enabled"),
        "swing_state_topic": LaunchConfiguration("swing_state_topic"),
        "boom_state_topic": LaunchConfiguration("boom_state_topic"),
        "arm_state_topic": LaunchConfiguration("arm_state_topic"),
        "bucket_state_topic": LaunchConfiguration("bucket_state_topic"),
    }

    gamepad_arguments = dict(common_hardware_arguments)
    gamepad_arguments.update(
        {
            "joy_device_id": LaunchConfiguration("joy_device_id"),
            "deadman_button": LaunchConfiguration("deadman_button"),
            "unity_position_output_enabled": LaunchConfiguration(
                "unity_position_output_enabled"
            ),
            "sim_feedback_timeout_sec": LaunchConfiguration(
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
            "swing_unity_speed_deg_s": LaunchConfiguration(
                "swing_unity_speed_deg_s"
            ),
            "boom_unity_speed_deg_s": LaunchConfiguration(
                "boom_unity_speed_deg_s"
            ),
            "arm_unity_speed_deg_s": LaunchConfiguration(
                "arm_unity_speed_deg_s"
            ),
            "bucket_unity_speed_deg_s": LaunchConfiguration(
                "bucket_unity_speed_deg_s"
            ),
            "swing_unity_position_sign": LaunchConfiguration(
                "swing_unity_position_sign"
            ),
            "boom_unity_position_sign": LaunchConfiguration(
                "boom_unity_position_sign"
            ),
            "arm_unity_position_sign": LaunchConfiguration(
                "arm_unity_position_sign"
            ),
            "bucket_unity_position_sign": LaunchConfiguration(
                "bucket_unity_position_sign"
            ),
        }
    )

    gamepad = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [control_share, "launch", "tb20e_gamepad.launch.py"]
            )
        ),
        launch_arguments=gamepad_arguments.items(),
        condition=_is_source("gamepad"),
    )

    http_control_arguments = dict(common_hardware_arguments)
    http_control_arguments["controller_name"] = "tb20e_controller"
    http_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [control_share, "launch", "tb20e_control.launch.py"]
            )
        ),
        launch_arguments=http_control_arguments.items(),
        condition=_is_source("http"),
    )

    http_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [scratch_share, "launch", "scratch_hci_bridge.launch.py"]
            )
        ),
        launch_arguments={
            "http_host": LaunchConfiguration("http_host"),
            "http_port": LaunchConfiguration("http_port"),
            "unity_position_output_enabled": LaunchConfiguration(
                "unity_position_output_enabled"
            ),
            "unity_position_command_prefix": "/tb20e",
            "swing_unity_position_sign": LaunchConfiguration(
                "swing_unity_position_sign"
            ),
            "boom_unity_position_sign": LaunchConfiguration(
                "boom_unity_position_sign"
            ),
            "arm_unity_position_sign": LaunchConfiguration(
                "arm_unity_position_sign"
            ),
            "bucket_unity_position_sign": LaunchConfiguration(
                "bucket_unity_position_sign"
            ),
        }.items(),
        condition=_is_source("http"),
    )

    return LaunchDescription(
        arguments + [gamepad, http_control, http_bridge]
    )
