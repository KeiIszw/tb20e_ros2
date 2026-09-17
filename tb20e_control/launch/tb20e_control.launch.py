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

import math

import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler,
    SetLaunchConfiguration,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


XACRO_ARGUMENT_DEFAULTS = {
    "state_timeout_sec": "0.10",
    "initial_feedback_wait_sec": "2.0",
    "feedback_limit_tolerance_deg": "2.0",
    "max_feedback_velocity_deg_s": "180.0",
    "feedback_velocity_jitter_tolerance_sec": "0.03",
    "command_output_enabled": "true",
    "swing_state_topic": "/TB20e_0/current_swing_angle",
    "swing_command_topic": "/TB20e_0/manipulated_swing_lever",
    "swing_lever_sign": "1.0",
    "swing_lever_min": "-100.0",
    "swing_lever_max": "100.0",
    "swing_lever_positive_min": "0.0",
    "swing_lever_negative_min": "0.0",
    "swing_lever_start": "2.0",
    "swing_lever_stop": "1.0",
    "boom_state_topic": "/TB20e_0/current_boom_angle",
    "boom_command_topic": "/TB20e_0/manipulated_boom_lever",
    "boom_lever_sign": "-1.0",
    "boom_lever_min": "-100.0",
    "boom_lever_max": "100.0",
    "boom_lever_positive_min": "0.0",
    "boom_lever_negative_min": "0.0",
    "boom_lever_start": "2.0",
    "boom_lever_stop": "1.0",
    "arm_state_topic": "/TB20e_0/current_arm_angle",
    "arm_command_topic": "/TB20e_0/manipulated_arm_lever",
    "arm_lever_sign": "1.0",
    "arm_lever_min": "-100.0",
    "arm_lever_max": "100.0",
    "arm_lever_positive_min": "0.0",
    "arm_lever_negative_min": "0.0",
    "arm_lever_start": "2.0",
    "arm_lever_stop": "1.0",
    "bucket_state_topic": "/TB20e_0/current_bucket_angle",
    "bucket_command_topic": "/TB20e_0/manipulated_bucket_lever",
    "bucket_lever_sign": "1.0",
    "bucket_lever_min": "-100.0",
    "bucket_lever_max": "100.0",
    "bucket_lever_positive_min": "0.0",
    "bucket_lever_negative_min": "0.0",
    "bucket_lever_start": "2.0",
    "bucket_lever_stop": "1.0",
}


COMPENSATION_DEFAULTS = {
    name: value for name, value in XACRO_ARGUMENT_DEFAULTS.items()
    if name.endswith(("_lever_positive_min", "_lever_negative_min", "_lever_start", "_lever_stop"))
}


def load_compensation(context):
    # Resolve the selected controller file at launch time; explicit CLI values win.
    path = LaunchConfiguration("controllers_file").perform(context)
    with open(path, encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    values = document.get("/**/tb20e_lever_hardware", {}).get("ros__parameters", {})
    if not isinstance(values, dict):
        raise ValueError("tb20e_lever_hardware.ros__parameters must be a mapping")
    unknown = set(values) - set(COMPENSATION_DEFAULTS)
    if unknown:
        raise ValueError(f"Unknown lever compensation parameters: {sorted(unknown)}")
    actions = []
    for name, default in COMPENSATION_DEFAULTS.items():
        if LaunchConfiguration(name).perform(context) != "auto":
            continue
        value = values.get(name, default)
        if isinstance(value, bool):
            raise ValueError(f"{name} must be a finite number")
        value = float(value)
        if not math.isfinite(value):
            raise ValueError(f"{name} must be a finite number")
        actions.append(SetLaunchConfiguration(name, str(value)))
    return actions


def generate_launch_description():
    package_share = FindPackageShare("tb20e_control")
    xacro_file = PathJoinSubstitution(
        [package_share, "urdf", "tb20e.urdf.xacro"]
    )
    default_controllers_file = PathJoinSubstitution(
        [package_share, "config", "tb20e_controllers_0.yaml"]
    )

    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("frame_prefix", default_value=""),
        DeclareLaunchArgument("controller_name", default_value="tb20e_controller"),
        DeclareLaunchArgument(
            "controllers_file", default_value=default_controllers_file
        ),
    ]
    declared_arguments.extend(
        DeclareLaunchArgument(
            name, default_value="auto" if name in COMPENSATION_DEFAULTS else value
        )
        for name, value in XACRO_ARGUMENT_DEFAULTS.items()
    )

    xacro_command = [FindExecutable(name="xacro"), " ", xacro_file]
    for name in XACRO_ARGUMENT_DEFAULTS:
        xacro_command.extend([" ", name, ":=", LaunchConfiguration(name)])

    robot_description = {
        "robot_description": ParameterValue(
            Command(xacro_command),
            value_type=str,
        )
    }
    use_sim_time = ParameterValue(
        LaunchConfiguration("use_sim_time"), value_type=bool
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {
            "use_sim_time": use_sim_time,
            "frame_prefix": ParameterValue(
                LaunchConfiguration("frame_prefix"), value_type=str
            ),
        }],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            robot_description,
            LaunchConfiguration("controllers_file"),
            {"use_sim_time": use_sim_time},
        ],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
        output="screen",
    )

    selected_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            LaunchConfiguration("controller_name"),
            "--controller-manager",
            "controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
        output="screen",
    )

    shutdown_on_control_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=control_node,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason="ros2_control_node exited; stopping bringup"
                    )
                )
            ],
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            OpaqueFunction(function=load_compensation),
            robot_state_publisher,
            shutdown_on_control_exit,
            control_node,
            joint_state_broadcaster,
            selected_controller,
        ]
    )
