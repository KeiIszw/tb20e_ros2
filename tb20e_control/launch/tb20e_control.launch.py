#!/usr/bin/env python3
# Copyright 2026 tb20e_ros2 contributors
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    "swing_state_topic": "/current_swing_angle",
    "swing_command_topic": "/manipulated_swing_lever",
    "swing_lever_sign": "1.0",
    "swing_lever_min": "-100.0",
    "swing_lever_max": "100.0",
    "boom_state_topic": "/current_boom_angle",
    "boom_command_topic": "/manipulated_boom_lever",
    "boom_lever_sign": "-1.0",
    "boom_lever_min": "-100.0",
    "boom_lever_max": "100.0",
    "arm_state_topic": "/current_arm_angle",
    "arm_command_topic": "/manipulated_arm_lever",
    "arm_lever_sign": "1.0",
    "arm_lever_min": "-100.0",
    "arm_lever_max": "100.0",
    "bucket_state_topic": "/current_bucket_angle",
    "bucket_command_topic": "/manipulated_bucket_lever",
    "bucket_lever_sign": "1.0",
    "bucket_lever_min": "-100.0",
    "bucket_lever_max": "100.0",
}


def generate_launch_description():
    package_share = FindPackageShare("tb20e_control")
    xacro_file = PathJoinSubstitution(
        [package_share, "urdf", "tb20e.urdf.xacro"]
    )
    default_controllers_file = PathJoinSubstitution(
        [package_share, "config", "tb20e_controllers.yaml"]
    )

    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "controllers_file", default_value=default_controllers_file
        ),
    ]
    declared_arguments.extend(
        DeclareLaunchArgument(name, default_value=value)
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
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
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
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
        output="screen",
    )

    trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "tb20e_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
        output="screen",
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_state_publisher,
            control_node,
            joint_state_broadcaster,
            trajectory_controller,
        ]
    )
