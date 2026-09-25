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
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import PushRosNamespace, SetRemap
from launch_ros.substitutions import FindPackageShare


def _is_source(name):
    return IfCondition(
        PythonExpression(
            ["'", LaunchConfiguration("input_source"), "' == '", name, "'"]
        )
    )


def generate_launch_description():
    robot_namespace = LaunchConfiguration("ns")
    control_share = FindPackageShare("tb20e_control")
    scratch_share = FindPackageShare("scratch_hci_bridge")

    arguments = [
        DeclareLaunchArgument("ns", default_value="TB20e_0"),
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [control_share, "config", "tb20e_controllers_0.yaml"]
            ),
        ),
        DeclareLaunchArgument(
            "input_source",
            default_value="gamepad",
            choices=["gamepad", "http"],
            description="Command source; the two modes are mutually exclusive.",
        ),
        DeclareLaunchArgument("real_output_enabled", default_value="true"),
        DeclareLaunchArgument("unity_position_output_enabled", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("joy_device_id", default_value="0"),
        DeclareLaunchArgument("neutral_hold_sec", default_value="0.5"),
        DeclareLaunchArgument("http_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("http_port", default_value="8899"),
        DeclareLaunchArgument("swing_state_topic", default_value=["/", robot_namespace, "/current_swing_angle"]),
        DeclareLaunchArgument("boom_state_topic", default_value=["/", robot_namespace, "/current_boom_angle"]),
        DeclareLaunchArgument("arm_state_topic", default_value=["/", robot_namespace, "/current_arm_angle"]),
        DeclareLaunchArgument("bucket_state_topic", default_value=["/", robot_namespace, "/current_bucket_angle"]),
        DeclareLaunchArgument(
            "swing_sim_state_topic",
            default_value=["/sim/", robot_namespace, "/current_swing_angle"],
        ),
        DeclareLaunchArgument(
            "boom_sim_state_topic",
            default_value=["/sim/", robot_namespace, "/current_boom_angle"],
        ),
        DeclareLaunchArgument(
            "arm_sim_state_topic",
            default_value=["/sim/", robot_namespace, "/current_arm_angle"],
        ),
        DeclareLaunchArgument(
            "bucket_sim_state_topic",
            default_value=["/sim/", robot_namespace, "/current_bucket_angle"],
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

    compensation_defaults = {
        "lever_positive_min": "auto", "lever_negative_min": "auto",
        "lever_start": "auto", "lever_stop": "auto",
    }
    for axis in ("swing", "boom", "arm", "bucket"):
        for suffix, default in compensation_defaults.items():
            arguments.append(DeclareLaunchArgument(f"{axis}_{suffix}", default_value=default))

    common_hardware_arguments = {
        "controllers_file": LaunchConfiguration("controllers_file"),
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "command_output_enabled": LaunchConfiguration("real_output_enabled"),
        "swing_state_topic": LaunchConfiguration("swing_state_topic"),
        "boom_state_topic": LaunchConfiguration("boom_state_topic"),
        "arm_state_topic": LaunchConfiguration("arm_state_topic"),
        "bucket_state_topic": LaunchConfiguration("bucket_state_topic"),
    }

    for axis in ("swing", "boom", "arm", "bucket"):
        for suffix in compensation_defaults:
            name = f"{axis}_{suffix}"
            common_hardware_arguments[name] = LaunchConfiguration(name)

    # The gamepad include uses the common control launch; explicitly select
    # this robot's actuator topics as well as its feedback topics.
    common_hardware_arguments["frame_prefix"] = [robot_namespace, "/"]
    for axis in ("swing", "boom", "arm", "bucket"):
        common_hardware_arguments[f"{axis}_command_topic"] = (
            ["/", robot_namespace, f"/manipulated_{axis}_lever"]
        )

    gamepad_arguments = dict(common_hardware_arguments)
    gamepad_arguments.update(
        {
            "joy_device_id": LaunchConfiguration("joy_device_id"),
            "neutral_hold_sec": LaunchConfiguration("neutral_hold_sec"),
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
            "excavator": robot_namespace,
            "trajectory_action": (
                ["/", robot_namespace, "/tb20e_controller/follow_joint_trajectory"]
            ),
            "joint_state_topic": ["/", robot_namespace, "/joint_states"],
            "unity_position_command_prefix": ["/", robot_namespace],
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

    # The gamepad node uses absolute names; scope these explicitly too.
    remappings = [
        SetRemap(src="/joy", dst=["/", robot_namespace, "/joy"]),
        SetRemap(
            src="/tb20e_gamepad_controller/commands",
            dst=["/", robot_namespace, "/tb20e_gamepad_controller/commands"],
        ),
    ]
    remappings.extend(
        SetRemap(src=f"/tb20e/{axis}/cmd", dst=["/", robot_namespace, f"/{axis}/cmd"])
        for axis in ("swing", "boom", "arm", "bucket")
    )
    return LaunchDescription(arguments + [
        GroupAction([
            PushRosNamespace(robot_namespace),
            *remappings,
            gamepad,
            http_control,
            http_bridge,
        ]),
    ])
