# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Vamsi Kalagaturu
# See LICENSE for details.

"""Tracking only: what the headset sends, calibrated into the world frame. No simulation.

rosbridge carries the client's stream; InputNode calibrates it and publishes the PC clock the
client stamps against. sim.launch.py includes this file and loads SceneNode into the same
container, so the two halves are one stack rather than two copies of the same wiring.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory("vr"), "config", "vr.yaml")

    args = [
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="topics, frames, rates and calibration",
        ),
        DeclareLaunchArgument("rosbridge_port", default_value="9090"),
        DeclareLaunchArgument(
            "container_name",
            default_value="vr_container",
            description="sim.launch.py loads SceneNode into this container",
        ),
        DeclareLaunchArgument(
            "publish_hand_joints",
            default_value="true",
            description="republish the 26 hand joints; the headset sends them regardless",
        ),
        DeclareLaunchArgument("publish_hand_tf", default_value="true"),
        DeclareLaunchArgument("publish_gaze", default_value="true"),
    ]

    params_file = LaunchConfiguration("params_file")

    def flag(name):
        return ParameterValue(LaunchConfiguration(name), value_type=bool)

    container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="vr",
                plugin="vr::InputNode",
                name="vr_input",
                parameters=[
                    params_file,
                    {
                        "publish_hand_joints": flag("publish_hand_joints"),
                        "publish_hand_tf": flag("publish_hand_tf"),
                        "publish_gaze": flag("publish_gaze"),
                    },
                ],
            ),
        ],
    )

    return LaunchDescription(
        args
        + [
            Node(
                package="rosbridge_server",
                executable="rosbridge_websocket",
                name="rosbridge_websocket",
                # Launch arguments are strings; this parameter is declared int.
                parameters=[
                    {
                        "port": ParameterValue(
                            LaunchConfiguration("rosbridge_port"), value_type=int
                        )
                    }
                ],
            ),
            container,
        ]
    )
