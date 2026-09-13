# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Vamsi Kalagaturu
# See LICENSE for details.

"""Brings up everything the headset talks to: rosbridge, the scene file server, the components.

SceneNode and InputNode share one container, so the pose stream and the controller stream cross
between them intra-process rather than through the network stack.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory("vr"), "config", "vr.yaml")

    args = [
        DeclareLaunchArgument("model", description="MJCF file to simulate"),
        DeclareLaunchArgument(
            "scene_dir",
            default_value="/tmp/vr_scene",
            description="directory holding scene.glb and manifest.json, served over HTTP",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="topics, frames, rates and calibration for both components",
        ),
        DeclareLaunchArgument("rosbridge_port", default_value="9090"),
        DeclareLaunchArgument("http_port", default_value="8000"),
        DeclareLaunchArgument(
            "host_ip",
            default_value="0.0.0.0",
            description="address the headset reaches this machine on; used to build scene_url",
        ),
    ]

    scene_dir = LaunchConfiguration("scene_dir")
    http_port = LaunchConfiguration("http_port")
    host_ip = LaunchConfiguration("host_ip")
    params_file = LaunchConfiguration("params_file")

    scene_url = PythonExpression(
        ["'http://' + '", host_ip, "' + ':' + '", http_port, "' + '/scene.glb'"]
    )

    container = ComposableNodeContainer(
        name="vr_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="vr",
                plugin="vr::SceneNode",
                name="vr_scene",
                # The config file carries everything stable; only what the launch computes
                # (which model, where it is served from) is passed alongside it.
                parameters=[
                    params_file,
                    {
                        "model": LaunchConfiguration("model"),
                        "manifest": PythonExpression(["'", scene_dir, "' + '/manifest.json'"]),
                        "scene_url": scene_url,
                    },
                ],
            ),
            ComposableNode(
                package="vr",
                plugin="vr::InputNode",
                name="vr_input",
                parameters=[params_file],
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
            # The headset fetches the .glb over plain HTTP; only the pose stream needs ROS.
            ExecuteProcess(
                cmd=["python3", "-m", "http.server", http_port, "--directory", scene_dir],
                output="screen",
            ),
            container,
        ]
    )
