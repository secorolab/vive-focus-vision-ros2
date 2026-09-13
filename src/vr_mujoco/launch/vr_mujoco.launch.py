# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Vamsi Kalagaturu
# See LICENSE for details.

"""Brings up everything the headset talks to: rosbridge, the scene file server, the sim stream."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument("model", description="MJCF file to simulate"),
        DeclareLaunchArgument(
            "scene_dir",
            default_value="/tmp/vr_scene",
            description="directory holding scene.glb and manifest.json, served over HTTP",
        ),
        DeclareLaunchArgument("rate_hz", default_value="60.0"),
        DeclareLaunchArgument("frame_id", default_value="world"),
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

    scene_url = PythonExpression(
        ["'http://' + '", host_ip, "' + ':' + '", http_port, "' + '/scene.glb'"]
    )

    return LaunchDescription(
        args
        + [
            Node(
                package="rosbridge_server",
                executable="rosbridge_websocket",
                name="rosbridge_websocket",
                # Launch arguments are strings; these two parameters are declared int/double.
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
            Node(
                package="vr_mujoco",
                executable="stream_node",
                name="vr_mujoco_stream",
                output="screen",
                parameters=[
                    {
                        "model": LaunchConfiguration("model"),
                        "manifest": PythonExpression(["'", scene_dir, "' + '/manifest.json'"]),
                        "scene_url": scene_url,
                        "rate_hz": ParameterValue(
                            LaunchConfiguration("rate_hz"), value_type=float
                        ),
                        "frame_id": LaunchConfiguration("frame_id"),
                    }
                ],
            ),
        ]
    )
