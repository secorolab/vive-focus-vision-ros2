# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Vamsi Kalagaturu
# See LICENSE for details.

"""The full stack: tracking, plus an MJCF world simulated here and drawn on the headset.

tracking.launch.py brings up rosbridge and InputNode; this file adds the HTTP server the client
fetches the .glb from and loads SceneNode into the container tracking.launch.py already started,
so the pose stream and the controller stream cross between them inside one process.
"""

import os
import socket

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def default_route_address() -> str:
    """This machine's address on the default route, which is the .glb URL's best guess.

    Connecting a UDP socket performs the route lookup and assigns a source address without
    sending anything; TEST-NET-1 is reserved and routed nowhere, so nothing is ever contacted.
    A machine whose default route is not the interface the headset is on needs host_ip
    passed explicitly - the guess is a default, not a detection.
    """
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(("192.0.2.1", 1))
        return probe.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        probe.close()


def generate_launch_description():
    share = get_package_share_directory("vive_vr_ros2")
    default_params = os.path.join(share, "config", "vive_vr.yaml")

    args = [
        DeclareLaunchArgument("model", description="MJCF file to simulate"),
        DeclareLaunchArgument(
            "scene_dir",
            default_value="/tmp/vive_vr_scene",
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
            default_value=default_route_address(),
            description="address the headset reaches this machine on; used to build scene_url. "
                        "Defaults to this machine's address on the default route",
        ),
        DeclareLaunchArgument(
            "env_glb",
            default_value="",
            description="scenery to draw and not simulate: a .glb in scene_dir, by file name",
        ),
        DeclareLaunchArgument("env_yaw_deg", default_value="0.0"),
        DeclareLaunchArgument("env_scale", default_value="1.0"),
        DeclareLaunchArgument("container_name", default_value="vive_container"),
        DeclareLaunchArgument("publish_hand_joints", default_value="true"),
        DeclareLaunchArgument("publish_hand_tf", default_value="true"),
        DeclareLaunchArgument("publish_gaze", default_value="true"),
    ]

    scene_dir = LaunchConfiguration("scene_dir")
    http_port = LaunchConfiguration("http_port")
    host_ip = LaunchConfiguration("host_ip")
    params_file = LaunchConfiguration("params_file")

    scene_url = PythonExpression(
        ["'http://' + '", host_ip, "' + ':' + '", http_port, "' + '/scene.glb'"]
    )

    # Empty stays empty, so that an unset env_glb does not become a URL to nothing.
    env_glb = LaunchConfiguration("env_glb")
    env_url = PythonExpression(
        ["('http://' + '", host_ip, "' + ':' + '", http_port, "' + '/' + '", env_glb,
         "') if '", env_glb, "' else ''"]
    )

    tracking = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "tracking.launch.py")),
        launch_arguments={
            "params_file": params_file,
            "rosbridge_port": LaunchConfiguration("rosbridge_port"),
            "container_name": LaunchConfiguration("container_name"),
            "publish_hand_joints": LaunchConfiguration("publish_hand_joints"),
            "publish_hand_tf": LaunchConfiguration("publish_hand_tf"),
            "publish_gaze": LaunchConfiguration("publish_gaze"),
        }.items(),
    )

    scene = LoadComposableNodes(
        target_container=LaunchConfiguration("container_name"),
        composable_node_descriptions=[
            ComposableNode(
                package="vive_vr_ros2",
                plugin="vive_vr_ros2::SceneNode",
                name="vive_scene",
                # The config file carries everything stable; only what the launch computes
                # (which model, where it is served from) is passed alongside it.
                parameters=[
                    params_file,
                    {
                        "model": LaunchConfiguration("model"),
                        "manifest": PythonExpression(["'", scene_dir, "' + '/manifest.json'"]),
                        "scene_url": scene_url,
                        "env_url": env_url,
                        "env_yaw_deg": ParameterValue(
                            LaunchConfiguration("env_yaw_deg"), value_type=float
                        ),
                        "env_scale": ParameterValue(
                            LaunchConfiguration("env_scale"), value_type=float
                        ),
                    },
                ],
            ),
        ],
    )

    return LaunchDescription(
        args
        + [
            tracking,
            # The headset fetches the .glb over plain HTTP; only the pose stream needs ROS.
            ExecuteProcess(
                cmd=["python3", "-m", "http.server", http_port, "--directory", scene_dir],
                output="screen",
            ),
            scene,
        ]
    )
