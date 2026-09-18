# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Vamsi Kalagaturu
# See LICENSE for details.

"""The full stack: tracking, plus an MJCF world simulated here and drawn on the headset.

tracking.launch.py brings up rosbridge and InputNode; this file adds the HTTP server the client
fetches the .glb from and loads SceneNode into the container tracking.launch.py already started,
so the pose stream and the controller stream cross between them inside one process.
"""

import json
import os
import pathlib
import socket

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


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
            default_value="",
            description="scene.glb and manifest.json, served over HTTP; "
                        "defaults to where scene_export wrote for this model",
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

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])


def launch_setup(context, *unused_args, **unused_kwargs):
    """Everything that needs argument values rather than substitutions."""
    share = get_package_share_directory("vive_vr_ros2")

    def scene_dir_for_model(model_path):
        """The export whose manifest records this model, whatever the directory is called."""
        wanted = os.path.realpath(os.path.expanduser(model_path))
        root = pathlib.Path(os.path.expanduser("~/.cache/vive_vr_ros2/scenes"))
        for manifest in sorted(root.glob("*/manifest.json")):
            try:
                recorded = json.loads(manifest.read_text()).get("model", "")
            except (OSError, ValueError):
                continue
            if recorded and os.path.realpath(os.path.expanduser(recorded)) == wanted:
                return str(manifest.parent)
        return None

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    model = arg("model")
    host_ip, http_port = arg("host_ip"), arg("http_port")

    scene_dir = arg("scene_dir")
    if not scene_dir:
        # Menagerie names every world <robot>/scene.xml, so a bare stem collides; this is the
        # same rule scene_export uses, which is what keeps the drawn and simulated worlds one.
        stem = pathlib.Path(model).stem
        name = pathlib.Path(model).parent.name if stem == "scene" else stem
        scene_dir = os.path.expanduser(f"~/.cache/vive_vr_ros2/scenes/{name}")

        # A generated world is exported under the name its yaml gives it, which is not the name
        # the path carries, so ask the manifests which one was built from this model.
        if not os.path.exists(os.path.join(scene_dir, "manifest.json")):
            scene_dir = scene_dir_for_model(model) or scene_dir

    env_glb = arg("env_glb")
    base = f"http://{host_ip}:{http_port}"

    scene = LoadComposableNodes(
        target_container=arg("container_name"),
        composable_node_descriptions=[
            ComposableNode(
                package="vive_vr_ros2",
                plugin="vive_vr_ros2::SceneNode",
                name="vive_scene",
                parameters=[
                    arg("params_file"),
                    {
                        "model": model,
                        "manifest": os.path.join(scene_dir, "manifest.json"),
                        "scene_url": f"{base}/scene.glb",
                        "env_url": f"{base}/{env_glb}" if env_glb else "",
                        "env_yaw_deg": float(arg("env_yaw_deg")),
                        "env_scale": float(arg("env_scale")),
                    },
                ],
            ),
        ],
    )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", "tracking.launch.py")),
            launch_arguments={
                k: arg(k)
                for k in ("params_file", "rosbridge_port", "container_name",
                          "publish_hand_joints", "publish_hand_tf", "publish_gaze")
            }.items(),
        ),
        # The headset fetches the .glb over plain HTTP; only the pose stream needs ROS.
        ExecuteProcess(
            cmd=["python3", "-m", "http.server", http_port, "--directory", scene_dir],
            output="screen",
        ),
        scene,
    ]
