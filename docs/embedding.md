# Embedding in an existing simulation {#page_embedding}

`vive_vr_ros2::SceneNode` exists for the case where you want to look at an MJCF file. It is not the way to
add VR output to an application that already owns a simulation — for that, use the class it is
built on.

## The publisher class

A state sink, not a simulator. It creates no node, no executor and no thread, owns no `mjData`,
and never steps anything. It links **MuJoCo only, not KDL**, so it costs an application nothing
beyond what it already has.

The shape deliberately mirrors `mj_kdl::CameraRosPublisher`, so it reads the same way as the rest
of a `mj_kdl_wrapper` application.

```cpp
#include <vr/body_pose_publisher.hpp>

vive_vr_ros2::SceneConf conf;
// scene_export writes these; no tilde expansion here, so give it a real path
conf.manifest_path = std::string(std::getenv("HOME"))
                   + "/.cache/vive_vr_ros2/exports/kinova_gen3/manifest.json";
conf.scene_url     = "http://192.168.2.118:8000/scene.glb";
conf.frame_id      = "world";
conf.rate_hz       = 60.0;

vive_vr_ros2::BodyPosePublisher vr_out(*node, model, conf);

while (running) {
    mj_step(model, data);
    if (vr_out.wants_update(data->time)) vr_out.publish(data);
}
```

That is the whole integration. The scene description is published once at construction and
latched, so a headset connecting later still receives it.

`wants_update()` returns true only when the rate is due **and** something is subscribed, so an
application with no headset attached pays almost nothing. `has_scene()` is false when
`scene_export` has not been run for this model, which is worth checking at startup rather than
discovering as an empty world.

## CMake

```cmake
find_package(vr REQUIRED)
target_link_libraries(my_app PRIVATE vive_vr_ros2::vive_vr_core)
```

## What it does not do

- It does not simulate. Stepping, pacing and reset stay with the application.
- It does not read input. Controller poses, hands and gaze arrive on the topics published by
  `vive_vr_ros2::InputNode`, which runs independently and knows nothing about MuJoCo.
- It does not export geometry. Run `scene_export` once for the model; see [Running](run.md).

## When to use the scene component instead

When there is no application yet — inspecting a model, checking an export, or bringing the
headset up. It adds a simulation loop, `mj_kdl::Env` scene building and the `/vive_scene/reset`
service around the same class.
