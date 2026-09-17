/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Vamsi Kalagaturu
 * See LICENSE for details. */

#pragma once

#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <vive_vr_ros2/msg/body_poses.hpp>

namespace vive_vr_ros2 {

/** Where the exported world lives and how often its bodies are published. */
struct SceneConf
{
    std::string manifest_path;          // manifest.json written by scene_export
    std::string scene_url;              // URL the headset fetches scene.glb from
    std::string frame_id  = "world";    // frame the poses are expressed in
    std::string topic_ns  = "/vive_vr"; // -> <topic_ns>/body_poses, <topic_ns>/scene
    double      rate_hz   = 60.0;

    /* Scenery the client draws and nothing simulates. Invisible to MuJoCo: anything the user
     * should collide with still needs geoms in the MJCF. */
    std::string env_url;
    double      env_xyz[3]   = { 0.0, 0.0, 0.0 }; // ROS coordinates
    double      env_yaw_deg  = 0.0;
    double      env_scale    = 1.0;
};

/**
 * Publishes a MuJoCo world to the headset: the scene description once, then every body's pose.
 *
 * A state sink, not a simulator: it creates no node, no executor and no thread, owns no mjData,
 * and never steps anything. Call it from whatever already owns the simulation - which is how an
 * existing mj_kdl_wrapper application gains VR output without restructuring:
 *
 *   vive_vr_ros2::BodyPosePublisher vr_out(*node, model, conf);
 *   while (running) {
 *       mj_step(model, data);
 *       if (vr_out.wants_update(data->time)) vr_out.publish(data);
 *   }
 *
 * Poses are MuJoCo world coordinates, already REP-103; the client converts to its left-handed
 * Y-up space once.
 *
 * Each frame carries only the bodies that moved. Sending all 194 bodies of a kitchen so one
 * bottle could move cost 16 KB of rosbridge JSON per frame, which saturated the socket. A full
 * frame still goes out for a new subscriber and at least every half second, or a client joining
 * a world at rest would place nothing.
 */
class BodyPosePublisher
{
  public:
    /** Publishes the scene description immediately; it is latched for clients that join later. */
    BodyPosePublisher(rclcpp::Node &node, const mjModel *model, SceneConf conf);

    /** Due by rate, and somebody is subscribed. No mjData is touched. */
    bool wants_update(double sim_t) const;

    /** One pose per body, stamped with the node clock. */
    void publish(const mjData *data);

    /** False when scene_export has not been run for this model; the client then has no geometry. */
    bool has_scene() const { return has_scene_; }

  private:
    SceneConf                                                   conf_;
    rclcpp::Node                                               &node_;
    rclcpp::Publisher<vive_vr_ros2::msg::BodyPoses>::SharedPtr            poses_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr         scene_pub_;
    vive_vr_ros2::msg::BodyPoses                                          poses_;
    std::vector<int>                                            dynamic_; // bodies that can move
    std::vector<mjtNum>                                         sent_;    // last published, 7/body
    bool                                                        have_sent_   = false;
    double                                                      last_full_s_ = 0.0;
    size_t                                                      last_subs_   = 0;
    int                                                         nbody_      = 0;
    double                                                      period_s_   = 0.0;
    double                                                      next_due_s_ = 0.0;
    bool                                                        has_scene_  = false;
};

} // namespace vive_vr_ros2
