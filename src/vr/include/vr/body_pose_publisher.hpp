/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Vamsi Kalagaturu
 * See LICENSE for details. */

#pragma once

#include <string>

#include <mujoco/mujoco.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace vr {

/** Where the exported world lives and how often its bodies are published. */
struct SceneConf
{
    std::string manifest_path;          // manifest.json written by scene_export
    std::string scene_url;              // URL the headset fetches scene.glb from
    std::string frame_id  = "world";    // frame the poses are expressed in
    std::string topic_ns  = "/vr";      // -> <topic_ns>/body_poses, <topic_ns>/scene
    double      rate_hz   = 60.0;
};

/**
 * Publishes a MuJoCo world to the headset: the scene description once, then every body's pose.
 *
 * A state sink, not a simulator: it creates no node, no executor and no thread, owns no mjData,
 * and never steps anything. Call it from whatever already owns the simulation - which is how an
 * existing mj_kdl_wrapper application gains VR output without restructuring:
 *
 *   vr::BodyPosePublisher vr_out(*node, model, conf);
 *   while (running) {
 *       mj_step(model, data);
 *       if (vr_out.wants_update(data->time)) vr_out.publish(data);
 *   }
 *
 * Poses are MuJoCo world coordinates, already REP-103 (Z up, right-handed); the conversion to
 * the client's left-handed Y-up space happens on the client, once.
 *
 * Pose i is body i, matching mjModel body order and the manifest's bodies[] - the client indexes
 * straight into it, so nothing here may reorder.
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
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr poses_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr         scene_pub_;
    geometry_msgs::msg::PoseArray                               poses_;
    int                                                         nbody_      = 0;
    double                                                      period_s_   = 0.0;
    double                                                      next_due_s_ = 0.0;
    bool                                                        has_scene_  = false;
};

} // namespace vr
