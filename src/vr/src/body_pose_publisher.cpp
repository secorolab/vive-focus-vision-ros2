/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Vamsi Kalagaturu
 * See LICENSE for details. */

#include "vr/body_pose_publisher.hpp"

#include <fstream>
#include <sstream>
#include <utility>

namespace vr {
namespace {

std::string read_file(const std::string &path)
{
    std::ifstream in(path);
    if (!in) return {};
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

} // namespace

BodyPosePublisher::BodyPosePublisher(rclcpp::Node &node, const mjModel *model, SceneConf conf)
  : conf_(std::move(conf)),
    node_(node),
    nbody_(static_cast<int>(model->nbody)),
    period_s_(conf_.rate_hz > 0.0 ? 1.0 / conf_.rate_hz : 0.0)
{
    /* Best effort: a dropped frame is replaced by the next one a frame-time later, and a queue
     * would only add latency to a stream the client renders live. */
    poses_pub_ = node.create_publisher<geometry_msgs::msg::PoseArray>(
      conf_.topic_ns + "/body_poses", rclcpp::SensorDataQoS());

    /* Transient local: the headset subscribes whenever it connects, long after this is sent. */
    scene_pub_ = node.create_publisher<std_msgs::msg::String>(
      conf_.topic_ns + "/scene", rclcpp::QoS(1).transient_local().reliable());

    poses_.header.frame_id = conf_.frame_id;
    poses_.poses.resize(nbody_);

    const std::string manifest = read_file(conf_.manifest_path);
    has_scene_                 = !manifest.empty();
    if (!has_scene_) {
        RCLCPP_WARN(node.get_logger(),
                    "no manifest at '%s': the client will have no geometry to load. Run "
                    "scene_export for this model first.",
                    conf_.manifest_path.c_str());
    }

    std_msgs::msg::String msg;
    msg.data = "{\"url\":\"" + conf_.scene_url + "\",\"manifest\":" +
               (has_scene_ ? manifest : "null") + "}";
    scene_pub_->publish(msg);
}

bool BodyPosePublisher::wants_update(double sim_t) const
{
    if (period_s_ > 0.0 && sim_t < next_due_s_) {
        /* A reset rewinds sim time to zero. Without noticing that, the stream stalls until the
         * simulation has run back up to the time of the last frame sent - which is silent, and
         * looks exactly like the renderer having died. */
        const bool rewound = sim_t + period_s_ < next_due_s_;
        if (!rewound) return false;
    }
    return poses_pub_->get_subscription_count() > 0;
}

void BodyPosePublisher::publish(const mjData *data)
{
    next_due_s_ = data->time + period_s_;

    /* Node clock, not sim time: the client aligns these against its own clock offset, and a
     * recorded demonstration lines controller poses up with sim state on the ROS clock. */
    poses_.header.stamp = node_.now();
    for (int b = 0; b < nbody_; ++b) {
        const mjtNum *p   = data->xpos + 3 * b;
        const mjtNum *q   = data->xquat + 4 * b; // MuJoCo order is (w, x, y, z)
        auto         &out = poses_.poses[b];
        out.position.x    = p[0];
        out.position.y    = p[1];
        out.position.z    = p[2];
        out.orientation.w = q[0];
        out.orientation.x = q[1];
        out.orientation.y = q[2];
        out.orientation.z = q[3];
    }
    poses_pub_->publish(poses_);
}

} // namespace vr
