/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Vamsi Kalagaturu
 * See LICENSE for details. */

#include "vr/body_pose_publisher.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
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

/* Welded bodies as [id, x,y,z, qx,qy,qz,qw]. Their pose cannot depend on qpos, so one pass of
 * forward kinematics on a scratch mjData answers for every future frame. */
std::string static_poses_json(const mjModel *model)
{
    mjData *scratch = mj_makeData(model);
    if (scratch == nullptr) return ",\"static\":[]";
    mj_kinematics(model, scratch);

    std::ostringstream out;
    out.precision(9); // the stream's own numbers are doubles; these should not be coarser
    out << ",\"static\":[";
    bool first = true;
    for (int b = 1; b < model->nbody; ++b) {
        if (model->body_weldid[b] != 0) continue;
        const mjtNum *p = scratch->xpos + 3 * b;
        const mjtNum *q = scratch->xquat + 4 * b; // MuJoCo order is (w, x, y, z)
        if (!first) out << ",";
        first = false;
        out << "[" << b << "," << p[0] << "," << p[1] << "," << p[2] << "," << q[1] << "," << q[2]
            << "," << q[3] << "," << q[0] << "]";
    }
    out << "]";

    mj_deleteData(scratch);
    return out.str();
}

} // namespace

BodyPosePublisher::BodyPosePublisher(rclcpp::Node &node, const mjModel *model, SceneConf conf)
  : conf_(std::move(conf)),
    node_(node),
    nbody_(static_cast<int>(model->nbody)),
    period_s_(conf_.rate_hz > 0.0 ? 1.0 / conf_.rate_hz : 0.0)
{
    /* Best effort: a queue would only add latency to a stream the client renders live. */
    poses_pub_ = node.create_publisher<vr::msg::BodyPoses>(
      conf_.topic_ns + "/body_poses", rclcpp::SensorDataQoS());

    /* Transient local: the headset subscribes whenever it connects, long after this is sent. */
    scene_pub_ = node.create_publisher<std_msgs::msg::String>(
      conf_.topic_ns + "/scene", rclcpp::QoS(1).transient_local().reliable());

    poses_.header.frame_id = conf_.frame_id;

    // weldid 0 is the world: those bodies are rigidly fixed to it and never need a second pose.
    for (int b = 1; b < nbody_; ++b) {
        if (model->body_weldid[b] != 0) dynamic_.push_back(b);
    }
    sent_.assign(dynamic_.size() * 7, 0.0);

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
               (has_scene_ ? manifest : "null");
    if (!conf_.env_url.empty()) {
        char placement[192];
        std::snprintf(placement, sizeof(placement),
                      ",\"env\":{\"url\":\"%s\",\"xyz\":[%.6g,%.6g,%.6g],\"yaw_deg\":%.6g,"
                      "\"scale\":%.6g}",
                      conf_.env_url.c_str(), conf_.env_xyz[0], conf_.env_xyz[1], conf_.env_xyz[2],
                      conf_.env_yaw_deg, conf_.env_scale);
        msg.data += placement;
        RCLCPP_INFO(node.get_logger(), "scenery: %s", conf_.env_url.c_str());
    }

    msg.data += static_poses_json(model);

    /* Whether the client should draw its stand-in floor. It replaces an unbounded MuJoCo plane,
     * which the exporter leaves out; a model whose floor is ordinary geometry already has one
     * exported, and a second at the same height z-fights with it. */
    bool has_plane = false;
    for (int g = 0; g < model->ngeom && !has_plane; ++g) {
        has_plane = model->geom_type[g] == mjGEOM_PLANE;
    }
    msg.data += has_plane ? ",\"ground\":true}" : ",\"ground\":false}";
    scene_pub_->publish(msg);

    RCLCPP_INFO(node.get_logger(), "streaming %zu of %d bodies: the rest are welded to the world",
                dynamic_.size(), nbody_);
}

bool BodyPosePublisher::wants_update(double sim_t) const
{
    if (period_s_ > 0.0 && sim_t < next_due_s_) {
        /* A reset rewinds sim time; without this the stream silently stalls until it catches up. */
        const bool rewound = sim_t + period_s_ < next_due_s_;
        if (!rewound) return false;
    }
    return poses_pub_->get_subscription_count() > 0;
}

void BodyPosePublisher::publish(const mjData *data)
{
    next_due_s_ = data->time + period_s_;

    /* Compared against the last frame actually sent, not the last frame seen, so a drift too slow
     * to clear the threshold still arrives once it has accumulated past it. */
    constexpr double kMovedM = 5e-4; // [m]
    constexpr double kMovedQ = 1e-5; // 1 - |dot|, about half a degree

    /* A client that connects to a world at rest would otherwise place nothing, because a frame of
     * only what changed is empty. This topic carries state, so a new subscriber - and anyone who
     * missed a best-effort frame - is owed the whole world periodically. */
    constexpr double kFull_s = 0.5;
    const size_t     subs    = poses_pub_->get_subscription_count();
    const bool       full = !have_sent_ || subs > last_subs_ || data->time - last_full_s_ > kFull_s;
    last_subs_            = subs;

    poses_.ids.clear();
    poses_.poses.clear();

    for (size_t i = 0; i < dynamic_.size(); ++i) {
        const int     b   = dynamic_[i];
        const mjtNum *p   = data->xpos + 3 * b;
        const mjtNum *q   = data->xquat + 4 * b; // MuJoCo order is (w, x, y, z)
        mjtNum       *ref = sent_.data() + 7 * i;

        bool moved = full;
        for (int k = 0; k < 3 && !moved; ++k) {
            moved = std::fabs(p[k] - ref[k]) > kMovedM;
        }
        /* The angle between the two orientations, not their components: a component-wise test has
         * no fixed meaning in radians, and at 5e-4 it was tripped every frame by a pizza jittering
         * 0.19 degrees in contact. */
        if (!moved) {
            const double dot = q[0] * ref[3] + q[1] * ref[4] + q[2] * ref[5] + q[3] * ref[6];
            moved            = (1.0 - std::fabs(dot)) > kMovedQ;
        }
        if (!moved) continue;

        geometry_msgs::msg::Pose out;
        out.position.x    = p[0];
        out.position.y    = p[1];
        out.position.z    = p[2];
        out.orientation.w = q[0];
        out.orientation.x = q[1];
        out.orientation.y = q[2];
        out.orientation.z = q[3];
        poses_.ids.push_back(b);
        poses_.poses.push_back(out);

        std::copy_n(p, 3, ref);
        std::copy_n(q, 4, ref + 3);
    }

    if (poses_.ids.empty()) return;

    have_sent_ = true;
    if (full) last_full_s_ = data->time;

    /* Node clock, not sim time: the client aligns these against its own clock offset, and a
     * recorded demonstration lines controller poses up with sim state on the ROS clock. */
    poses_.header.stamp = node_.now();
    poses_pub_->publish(poses_);
}

} // namespace vr
