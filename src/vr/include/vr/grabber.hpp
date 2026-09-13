/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Vamsi Kalagaturu
 * See LICENSE for details. */

#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include <mujoco/mujoco.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace vr {

/** What counts as a grab, and how hard a grabbed body is then pulled. */
struct GrabConf
{
    std::string topic_ns    = "/vr"; // -> <topic_ns>/<hand>/{pose,joy,joints}
    std::vector<std::string> hands = { "left", "right" };

    /* Both sources can run at once: whichever grabs first owns the body. Tracked hands are the
     * more natural interaction and the better demonstration data; controllers are steadier and
     * their button is unambiguous, which matters for a long session. */
    bool   use_controllers  = true;
    bool   use_hands        = true;
    int    grab_button      = 1;     // index into sensor_msgs/Joy buttons; 1 is squeeze
    double pinch_close_m    = 0.025; // thumb-to-index distance that starts a pinch
    double pinch_open_m     = 0.040; // and the wider distance that ends it, so it cannot chatter

    double reach_m          = 0.15;  // a body further than this from the hand is not grabbed
    double kp               = 400.0; // [N/m]
    double kd               = 40.0;  // [Ns/m]
    double kp_rot           = 15.0;  // [Nm/rad]
    double kd_rot           = 2.0;   // [Nms/rad]
    double max_force        = 200.0; // [N]  clamped, or a far reach launches the object
    double max_torque       = 20.0;  // [Nm]
};

/**
 * Lets the user pick up and push simulated bodies with the controllers.
 *
 * A force source, not a simulator: it creates no node, no executor and no thread, and owns no
 * mjData. Feed it a model once, call apply() from whatever owns the simulation loop:
 *
 *   vr::Grabber grab(*node, model, conf);
 *   while (running) {
 *       grab.apply(data);        // writes xfrc_applied for held bodies
 *       mj_step(model, data);
 *   }
 *
 * Held bodies are pulled by a spring-damper rather than teleported, so mass, contact and the
 * rest of the simulation still decide what actually happens - an object too heavy to lift stays
 * put, and pushing a robot link fights its actuators. That is also what makes the resulting
 * motion worth recording as demonstration data.
 *
 * The same mechanism covers both cases asked of it: a free body is picked up, and a body that
 * belongs to an articulated chain is pushed.
 */
class Grabber
{
  public:
    Grabber(rclcpp::Node &node, const mjModel *model, GrabConf conf);

    /** Writes xfrc_applied for every held body. Call once per step, before mj_step. */
    void apply(mjData *data);

    /** Body currently held by a hand, or -1. */
    int held_body(const std::string &hand) const;

  private:
    struct Hand
    {
        bool   pressed   = false; // controller button
        bool   pinching  = false; // tracked hand, with hysteresis
        bool   have_pose = false;
        int    body      = -1;
        mjtNum pos[3]  = { 0, 0, 0 }; // latest controller position, world frame
        mjtNum quat[4] = { 1, 0, 0, 0 };
        /* Pose of the body in the controller's frame at the moment it was grabbed, so it keeps
         * its offset instead of snapping into the hand. */
        mjtNum grab_pos[3]  = { 0, 0, 0 };
        mjtNum grab_quat[4] = { 1, 0, 0, 0 };
    };

    void on_pose(const std::string &hand, const geometry_msgs::msg::PoseStamped &msg);
    void on_joy(const std::string &hand, const sensor_msgs::msg::Joy &msg);
    void on_joints(const std::string &hand, const geometry_msgs::msg::PoseArray &msg);
    int  nearest_body(const mjData *data, const mjtNum point[3]) const;

    GrabConf                                   conf_;
    const mjModel                             *model_ = nullptr;
    rclcpp::Node                              &node_;
    std::unordered_map<std::string, Hand>      hands_;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
};

} // namespace vr
