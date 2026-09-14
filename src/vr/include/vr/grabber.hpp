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
#include <std_msgs/msg/int32.hpp>

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

    /* The client points a ray and publishes what it hit on <topic_ns>/<hand>/target; reach_m is
     * only the fallback for a hand that has no pointer, which is the tracked-hand case. */
    double reach_m          = 0.15;  // a body further than this from the hand is not grabbed

    /* Accelerations, not forces: scaled by each body's mass and inertia so the same numbers
     * behave the same on a loose ball and on a robot link. kp/kd are a critically damped
     * second-order response, omega = sqrt(kp) and kd = 2*omega. */
    double kp               = 400.0; // [1/s^2]
    double kd               = 40.0;  // [1/s]
    double kp_rot           = 100.0; // [1/s^2]
    double kd_rot           = 20.0;  // [1/s]
    double max_accel        = 50.0;  // [m/s^2]    clamped, or a far reach launches the object
    double max_ang_accel    = 100.0; // [rad/s^2]

    /* The hand velocity the damper is given is differenced from poses that arrive over Wi-Fi in
     * bursts, so it needs bounding and smoothing or the grab turns jittery. */
    double vel_filter        = 0.3;  // low-pass weight on each new sample, 1 disables it
    double max_hand_speed    = 4.0;  // [m/s]
    double max_hand_turn_rate = 15.0; // [rad/s]
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
        int    target    = -1;    // what the client's ray is on, or -1 for nothing
        mjtNum pos[3]  = { 0, 0, 0 }; // latest controller position, world frame
        mjtNum quat[4] = { 1, 0, 0, 0 };
        /* Pose of the body in the controller's frame at the moment it was grabbed, so it keeps
         * its offset instead of snapping into the hand. */
        mjtNum grab_pos[3]  = { 0, 0, 0 };
        mjtNum grab_quat[4] = { 1, 0, 0, 0 };
        /* Hand velocity, differenced between pose messages rather than between simulation steps:
         * apply() runs at the timestep (500 Hz) while poses arrive at about 50 Hz, so a per-step
         * difference is zero on most steps and a large spike on the rest. The damper needs a
         * usable value on every step or following a moving hand leaves a standing error. */
        bool   have_vel      = false;
        double last_stamp_s  = 0.0;
        mjtNum last_pos[3]   = { 0, 0, 0 };
        mjtNum last_quat[4]  = { 1, 0, 0, 0 };
        mjtNum lin_vel[3]    = { 0, 0, 0 };
        mjtNum ang_vel[3]    = { 0, 0, 0 };
    };

    void on_pose(const std::string &hand, const geometry_msgs::msg::PoseStamped &msg);
    void on_joy(const std::string &hand, const sensor_msgs::msg::Joy &msg);
    void on_joints(const std::string &hand, const geometry_msgs::msg::PoseArray &msg);
    void on_target(const std::string &hand, const std_msgs::msg::Int32 &msg);
    int  nearest_body(const mjData *data, const mjtNum point[3]) const;

    GrabConf                                   conf_;
    const mjModel                             *model_ = nullptr;
    rclcpp::Node                              &node_;
    std::unordered_map<std::string, Hand>      hands_;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
};

} // namespace vr
