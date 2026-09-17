/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Vamsi Kalagaturu
 * See LICENSE for details. */

/* Runs an MJCF world and streams it to the headset; input is InputNode's job.
 *
 * Built through mj_kdl::Env rather than mj_loadXML for composable scenes and for reset(), which
 * re-synchronises Robot command ports that a bare mj_resetData leaves stale.
 *
 * A simulation an application already owns should use vr::BodyPosePublisher directly. */

#include <chrono>
#include <algorithm>
#include <memory>
#include <stdexcept>
#include <map>
#include <string>
#include <vector>

#include <mj_kdl_wrapper/mj_kdl_wrapper.hpp>
#include <mujoco/mujoco.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "vr/body_pose_publisher.hpp"
#include "vr/grabber.hpp"

namespace vr {

class SceneNode : public rclcpp::Node
{
  public:
    explicit SceneNode(const rclcpp::NodeOptions &options) : Node("vr_scene", options)
    {
        const std::string mjcf = declare_parameter<std::string>("model", "");
        SceneConf conf;
        conf.manifest_path = declare_parameter<std::string>("manifest", "");
        conf.scene_url     = declare_parameter<std::string>("scene_url", "");

        /* Scenery the client draws and nothing simulates; see SceneConf. Placed here because the
         * environment's own origin will not be the world origin. */
        conf.env_url     = declare_parameter<std::string>("env_url", "");
        conf.env_yaw_deg = declare_parameter<double>("env_yaw_deg", 0.0);
        conf.env_scale   = declare_parameter<double>("env_scale", 1.0);
        const auto env_xyz =
          declare_parameter<std::vector<double>>("env_xyz", { 0.0, 0.0, 0.0 });
        if (env_xyz.size() == 3) std::copy(env_xyz.begin(), env_xyz.end(), conf.env_xyz);
        conf.frame_id      = declare_parameter<std::string>("frame_id", "world");
        conf.rate_hz       = declare_parameter<double>("rate_hz", 60.0);
        conf.topic_ns      = declare_parameter<std::string>("out_ns", "/vr");
        rate_hz_           = conf.rate_hz;

        const auto timestep      = declare_parameter<double>("timestep", 0.002);
        const auto gravity_z     = declare_parameter<double>("gravity_z", -9.81);

        if (mjcf.empty()) {
            throw std::runtime_error("parameter 'model' (path to an MJCF file) is required");
        }

        /* Loaded directly, not through build_scene: that attaches only the first root body of
         * each RobotSpec, which is right for a robot and silently truncates a world file with
         * several top-level bodies. The model here has to match what scene_export read, or the
         * client indexes body poses into the wrong geometry.
         *
         * Env still owns the result, so reset() keeps working. */
        mj_kdl::ensure_plugins_loaded();
        char error[1024] = "";
        env_.model       = mj_loadXML(mjcf.c_str(), nullptr, error, sizeof(error));
        if (!env_.model) throw std::runtime_error("failed to load " + mjcf + ": " + error);
        env_.model->opt.timestep  = timestep;
        env_.model->opt.gravity[2] = gravity_z;
        env_.data                 = mj_makeData(env_.model);
        mj_forward(env_.model, env_.data);

        scene_out_ = std::make_unique<BodyPosePublisher>(*this, env_.model, conf);

        if (declare_parameter<bool>("enable_grab", true)) {
            GrabConf grab;
            grab.topic_ns   = declare_parameter<std::string>("out_ns_grab", conf.topic_ns);
            grab.grab_button = declare_parameter<int>("grab_button", 4);
            grab.reach_m    = declare_parameter<double>("grab_reach_m", 0.15);
            grab.kp            = declare_parameter<double>("grab_kp", 400.0);
            grab.kd            = declare_parameter<double>("grab_kd", 40.0);
            grab.kp_rot        = declare_parameter<double>("grab_kp_rot", 100.0);
            grab.kd_rot        = declare_parameter<double>("grab_kd_rot", 20.0);
            grab.max_accel     = declare_parameter<double>("grab_max_accel", 150.0);
            grab.max_ang_accel = declare_parameter<double>("grab_max_ang_accel", 100.0);
            grab.vel_filter    = declare_parameter<double>("grab_vel_filter", 0.3);
            grab.max_hand_speed = declare_parameter<double>("grab_max_hand_speed", 4.0);
            grab.max_hand_turn_rate =
              declare_parameter<double>("grab_max_hand_turn_rate", 15.0);
            grabber_ = std::make_unique<Grabber>(*this, env_.model, grab);

            for (const std::string &hand : grab.hands) {
                held_pubs_[hand] = create_publisher<std_msgs::msg::Int32>(
                  grab.topic_ns + "/" + hand + "/held", rclcpp::QoS(1).transient_local());
                last_held_[hand] = -2; // not -1, so the first "holding nothing" is still published
            }
        }

        /* -1 restores the model's own initial pose. A robot's keyframe only covers that robot's
         * joints, so in a world with free bodies it resets them to zero - dropping every loose
         * object at the origin - rather than to where the MJCF put them. */
        reset_keyframe_ = declare_parameter<int>("reset_keyframe", -1);

        reset_srv_ = create_service<std_srvs::srv::Trigger>(
          "~/reset",
          [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                 std_srvs::srv::Trigger::Response::SharedPtr response) {
              mj_kdl::ResetOptions options;
              options.use_keyframe         = reset_keyframe_ >= 0;
              options.keyframe             = reset_keyframe_;
              const mj_kdl::ResetInfo info = mj_kdl::reset(&env_, &options);
              response->success            = true;
              response->message = info.used_keyframe
                                    ? "reset to keyframe " + std::to_string(info.keyframe)
                                    : "reset to the model's initial state";
              RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
          });

        const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
        sim_timer_        = create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(period), [this] { tick(); });

        RCLCPP_INFO(get_logger(), "model %s: %ld bodies, %ld geoms, timestep %.4f s", mjcf.c_str(),
                    static_cast<long>(env_.model->nbody), static_cast<long>(env_.model->ngeom),
                    env_.model->opt.timestep);
        RCLCPP_INFO(get_logger(), "streaming bodies at %.0f Hz in frame '%s'", rate_hz_,
                    conf.frame_id.c_str());
    }

    ~SceneNode() override { mj_kdl::cleanup(&env_); }

  private:
    void tick()
    {
        /* One publish period of sim time, so the stream tracks the wall clock. */
        const mjtNum target = env_.data->time + 1.0 / rate_hz_;
        while (env_.data->time < target) {
            if (grabber_) grabber_->apply(env_.data);
            mj_step(env_.model, env_.data);
        }

        if (scene_out_->wants_update(env_.data->time)) scene_out_->publish(env_.data);
        publish_held();
    }

    /* What each hand actually holds, so the client can show it. Only the grabber knows: it
     * refuses a massless body, and a tracked hand pinches without the client selecting anything.
     * Published on change - it is a latched fact, not a stream. */
    void publish_held()
    {
        if (!grabber_) return;
        for (auto &[hand, pub] : held_pubs_) {
            const int body = grabber_->held_body(hand);
            if (body == last_held_[hand]) continue;
            last_held_[hand] = body;
            std_msgs::msg::Int32 msg;
            msg.data = body;
            pub->publish(msg);
        }
    }

    mj_kdl::Env       env_;
    double            rate_hz_ = 60.0;
    int               reset_keyframe_ = -1;

    std::unique_ptr<BodyPosePublisher>                          scene_out_;
    std::unique_ptr<Grabber>                                    grabber_;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> held_pubs_;
    std::map<std::string, int>                                               last_held_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr          reset_srv_;
    rclcpp::TimerBase::SharedPtr                                sim_timer_;
};

} // namespace vr

RCLCPP_COMPONENTS_REGISTER_NODE(vr::SceneNode)
