/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Vamsi Kalagaturu
 * See LICENSE for details. */

/* Runs an MJCF world and streams it to the headset. Nothing about the headset's input reaches
 * here; that is InputNode's job.
 *
 * The world is built through mj_kdl::Env rather than mj_loadXML, which is what makes the scene
 * composable (robots, attachments, floor, skybox, objects) and gives episode reset a single
 * correct implementation - reset() restores a keyframe and re-synchronises every registered
 * Robot's command ports, which a bare mj_resetData does not.
 *
 * For a simulation an application already owns, use vr::BodyPosePublisher directly instead of
 * running this node; it takes only mjModel/mjData and pulls in no KDL. */

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include <mj_kdl_wrapper/mj_kdl_wrapper.hpp>
#include <mujoco/mujoco.h>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "vr/body_pose_publisher.hpp"

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
        conf.frame_id      = declare_parameter<std::string>("frame_id", "world");
        conf.rate_hz       = declare_parameter<double>("rate_hz", 60.0);
        conf.topic_ns      = declare_parameter<std::string>("out_ns", "/vr");
        rate_hz_           = conf.rate_hz;

        const auto pc_time_topic = declare_parameter<std::string>("pc_time_topic", "/vr/pc_time");
        const auto pc_time_rate  = declare_parameter<double>("pc_time_rate_hz", 10.0);
        const auto timestep      = declare_parameter<double>("timestep", 0.002);
        const auto add_floor     = declare_parameter<bool>("add_floor", false);
        const auto add_skybox    = declare_parameter<bool>("add_skybox", false);
        const auto gravity_z     = declare_parameter<double>("gravity_z", -9.81);

        if (mjcf.empty()) {
            throw std::runtime_error("parameter 'model' (path to an MJCF file) is required");
        }

        /* One MJCF is the degenerate scene: a single root with nothing attached. Composing more
         * is a matter of adding RobotSpec / SceneObject entries here. */
        mj_kdl::RobotSpec root;
        root.path = mjcf.c_str();
        spec_.robots.push_back(root);
        spec_.timestep   = timestep;
        spec_.gravity_z  = gravity_z;
        spec_.add_floor  = add_floor;
        spec_.add_skybox = add_skybox;

        if (!mj_kdl::init_env(&env_, &spec_)) {
            throw std::runtime_error("failed to build a scene from " + mjcf);
        }

        scene_out_ = std::make_unique<BodyPosePublisher>(*this, env_.model, conf);

        /* The client estimates its clock offset against this; see ClockSync on the Unity side. */
        pc_time_ = create_publisher<builtin_interfaces::msg::Time>(pc_time_topic,
                                                                   rclcpp::SensorDataQoS());
        const auto pc_time_period = std::chrono::duration<double>(1.0 / pc_time_rate);
        time_timer_               = create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(pc_time_period), [this] {
              builtin_interfaces::msg::Time msg = now();
              pc_time_->publish(msg);
          });

        reset_srv_ = create_service<std_srvs::srv::Trigger>(
          "~/reset",
          [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                 std_srvs::srv::Trigger::Response::SharedPtr response) {
              const mj_kdl::ResetInfo info = mj_kdl::reset(&env_);
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
        /* Advance one publish period of simulated time, so the stream stays in step with the
         * wall clock rather than running as fast as the machine allows. */
        const mjtNum target = env_.data->time + 1.0 / rate_hz_;
        while (env_.data->time < target) mj_step(env_.model, env_.data);

        if (scene_out_->wants_update(env_.data->time)) scene_out_->publish(env_.data);
    }

    mj_kdl::SceneSpec spec_;
    mj_kdl::Env       env_;
    double            rate_hz_ = 60.0;

    std::unique_ptr<BodyPosePublisher>                          scene_out_;
    rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr pc_time_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr          reset_srv_;
    rclcpp::TimerBase::SharedPtr                                sim_timer_, time_timer_;
};

} // namespace vr

RCLCPP_COMPONENTS_REGISTER_NODE(vr::SceneNode)
