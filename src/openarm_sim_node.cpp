/* SPDX-License-Identifier: MIT */

/* Robot-specific simulation. The generic SceneNode remains a scene viewer;
 * this application owns physics and uses BodyPosePublisher as documented in embedding.md. */

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>

#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "vive_vr_ros2/body_pose_publisher.hpp"
#include "vive_vr_ros2/openarm_teleop.hpp"

namespace vive_vr_ros2 {

class OpenArmSimNode : public rclcpp::Node
{
  public:
    OpenArmSimNode() : Node("vive_scene")
    {
        const auto filename = declare_parameter<std::string>("model", "");
        SceneConf conf;
        conf.manifest_path = declare_parameter<std::string>("manifest", "");
        conf.scene_url = declare_parameter<std::string>("scene_url", "");
        conf.topic_ns = declare_parameter<std::string>("out_ns", "/vive_vr");
        conf.frame_id = declare_parameter<std::string>("frame_id", "world");
        conf.rate_hz = declare_parameter<double>("rate_hz", 60.0);
        rate_hz_ = conf.rate_hz;
        const double timestep = declare_parameter<double>("timestep", 0.002);
        const double gravity = declare_parameter<double>("gravity_z", 0.0);
        reset_keyframe_ = declare_parameter<int>("reset_keyframe", 0);
        if (!std::isfinite(rate_hz_) || rate_hz_ <= 0 ||
            !std::isfinite(timestep) || timestep <= 0) {
            throw std::runtime_error("Simulation rates must be finite and positive");
        }
        char error[1024] = {};
        model_.reset(mj_loadXML(filename.c_str(), nullptr, error, sizeof(error)));
        if (!model_) throw std::runtime_error(error);
        if (reset_keyframe_ < 0 || reset_keyframe_ >= model_->nkey) {
            throw std::runtime_error("OpenArm model needs a valid home keyframe; run prepare");
        }
        model_->opt.timestep = timestep;
        model_->opt.gravity[2] = gravity;
        data_.reset(mj_makeData(model_.get()));
        if (!data_) throw std::runtime_error("Cannot allocate simulation state");
        reset_home();
        scene_out_ = std::make_unique<BodyPosePublisher>(*this, model_.get(), conf);
        if (declare_parameter<bool>("openarm.enabled", false)) {
            teleop_ = std::make_unique<OpenArmTeleop>(*this, model_.get(), data_.get());
        }
        reset_srv_ = create_service<std_srvs::srv::Trigger>(
          "~/reset", [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                            std_srvs::srv::Trigger::Response::SharedPtr response) {
              reset_home();
              if (teleop_) teleop_->reset();
              response->success = true;
              response->message = "OpenArm simulation reset; release grip before re-engaging";
          });
        const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
        timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                  [this] { tick(); });
        RCLCPP_INFO(get_logger(), "OpenArm simulation ready; IK uses Orocos KDL, hardware is not connected");
    }

  private:
    void reset_home()
    {
        mj_resetDataKeyframe(model_.get(), data_.get(), reset_keyframe_);
        mj_forward(model_.get(), data_.get());
    }

    void tick()
    {
        if (teleop_) teleop_->tick(1.0 / rate_hz_);
        const double target = data_->time + 1.0 / rate_hz_;
        while (data_->time < target) mj_step(model_.get(), data_.get());
        if (scene_out_->wants_update(data_->time)) scene_out_->publish(data_.get());
    }

    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model_{ nullptr, mj_deleteModel };
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data_{ nullptr, mj_deleteData };
    std::unique_ptr<BodyPosePublisher> scene_out_;
    std::unique_ptr<OpenArmTeleop> teleop_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
    double rate_hz_ = 60.0;
    int reset_keyframe_ = 0;
};

} // namespace vive_vr_ros2

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<vive_vr_ros2::OpenArmSimNode>());
    } catch (const std::exception &error) {
        RCLCPP_ERROR(rclcpp::get_logger("openarm_sim"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
