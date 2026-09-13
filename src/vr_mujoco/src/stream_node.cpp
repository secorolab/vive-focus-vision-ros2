/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Vamsi Kalagaturu
 * See LICENSE for details. */

/* Runs an MJCF model and publishes what the headset client needs to render it, plus republishes
 * the client's controller poses as TF so the link can be checked in rviz without a headset on.
 *
 * Poses are MuJoCo world coordinates, which are already REP-103 (Z up, right-handed); the
 * conversion to the client's left-handed Y-up space happens on the client, once, at its own
 * publish/apply boundary. */

#include <chrono>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>

#include <mujoco/mujoco.h>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>

using namespace std::chrono_literals;

namespace {

/** Button order the client fills in; documented here because sensor_msgs/Joy is untyped. */
constexpr const char *kJoyLayout =
  "buttons=[trigger, squeeze, primary(X|A), secondary(Y|B), stick_click, menu] "
  "axes=[stick_x, stick_y, trigger, squeeze]";

std::string read_file(const std::string &path)
{
    std::ifstream in(path);
    if (!in) return {};
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

} // namespace

class StreamNode : public rclcpp::Node
{
  public:
    StreamNode() : Node("vr_mujoco_stream")
    {
        const std::string mjcf = declare_parameter<std::string>("model", "");
        manifest_path_         = declare_parameter<std::string>("manifest", "");
        scene_url_             = declare_parameter<std::string>("scene_url", "");
        frame_id_              = declare_parameter<std::string>("frame_id", "world");
        rate_hz_               = declare_parameter<double>("rate_hz", 60.0);

        if (mjcf.empty()) {
            throw std::runtime_error("parameter 'model' (path to an MJCF file) is required");
        }

        char error[1024] = "";
        model_           = mj_loadXML(mjcf.c_str(), nullptr, error, sizeof(error));
        if (!model_) throw std::runtime_error("failed to load " + mjcf + ": " + error);
        data_ = mj_makeData(model_);
        mj_forward(model_, data_);

        /* Best effort: a dropped pose frame is replaced by the next one 16 ms later, and a
         * queue would only add latency to a stream the client renders live. */
        const auto stream_qos = rclcpp::SensorDataQoS();
        body_poses_ = create_publisher<geometry_msgs::msg::PoseArray>("/vr/body_poses", stream_qos);
        pc_time_ = create_publisher<builtin_interfaces::msg::Time>("/vr/pc_time", stream_qos);

        /* Transient local: the client subscribes whenever it connects, which is long after the
         * scene was first published. */
        scene_ = create_publisher<std_msgs::msg::String>(
          "/vr/scene", rclcpp::QoS(1).transient_local().reliable());

        tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        for (const auto &hand : { std::string("left"), std::string("right") }) {
            pose_subs_.push_back(create_subscription<geometry_msgs::msg::PoseStamped>(
              "/vr/" + hand + "/pose", rclcpp::SensorDataQoS(),
              [this, hand](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                  broadcast_controller(hand, *msg);
              }));
            joy_subs_.push_back(create_subscription<sensor_msgs::msg::Joy>(
              "/vr/" + hand + "/joy", rclcpp::SensorDataQoS(),
              [this, hand](sensor_msgs::msg::Joy::SharedPtr msg) { log_joy(hand, *msg); }));
        }
        head_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
          "/vr/head/pose", rclcpp::SensorDataQoS(),
          [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
              broadcast_controller("head", *msg);
          });

        publish_scene();

        const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
        sim_timer_        = create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(period), [this] { tick(); });
        time_timer_ = create_wall_timer(100ms, [this] {
            builtin_interfaces::msg::Time msg = now();
            pc_time_->publish(msg);
        });

        RCLCPP_INFO(get_logger(), "model %s: %ld bodies, %ld geoms, timestep %.4f s", mjcf.c_str(),
                    static_cast<long>(model_->nbody), static_cast<long>(model_->ngeom),
                    model_->opt.timestep);
        RCLCPP_INFO(get_logger(), "streaming /vr/body_poses at %.0f Hz in frame '%s'", rate_hz_,
                    frame_id_.c_str());
        RCLCPP_INFO(get_logger(), "expecting %s", kJoyLayout);
    }

    ~StreamNode() override
    {
        if (data_) mj_deleteData(data_);
        if (model_) mj_deleteModel(model_);
    }

  private:
    void publish_scene()
    {
        const std::string manifest = read_file(manifest_path_);
        if (manifest.empty()) {
            RCLCPP_WARN(get_logger(),
                        "no manifest at '%s'; publishing /vr/scene with a url only. Run "
                        "scene_export first.",
                        manifest_path_.c_str());
        }
        std_msgs::msg::String msg;
        msg.data = "{\"url\":\"" + scene_url_ + "\",\"manifest\":" +
                   (manifest.empty() ? "null" : manifest) + "}";
        scene_->publish(msg);
    }

    void tick()
    {
        /* Advance one publish period of simulated time, so the stream stays in step with the
         * wall clock rather than running as fast as the machine allows. */
        const mjtNum target = data_->time + 1.0 / rate_hz_;
        while (data_->time < target) mj_step(model_, data_);

        geometry_msgs::msg::PoseArray msg;
        msg.header.stamp    = now();
        msg.header.frame_id = frame_id_;
        msg.poses.resize(model_->nbody);
        for (int b = 0; b < model_->nbody; ++b) {
            const mjtNum *p = data_->xpos + 3 * b;
            const mjtNum *q = data_->xquat + 4 * b; // MuJoCo order is (w, x, y, z)
            auto         &out = msg.poses[b];
            out.position.x    = p[0];
            out.position.y    = p[1];
            out.position.z    = p[2];
            out.orientation.w = q[0];
            out.orientation.x = q[1];
            out.orientation.y = q[2];
            out.orientation.z = q[3];
        }
        body_poses_->publish(msg);
    }

    void broadcast_controller(const std::string &name, const geometry_msgs::msg::PoseStamped &msg)
    {
        geometry_msgs::msg::TransformStamped tf;
        /* Client stamps carry its clock-offset correction; fall back only if it sends none. */
        const bool client_stamped = msg.header.stamp.sec || msg.header.stamp.nanosec;
        tf.header.stamp           = client_stamped ? msg.header.stamp
                                                   : builtin_interfaces::msg::Time(now());
        tf.header.frame_id        = frame_id_;
        tf.child_frame_id          = "vr_" + name;
        tf.transform.translation.x = msg.pose.position.x;
        tf.transform.translation.y = msg.pose.position.y;
        tf.transform.translation.z = msg.pose.position.z;
        tf.transform.rotation      = msg.pose.orientation;
        tf_->sendTransform(tf);
    }

    void log_joy(const std::string &hand, const sensor_msgs::msg::Joy &msg)
    {
        /* Throttled: this exists so bring-up can see buttons arrive without a rosbag. */
        std::ostringstream ss;
        for (int b : msg.buttons) ss << b;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "%s joy buttons=%s axes=%zu",
                             hand.c_str(), ss.str().c_str(), msg.axes.size());
    }

    mjModel *model_ = nullptr;
    mjData  *data_  = nullptr;

    std::string manifest_path_, scene_url_, frame_id_;
    double      rate_hz_ = 60.0;

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr    body_poses_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            scene_;
    rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr    pc_time_;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_subs_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr>           joy_subs_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr              head_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>                                tf_;
    rclcpp::TimerBase::SharedPtr sim_timer_, time_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<StreamNode>());
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("vr_mujoco_stream"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
