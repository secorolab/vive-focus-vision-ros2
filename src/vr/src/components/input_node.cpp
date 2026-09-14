/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Vamsi Kalagaturu
 * See LICENSE for details. */

/* Raw in, clean out: /vr/raw/<x> -> /vr/<x>.
 *
 * Everything headset-specific lives here, so another OpenXR client changes this node and nothing
 * downstream: the vr_origin -> world calibration, and flagging a controller inactive because
 * VIVE keeps reporting a tracked pose for one lying on a table. */

#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <vr/msg/eye_gaze.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

namespace vr {
namespace {

/* Unity's XRHandJointID order, which is what the client iterates. Index 0 is the wrist, the
 * root every other joint hangs off; index 1 is the palm. TF wants each transform relative to
 * its parent, so this table also says who the parent is (-1 means the hand root itself). */
struct HandJoint
{
    const char *name;
    int         parent;
};

constexpr HandJoint kHandJoints[] = {
    { "wrist", -1 },              { "palm", 0 },
    { "thumb_metacarpal", 0 },    { "thumb_proximal", 2 },
    { "thumb_distal", 3 },        { "thumb_tip", 4 },
    { "index_metacarpal", 0 },    { "index_proximal", 6 },
    { "index_intermediate", 7 },  { "index_distal", 8 },
    { "index_tip", 9 },           { "middle_metacarpal", 0 },
    { "middle_proximal", 11 },    { "middle_intermediate", 12 },
    { "middle_distal", 13 },      { "middle_tip", 14 },
    { "ring_metacarpal", 0 },     { "ring_proximal", 16 },
    { "ring_intermediate", 17 },  { "ring_distal", 18 },
    { "ring_tip", 19 },           { "little_metacarpal", 0 },
    { "little_proximal", 21 },    { "little_intermediate", 22 },
    { "little_distal", 23 },      { "little_tip", 24 },
};
constexpr size_t kHandJointCount = sizeof(kHandJoints) / sizeof(kHandJoints[0]);

tf2::Transform to_tf(const geometry_msgs::msg::Pose &p)
{
    return tf2::Transform(
      tf2::Quaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w),
      tf2::Vector3(p.position.x, p.position.y, p.position.z));
}

} // namespace

class InputNode : public rclcpp::Node
{
  public:
    explicit InputNode(const rclcpp::NodeOptions &options) : Node("vr_input", options)
    {
        world_frame_  = declare_parameter<std::string>("world_frame", "world");
        origin_frame_ = declare_parameter<std::string>("origin_frame", "vr_origin");
        raw_ns_       = declare_parameter<std::string>("raw_ns", "/vr/raw");
        out_ns_       = declare_parameter<std::string>("out_ns", "/vr");
        const auto hands     = declare_parameter<std::vector<std::string>>(
          "hands", std::vector<std::string>{ "left", "right" });
        const auto head_name = declare_parameter<std::string>("head_name", "head");

        if (raw_ns_ == out_ns_) {
            throw std::runtime_error("raw_ns and out_ns must differ, or this node subscribes to "
                                     "its own output");
        }
        /* Play-space calibration. Identity until something measures the real offset between the
         * headset's guardian origin and the robot's world frame. */
        const auto xyz = declare_parameter<std::vector<double>>("origin_xyz", { 0.0, 0.0, 0.0 });
        const auto rpy = declare_parameter<std::vector<double>>("origin_rpy", { 0.0, 0.0, 0.0 });
        motion_eps_m_  = declare_parameter<double>("motion_eps_m", 0.003);
        stale_after_s_ = declare_parameter<double>("stale_after_s", 1.0);

        if (xyz.size() != 3 || rpy.size() != 3) {
            throw std::runtime_error("origin_xyz and origin_rpy must each have 3 elements");
        }
        tf2::Quaternion q;
        q.setRPY(rpy[0], rpy[1], rpy[2]);
        origin_T_.setRotation(q);
        origin_T_.setOrigin(tf2::Vector3(xyz[0], xyz[1], xyz[2]));

        tf_        = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
        publish_origin();

        std::vector<std::string> tracked_names = hands;
        tracked_names.push_back(head_name);
        for (const std::string &name : tracked_names) {
            pose_pubs_[name] = create_publisher<geometry_msgs::msg::PoseStamped>(
              out_ns_ + "/" + name + "/pose", rclcpp::SensorDataQoS());
            pose_subs_.push_back(create_subscription<geometry_msgs::msg::PoseStamped>(
              raw_ns_ + "/" + name + "/pose", rclcpp::SensorDataQoS(),
              [this, name](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                  on_pose(name, *msg);
              }));
        }

        for (const std::string &hand : hands) {
            active_pubs_[hand] = create_publisher<std_msgs::msg::Bool>(
              out_ns_ + "/" + hand + "/active", rclcpp::QoS(1).transient_local());
            joy_pubs_[hand] = create_publisher<sensor_msgs::msg::Joy>(
              out_ns_ + "/" + hand + "/joy", rclcpp::SensorDataQoS());
            joy_subs_.push_back(create_subscription<sensor_msgs::msg::Joy>(
              raw_ns_ + "/" + hand + "/joy", rclcpp::SensorDataQoS(),
              [this, hand](sensor_msgs::msg::Joy::SharedPtr msg) {
                  joy_pubs_[hand]->publish(*msg);
              }));

            /* The body the client's pointer is on. Passed through untouched: it is an index into
             * the manifest, not a pose, so calibration does not apply to it. */
            target_pubs_[hand] = create_publisher<std_msgs::msg::Int32>(
              out_ns_ + "/" + hand + "/target", rclcpp::SensorDataQoS());
            target_subs_.push_back(create_subscription<std_msgs::msg::Int32>(
              raw_ns_ + "/" + hand + "/target", rclcpp::SensorDataQoS(),
              [this, hand](std_msgs::msg::Int32::SharedPtr msg) {
                  target_pubs_[hand]->publish(*msg);
              }));
        }

        /* Hands and gaze are optional: the client only publishes them when the corresponding
         * OpenXR features are enabled, and a headset without them is still fully usable. */
        publish_hand_tf_ = declare_parameter<bool>("publish_hand_tf", true);
        for (const std::string &hand : hands) {
            joints_pubs_[hand] = create_publisher<geometry_msgs::msg::PoseArray>(
              out_ns_ + "/" + hand + "/joints", rclcpp::SensorDataQoS());
            hand_subs_.push_back(create_subscription<geometry_msgs::msg::PoseArray>(
              raw_ns_ + "/" + hand + "/joints", rclcpp::SensorDataQoS(),
              [this, hand](geometry_msgs::msg::PoseArray::SharedPtr msg) { on_hand(hand, *msg); }));
        }

        gaze_pub_ = create_publisher<vr::msg::EyeGaze>(out_ns_ + "/gaze",
                                                       rclcpp::SensorDataQoS());
        gaze_sub_ = create_subscription<vr::msg::EyeGaze>(
          raw_ns_ + "/gaze", rclcpp::SensorDataQoS(),
          [this](vr::msg::EyeGaze::SharedPtr msg) { on_gaze(*msg); });

        RCLCPP_INFO(get_logger(), "%s -> %s, calibration xyz [%.3f %.3f %.3f] rpy [%.3f %.3f %.3f]",
                    origin_frame_.c_str(), world_frame_.c_str(), xyz[0], xyz[1], xyz[2], rpy[0],
                    rpy[1], rpy[2]);
        RCLCPP_INFO(get_logger(), "inactive after %.1f s without %.0f mm of motion",
                    stale_after_s_, motion_eps_m_ * 1000.0);
    }

  private:
    struct Tracked
    {
        tf2::Vector3 last_pos{ 0, 0, 0 };
        rclcpp::Time last_motion;
        bool         active    = false;
        bool         seen      = false;
        bool         published = false;
    };

    void publish_origin()
    {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp            = now();
        tf.header.frame_id         = world_frame_;
        tf.child_frame_id          = origin_frame_;
        tf.transform.translation.x = origin_T_.getOrigin().x();
        tf.transform.translation.y = origin_T_.getOrigin().y();
        tf.transform.translation.z = origin_T_.getOrigin().z();
        tf.transform.rotation.x    = origin_T_.getRotation().x();
        tf.transform.rotation.y    = origin_T_.getRotation().y();
        tf.transform.rotation.z    = origin_T_.getRotation().z();
        tf.transform.rotation.w    = origin_T_.getRotation().w();
        static_tf_->sendTransform(tf);
    }

    void on_pose(const std::string &name, const geometry_msgs::msg::PoseStamped &raw)
    {
        const tf2::Transform in(
          tf2::Quaternion(raw.pose.orientation.x, raw.pose.orientation.y, raw.pose.orientation.z,
                          raw.pose.orientation.w),
          tf2::Vector3(raw.pose.position.x, raw.pose.position.y, raw.pose.position.z));
        const tf2::Transform out = origin_T_ * in;

        /* The client's stamp carries its own clock-offset correction; keep it so a recorded
         * demonstration lines up. Fall back only if it sends none. */
        const bool         client_stamped = raw.header.stamp.sec || raw.header.stamp.nanosec;
        const rclcpp::Time stamp = client_stamped ? rclcpp::Time(raw.header.stamp) : now();

        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp       = stamp;
        msg.header.frame_id    = world_frame_;
        msg.pose.position.x    = out.getOrigin().x();
        msg.pose.position.y    = out.getOrigin().y();
        msg.pose.position.z    = out.getOrigin().z();
        msg.pose.orientation.x = out.getRotation().x();
        msg.pose.orientation.y = out.getRotation().y();
        msg.pose.orientation.z = out.getRotation().z();
        msg.pose.orientation.w = out.getRotation().w();
        pose_pubs_[name]->publish(msg);

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp            = stamp;
        tf.header.frame_id         = world_frame_;
        tf.child_frame_id          = "vr_" + name;
        tf.transform.translation.x = msg.pose.position.x;
        tf.transform.translation.y = msg.pose.position.y;
        tf.transform.translation.z = msg.pose.position.z;
        tf.transform.rotation      = msg.pose.orientation;
        tf_->sendTransform(tf);

        update_activity(name, out.getOrigin(), stamp);
    }

    /** 26 joint poses in the play space -> one TF frame per joint, parented as a real hand. */
    void on_hand(const std::string &hand, const geometry_msgs::msg::PoseArray &msg)
    {
        if (msg.poses.size() != kHandJointCount) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "%s hand: expected %zu joints, got %zu; ignoring",
                                 hand.c_str(), kHandJointCount, msg.poses.size());
            return;
        }

        const bool         client_stamped = msg.header.stamp.sec || msg.header.stamp.nanosec;
        const rclcpp::Time stamp = client_stamped ? rclcpp::Time(msg.header.stamp) : now();

        /* Republished as an array as well as TF: a consumer that acts on a whole hand at once,
         * like grabbing, wants one timestamped snapshot rather than 26 lookups. */
        geometry_msgs::msg::PoseArray calibrated;
        calibrated.header.stamp    = stamp;
        calibrated.header.frame_id = world_frame_;
        calibrated.poses.resize(kHandJointCount);

        std::vector<geometry_msgs::msg::TransformStamped> transforms;
        transforms.reserve(kHandJointCount);
        for (size_t j = 0; j < kHandJointCount; ++j) {
            const tf2::Transform world_T_joint = origin_T_ * to_tf(msg.poses[j]);
            const int            parent        = kHandJoints[j].parent;

            auto &out             = calibrated.poses[j];
            out.position.x        = world_T_joint.getOrigin().x();
            out.position.y        = world_T_joint.getOrigin().y();
            out.position.z        = world_T_joint.getOrigin().z();
            out.orientation.x     = world_T_joint.getRotation().x();
            out.orientation.y     = world_T_joint.getRotation().y();
            out.orientation.z     = world_T_joint.getRotation().z();
            out.orientation.w     = world_T_joint.getRotation().w();

            /* TF stores each frame relative to its parent, so a child joint is expressed in its
             * parent's frame; only the wrist is placed in the world. */
            const tf2::Transform local =
              parent < 0 ? world_T_joint
                         : (origin_T_ * to_tf(msg.poses[parent])).inverse() * world_T_joint;

            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp    = stamp;
            tf.header.frame_id = parent < 0 ? world_frame_
                                            : "vr_" + hand + "_" + kHandJoints[parent].name;
            tf.child_frame_id  = "vr_" + hand + "_" + kHandJoints[j].name;
            tf.transform.translation.x = local.getOrigin().x();
            tf.transform.translation.y = local.getOrigin().y();
            tf.transform.translation.z = local.getOrigin().z();
            tf.transform.rotation.x    = local.getRotation().x();
            tf.transform.rotation.y    = local.getRotation().y();
            tf.transform.rotation.z    = local.getRotation().z();
            tf.transform.rotation.w    = local.getRotation().w();
            transforms.push_back(tf);
        }
        joints_pubs_[hand]->publish(calibrated);
        if (publish_hand_tf_) tf_->sendTransform(transforms);
    }

    /** Gaze is a ray per eye; calibration moves it from the play space into the world. */
    void on_gaze(const vr::msg::EyeGaze &raw)
    {
        vr::msg::EyeGaze out = raw;
        const bool client_stamped = raw.header.stamp.sec || raw.header.stamp.nanosec;
        out.header.stamp    = client_stamped ? raw.header.stamp
                                             : builtin_interfaces::msg::Time(now());
        out.header.frame_id = world_frame_;

        const auto calibrate = [this](const geometry_msgs::msg::Pose &in,
                                      geometry_msgs::msg::Pose       &dst) {
            const tf2::Transform t = origin_T_ * to_tf(in);
            dst.position.x    = t.getOrigin().x();
            dst.position.y    = t.getOrigin().y();
            dst.position.z    = t.getOrigin().z();
            dst.orientation.x = t.getRotation().x();
            dst.orientation.y = t.getRotation().y();
            dst.orientation.z = t.getRotation().z();
            dst.orientation.w = t.getRotation().w();
        };
        calibrate(raw.left, out.left);
        calibrate(raw.right, out.right);
        gaze_pub_->publish(out);
    }

    void update_activity(const std::string &name, const tf2::Vector3 &pos,
                         const rclcpp::Time &stamp)
    {
        auto it = active_pubs_.find(name);
        if (it == active_pubs_.end()) return; // head has no active flag

        Tracked &t = tracked_[name];
        if (!t.seen) {
            t.seen        = true;
            t.last_pos    = pos;
            t.last_motion = stamp;
        }
        if ((pos - t.last_pos).length() > motion_eps_m_) {
            t.last_pos    = pos;
            t.last_motion = stamp;
        }

        const bool active = (stamp - t.last_motion).seconds() < stale_after_s_;
        if (active == t.active && t.published) return;
        t.active    = active;
        t.published = true;
        std_msgs::msg::Bool msg;
        msg.data = active;
        it->second->publish(msg);
    }

    std::string    world_frame_, origin_frame_, raw_ns_, out_ns_;
    tf2::Transform origin_T_;
    double         motion_eps_m_  = 0.003;
    double         stale_after_s_ = 1.0;

    std::unordered_map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr>
                                                                                        pose_pubs_;
    std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr> joy_pubs_;
    std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> active_pubs_;
    std::unordered_map<std::string, Tracked>                                           tracked_;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr>       pose_subs_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr>                 joy_subs_;
    std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr>
                                                                                     target_pubs_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr>               target_subs_;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr>         hand_subs_;
    std::unordered_map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr>
                                                                                     joints_pubs_;
    rclcpp::Publisher<vr::msg::EyeGaze>::SharedPtr                                      gaze_pub_;
    rclcpp::Subscription<vr::msg::EyeGaze>::SharedPtr                                   gaze_sub_;
    bool                                                                       publish_hand_tf_ = true;
    std::unique_ptr<tf2_ros::TransformBroadcaster>                                      tf_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster>                                static_tf_;
};

} // namespace vr

RCLCPP_COMPONENTS_REGISTER_NODE(vr::InputNode)
