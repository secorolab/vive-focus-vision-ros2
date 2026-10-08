/* SPDX-License-Identifier: MIT */
// Hardware-feedback bridge. Preview by default; live output requires explicit arming and fresh guarded-driver health.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include "vive_vr_ros2/hardware_guard.hpp"
#include "vive_vr_ros2/body_pose_publisher.hpp"
#include "vive_vr_ros2/openarm_teleop.hpp"

namespace vive_vr_ros2 {
class HardwarePreview : public rclcpp::Node {
    using Clock = std::chrono::steady_clock;
    struct Axis { std::string name; int joint, qpos, motor; };
    struct Arm {
        std::vector<Axis> axes;
        std::unique_ptr<OpenArmTeleop> teleop;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr preview, command, gripper;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr health;
        Clock::time_point healthy_at{};
        bool healthy = false, sent = false;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr hand_pose;
        Clock::time_point joy_at{}, hand_at{};
        int64_t joy_stamp=0, hand_stamp=0;
        bool gripper_ready=false, gripper_sent=false;
        int gripper_direction=0; // +1 opens, -1 closes; zero requires a fresh engagement.
        double gripper_last=0, gripper_velocity=0;
        std::array<double,8> last{};
    };
public:
    HardwarePreview() : Node("openarm_hardware_preview") {
        live_output_ = declare_parameter<bool>("hardware.enable_commands", false);
        grippers_ = declare_parameter<bool>("hardware.control_grippers", false);
        const auto filename = declare_parameter<std::string>("model", "");
        declare_parameter<std::string>("out_ns", "/vive_vr");
        declare_parameter<std::string>("openarm.ee_topic_prefix", "/vive_vr/hardware");
        // Hardware preview deliberately uses conservative values, never simulation tuning.
        declare_parameter<double>("openarm.translation_scale", 1.0);
        declare_parameter<double>("openarm.max_translation_m", 0.50);
        declare_parameter<double>("openarm.max_rotation_rad", 3.141592653589793);
        declare_parameter<bool>("openarm.stop_at_motion_bounds", false);
        declare_parameter<double>("openarm.joint_speed_rad_s", 0.3);
        declare_parameter<double>("openarm.max_linear_speed_m_s", 0.10);
        declare_parameter<double>("openarm.max_angular_speed_rad_s", 0.5);
        declare_parameter<double>("openarm.posture_gain_hz", 0.0);
        declare_parameter<double>("openarm.tracking_time_constant_s", 0.15);
        declare_parameter<double>("openarm.finger_speed_m_s", 0.015);
        char error[1024] = {};
        model_.reset(mj_loadXML(filename.c_str(), nullptr, error, sizeof(error)));
        if (!model_) throw std::runtime_error(error);
        data_.reset(mj_makeData(model_.get()));
        if (!data_) throw std::runtime_error("Cannot allocate kinematic state");
        for (size_t side = 0; side < arms_.size(); ++side) {
            const std::string name = side == 0 ? "right" : "left";
            auto &arm = arms_[side];
            for (int j = 1; j <= 8; ++j) {
                const auto joint_name = "openarm_" + name + (j == 8 ? "_finger_joint1" : "_joint" + std::to_string(j));
                const int joint = mj_name2id(model_.get(), mjOBJ_JOINT, joint_name.c_str());
                const int motor = mj_name2id(model_.get(), mjOBJ_ACTUATOR, (joint_name + "_position").c_str());
                if (joint < 0 || motor < 0) throw std::runtime_error("Missing joint/actuator " + joint_name);
                arm.axes.push_back({joint_name, joint, model_->jnt_qposadr[joint], motor});
            }
            arm.teleop = std::make_unique<OpenArmTeleop>(*this, model_.get(), data_.get(), name);
            arm.teleop->inhibit(true);
            // Fixed preview namespace: no parameter can redirect these to controllers.
            arm.preview = create_publisher<trajectory_msgs::msg::JointTrajectory>(
                "/vive_vr/hardware_preview/" + name + "/joint_trajectory", 1);
        }
        if (live_output_) {
            if (!get_parameter("openarm.require_alignment").as_bool() ||
                get_parameter("openarm.joint_speed_rad_s").as_double() > 0.6 ||
                get_parameter("openarm.max_linear_speed_m_s").as_double() > 0.20 ||
                get_parameter("openarm.max_angular_speed_rad_s").as_double() > 1.0)
                throw std::runtime_error("Live control requires alignment and bounded speed limits");
            const double finger_speed=get_parameter("openarm.finger_speed_m_s").as_double();
            if (!std::isfinite(finger_speed) || finger_speed<=0 || finger_speed>0.015)
                throw std::runtime_error("Gripper speed must be in (0, 0.015] m/s");
            for (size_t side=0; side<2; ++side) {
                const std::string name = side==0 ? "right" : "left";
                auto &arm = arms_[side];
                arm.command = create_publisher<trajectory_msgs::msg::JointTrajectory>(
                    "/" + name + "_joint_trajectory_controller/joint_trajectory", rclcpp::QoS(1));
                if (grippers_) {
                    arm.gripper = create_publisher<trajectory_msgs::msg::JointTrajectory>(
                        "/" + name + "_gripper_controller/joint_trajectory", rclcpp::QoS(1));
                    const auto ns=get_parameter("out_ns").as_string()+"/"+name;
                    arm.joy=create_subscription<sensor_msgs::msg::Joy>(ns+"/joy",rclcpp::SensorDataQoS().keep_last(1),
                        [this,side](sensor_msgs::msg::Joy::SharedPtr msg) { gripper_buttons(arms_[side],*msg); });
                    arm.hand_pose=create_subscription<geometry_msgs::msg::PoseStamped>(ns+"/pose",rclcpp::SensorDataQoS().keep_last(1),
                        [this,side](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                            auto &a=arms_[side];
                            const auto &p=msg->pose.position; const auto &q=msg->pose.orientation;
                            const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
                            if (!fresh_input(msg->header.stamp,a.hand_stamp) ||
                                !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
                                !std::isfinite(norm) || norm<0.81 || norm>1.21) {
                                a.hand_at={}; a.gripper_ready=false; a.gripper_direction=0; return;
                            }
                            a.hand_at=Clock::now();
                        });
                }
                arm.health = create_subscription<std_msgs::msg::Bool>(
                    "/vive_vr/hardware/" + name + "/healthy", rclcpp::QoS(1).transient_local(),
                    [this,side](std_msgs::msg::Bool::SharedPtr msg) {
                        arms_[side].healthy=msg->data;
                        arms_[side].healthy_at=Clock::now();
                        if (!msg->data) disarm("hardware fault");
                    });
            }
            controllers_ = create_client<controller_manager_msgs::srv::ListControllers>("/controller_manager/list_controllers");
            controller_timer_ = create_wall_timer(std::chrono::milliseconds(250), [this] { check_controllers(); });
        }
        clutch_reset_=create_publisher<std_msgs::msg::Empty>(get_parameter("out_ns").as_string()+"/teleop/reset",10);
        arm_service_ = create_service<std_srvs::srv::SetBool>("~/enable",
            [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
                   std_srvs::srv::SetBool::Response::SharedPtr response) {
                if (!request->data) {
                    disarm("operator disabled"); response->success=true; response->message="VR commands disabled; last targets held";
                } else if (homing_) {
                    response->success=false; response->message="Return to zero is active; cancel it before enabling VR";
                } else if (grippers_ && std::any_of(arms_.begin(),arms_.end(),[](const Arm &arm) {
                    return arm.joy_at==Clock::time_point{} ||
                        std::chrono::duration<double>(Clock::now()-arm.joy_at).count()>0.25;
                })) {
                    response->success=false;
                    response->message="Lock the world in the updated headset app and supply fresh controller input before enabling";
                } else if (const auto reason = arming_blockers(); !reason.empty()) {
                    disarm("arming conditions failed");
                    response->success=false; response->message="Cannot arm: " + reason;
                } else {
                    // Re-arming cannot revive a held trigger or retain an old stream.
                    for (auto &arm:arms_) stop_gripper(arm,true);
                    armed_=true;
                    for (auto &arm : arms_) { arm.teleop->reset(); arm.teleop->inhibit(false); arm.sent=false; }
                    clutch_reset_->publish(std_msgs::msg::Empty{});
                    response->success=true; response->message="Armed with bounded speeds; release controller buttons before engaging";
                }
            });
        home_status_ = create_publisher<std_msgs::msg::String>("~/zero_status", rclcpp::QoS(1).transient_local());
        home_heartbeat_ = create_subscription<std_msgs::msg::Empty>("~/zero_keepalive", 1,
            [this](std_msgs::msg::Empty::SharedPtr) { if (homing_) home_lease_=Clock::now(); });
        auto home_callback = [this](bool close_grippers) { return [this,close_grippers](const std_srvs::srv::SetBool::Request::SharedPtr request,
                   std_srvs::srv::SetBool::Response::SharedPtr response) {
                if (!request->data) {
                    disarm("return to zero cancelled");
                    response->success=true; response->message="Stopped; VR remains disarmed";
                    return;
                }
                if (homing_) { response->message="Return to zero already active"; return; }
                if (const auto why=arming_blockers(); !why.empty()) {
                    response->message="Cannot return to zero: " + why; return;
                }
                if (close_grippers && !grippers_) { response->message="Gripper control is disabled"; return; }
                double largest=0, finger_largest=0;
                for (size_t side=0;side<2;++side) for (size_t i=0;i<7;++i) {
                    const auto &axis=arms_[side].axes[i];
                    const double q=data_->qpos[axis.qpos];
                    const double lo=model_->jnt_range[axis.joint*2], hi=model_->jnt_range[axis.joint*2+1];
                    const double margin=i==3 ? 0.05 : 0.0;
                    if (lo>0 || hi<0 || q<lo-margin || q>hi) {
                        response->message="Zero or measured position outside joint limits: " + axis.name;
                        return;
                    }
                    home_start_[side][i]=q;
                    largest=std::max(largest,std::abs(q));
                }
                for (size_t side=0;side<2;++side) {
                    const auto &axis=arms_[side].axes[7];
                    home_finger_start_[side]=data_->qpos[axis.qpos];
                    if (close_grippers && (model_->jnt_range[axis.joint*2]>0 || model_->jnt_range[axis.joint*2+1]<0)) {
                        response->message="Closed gripper target outside model limits"; return;
                    }
                    finger_largest=std::max(finger_largest,std::abs(home_finger_start_[side]));
                }
                disarm("return to zero requested");
                clutch_reset_->publish(std_msgs::msg::Empty{});
                home_close_grippers_=close_grippers;
                for (auto &arm:arms_) {
                    for (size_t i=0;i<8;++i) arm.last[i]=data_->qpos[arm.axes[i].qpos];
                    arm.sent=false;
                }
                // Quintic smoothstep has maximum derivative 1.875. Peak joint
                // speed is <=0.05 rad/s, independent of VR speed settings.
                home_duration_=std::max(2.0,1.875*largest/0.05);
                if (close_grippers) home_duration_=std::max(home_duration_,1.875*finger_largest/0.01);
                home_elapsed_=0; home_settled_=0;
                home_lease_=home_began_=Clock::now(); homing_=true;
                set_home_status("running");
                response->success=true;
                response->message=close_grippers ? "Returning arms to zero and closing grippers; VR locked out" :
                    "Returning both arms to existing joint zero; grippers unchanged; VR locked out";
            }; };
        home_service_ = create_service<std_srvs::srv::SetBool>("~/return_to_zero",home_callback(false));
        home_close_service_ = create_service<std_srvs::srv::SetBool>("~/return_to_zero_and_close",home_callback(true));
        set_home_status("idle");
        SceneConf conf;
        conf.manifest_path = declare_parameter<std::string>("manifest", "");
        conf.scene_url = declare_parameter<std::string>("scene_url", "");
        conf.topic_ns = get_parameter("out_ns").as_string();
        conf.frame_id = "world";
        conf.rate_hz = 50.0;
        conf.operator_view = declare_parameter<bool>("operator_view", false);
        conf.view_yaw_deg = declare_parameter<double>("view_yaw_deg", 0.0);
        scene_ = std::make_unique<BodyPosePublisher>(*this, model_.get(), conf);
        feedback_ = create_subscription<sensor_msgs::msg::JointState>(
            declare_parameter<std::string>("joint_states_topic", "/joint_states"),
            rclcpp::SensorDataQoS().keep_last(1),
            [this](sensor_msgs::msg::JointState::SharedPtr msg) { feedback(*msg); });
        timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] { tick(); });
        RCLCPP_WARN(get_logger(), "%s", live_output_ ?
            "LIVE OUTPUT AVAILABLE, DISARMED: call ~/enable after checking workspace and E-stop" :
            "PREVIEW ONLY: targets never go to hardware controllers; scene follows measured joints");
    }
private:
    std::string arming_blockers() const {
        const auto time = Clock::now();
        std::string reasons;
        auto add = [&](const std::string &why) {
            if (!reasons.empty()) reasons += "; ";
            reasons += why;
        };
        if (!live_output_) add("preview-only mode");
        if (!live_ || std::chrono::duration<double>(time-received_).count()>0.15)
            add("joint feedback missing, invalid or older than 150 ms");
        for (size_t i=0;i<arms_.size();++i) {
            const auto &arm=arms_[i];
            const std::string side=i==0 ? "right" : "left";
            if (arm.healthy_at == Clock::time_point{}) add(side+" driver health not received");
            else if (!arm.healthy) add(side+" driver reports a fault; inspect driver log and restart after fixing it");
            else if (std::chrono::duration<double>(time-arm.healthy_at).count()>0.15)
                add(side+" driver health older than 150 ms");
        }
        if (!controllers_ok_) add("trajectory controllers inactive or not yet confirmed");
        else if (std::chrono::duration<double>(time-controllers_at_).count()>0.75)
            add("controller-state reply older than 750 ms");
        return reasons;
    }
    bool healthy() const {
        const auto now=Clock::now();
        if (!controllers_ok_ || std::chrono::duration<double>(now-controllers_at_).count()>0.75) return false;
        for (const auto &arm : arms_)
            if (!arm.healthy || std::chrono::duration<double>(now-arm.healthy_at).count()>0.15) return false;
        return true;
    }
    void check_controllers() {
        if (!controllers_->service_is_ready()) { controllers_ok_=false; return; }
        // At most one outstanding request; remove a timed-out request to bound memory.
        if (request_pending_) {
            if (Clock::now()-request_at_ < std::chrono::milliseconds(500)) return;
            controllers_->prune_pending_requests(); request_pending_=false; controllers_ok_=false;
        }
        request_pending_=true; request_at_=Clock::now();
        controllers_->async_send_request(std::make_shared<controller_manager_msgs::srv::ListControllers::Request>(),
            [this](rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture future) {
                request_pending_=false;
                const auto response=future.get();
                bool ok=true;
                for (const auto *side : {"right", "left"}) {
                    for (const auto *kind : {"joint_trajectory_controller", "gripper_controller"}) {
                        const std::string name=std::string(side)+"_"+kind;
                        ok &= std::any_of(response->controller.begin(),response->controller.end(),
                            [&](const auto &c) { return c.name==name && c.state=="active"; });
                    }
                }
                controllers_ok_=ok; controllers_at_=Clock::now();
                if (!ok) disarm("controller inactive");
            });
    }
    bool fresh_input(const builtin_interfaces::msg::Time &stamp, int64_t &previous) const {
        const rclcpp::Time time(stamp);
        const double age=(now()-time).seconds();
        if (time.nanoseconds()<=previous || age < -0.1 || age>0.25) return false;
        previous=time.nanoseconds(); return true;
    }
    bool hand_fresh(const Arm &arm) const {
        return arm.hand_at!=Clock::time_point{} &&
               std::chrono::duration<double>(Clock::now()-arm.hand_at).count()<=0.25;
    }
    void gripper_buttons(Arm &arm, const sensor_msgs::msg::Joy &msg) {
        if (!fresh_input(msg.header.stamp,arm.joy_stamp) || msg.buttons.size()<7 ||
            msg.axes.size()<2 || !std::isfinite(msg.axes[1]) || std::abs(msg.axes[1])>1.0 ||
            msg.buttons[6]!=1) {
            arm.joy_at={}; arm.gripper_ready=false; arm.gripper_direction=0;
            if (armed_) disarm("world unlocked or invalid stick input; lock world before enabling");
            return;
        }
        arm.joy_at=Clock::now();
        if (!armed_ || homing_ || !live_ || !hand_fresh(arm)) {
            arm.gripper_ready=false; arm.gripper_direction=0; return;
        }
        const double stick=msg.axes[1];
        if (std::abs(stick)<=0.2) {
            arm.gripper_direction=0;
            arm.gripper_ready=true;
            return;
        }
        if (arm.gripper_ready) {
            if (stick>=0.5) arm.gripper_direction=1;
            else if (stick<=-0.5) arm.gripper_direction=-1;
        }
    }

    void publish_gripper(Arm &arm) {
        trajectory_msgs::msg::JointTrajectory msg;
        msg.joint_names={arm.axes[7].name};
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions={arm.gripper_last}; point.time_from_start.nanosec=40000000;
        msg.points.push_back(point); arm.gripper->publish(msg);
    }
    void stop_gripper(Arm &arm, bool require_release) {
        if (arm.gripper_sent) { publish_gripper(arm); arm.gripper_sent=false; }
        arm.gripper_velocity=0;
        if (require_release) { arm.gripper_ready=false; arm.gripper_direction=0; }
    }
    void tick_gripper(Arm &arm, double dt) {
        if (!arm.gripper) return;
        if (!armed_ || homing_ || !live_ || !hand_fresh(arm) ||
            arm.joy_at==Clock::time_point{} ||
            std::chrono::duration<double>(Clock::now()-arm.joy_at).count()>0.25) {
            stop_gripper(arm,true); return;
        }
        if (!arm.gripper_direction) { stop_gripper(arm,false); return; }
        const auto &axis=arm.axes[7];
        const double measured=data_->qpos[axis.qpos];
        const double previous=arm.gripper_sent ? arm.gripper_last : measured;
        const double lower=model_->jnt_range[axis.joint*2], upper=model_->jnt_range[axis.joint*2+1];
        const double goal=arm.gripper_direction>0 ? upper : lower;
        double next=smooth_bounded_step(goal,previous,arm.gripper_velocity,
            get_parameter("openarm.finger_speed_m_s").as_double(),0.075,std::clamp(dt,0.0,0.02));
        // Fingers stalled on an object hold a bounded squeeze instead of winding up into a disarm.
        constexpr double kMaxGripperLead=0.004;
        if (std::abs(next-measured)>kMaxGripperLead) {
            next=std::clamp(next,measured-kMaxGripperLead,measured+kMaxGripperLead);
            arm.gripper_velocity=0;
        }
        if (!valid_recovery_command(next,measured,previous,lower,upper,0.008)) {
            disarm("gripper following error or joint limit"); return;
        }
        arm.gripper_last=next; arm.gripper_sent=true; publish_gripper(arm);
    }
    void publish_command(Arm &arm) {
        trajectory_msgs::msg::JointTrajectory msg;
        // Zero stamp means start now, avoiding queued targets with stale start times.
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.time_from_start.nanosec=40000000;
        for (size_t i=0;i<7;++i) { msg.joint_names.push_back(arm.axes[i].name); point.positions.push_back(arm.last[i]); }
        msg.points.push_back(point); arm.command->publish(msg);
    }
    void disarm(const char *reason) {
        if ((armed_ || homing_) && clutch_reset_) clutch_reset_->publish(std_msgs::msg::Empty{});
        for (auto &arm:arms_) stop_gripper(arm,true);
        if (homing_) {
            for (auto &arm:arms_) if (arm.sent) publish_command(arm);
            homing_=false;
            set_home_status(std::string("stopped: ")+reason);
        }
        if (armed_) {
            for (auto &arm : arms_) if (arm.sent) publish_command(arm);
            RCLCPP_WARN(get_logger(), "VR disarmed: %s; explicit enable and a new grip cycle required",reason);
        }
        armed_=false;
        if (live_output_) for (auto &arm : arms_) arm.teleop->inhibit(true);
    }
    void invalidate() {
        disarm("joint feedback invalid/stale or timer stalled");
        live_ = false;
        for (auto &arm : arms_) arm.teleop->inhibit(true);
    }
    void feedback(const sensor_msgs::msg::JointState &msg) {
        const rclcpp::Time stamp(msg.header.stamp);
        const double stamp_age = (now() - stamp).seconds();
        if (msg.name.size() != msg.position.size() || stamp.nanoseconds() <= last_stamp_ ||
            stamp_age < -0.1 || stamp_age > 0.15) { invalidate(); return; }
        std::unordered_map<std::string, double> values;
        for (size_t i = 0; i < msg.name.size(); ++i) {
            if (!std::isfinite(msg.position[i]) || !values.emplace(msg.name[i], msg.position[i]).second) {
                invalidate(); return;
            }
        }
        // Validate the whole sample before changing any model state.
        for (const auto &arm : arms_) for (const auto &axis : arm.axes) {
            const auto it = values.find(axis.name);
            const double margin = axis.name.find("finger") == std::string::npos ? 0.05 : 0.002;
            if (it == values.end() || it->second < model_->jnt_range[axis.joint*2] - margin ||
                it->second > model_->jnt_range[axis.joint*2+1] + margin) { invalidate(); return; }
        }
        for (auto &arm : arms_) for (const auto &axis : arm.axes) {
            data_->qpos[axis.qpos] = values.at(axis.name);
            if (axis.name.find("finger_joint1") != std::string::npos) {
                auto mimic = axis.name;
                mimic.back() = '2';
                const int joint = mj_name2id(model_.get(), mjOBJ_JOINT, mimic.c_str());
                if (joint >= 0) data_->qpos[model_->jnt_qposadr[joint]] = values.at(axis.name);
            }
        }
        mj_forward(model_.get(), data_.get()); // Kinematics only: never mj_step.
        if (!live_) for (auto &arm : arms_) {
            arm.teleop->reset();
            arm.teleop->inhibit(live_output_ && !armed_);
        }
        live_ = true;
        last_stamp_ = stamp.nanoseconds();
        received_ = Clock::now();
    }
    void set_home_status(const std::string &state) {
        std_msgs::msg::String msg; msg.data=state; home_status_->publish(msg);
    }
    void tick_home(double dt) {
        const auto time=Clock::now();
        if (!live_ || !healthy() || std::chrono::duration<double>(time-home_lease_).count()>0.5) {
            disarm("return to zero lost feedback, controller health or script keepalive"); return;
        }
        if (std::chrono::duration<double>(time-home_began_).count()>home_duration_+15.0) {
            disarm("return to zero did not settle before deadline"); return;
        }
        // Never catch up after scheduler delays.
        home_elapsed_+=std::clamp(dt,0.0,0.02);
        const double u=std::clamp(home_elapsed_/home_duration_,0.0,1.0);
        const double blend=u*u*u*(10.0+u*(-15.0+6.0*u));
        std::array<std::array<double,8>,2> next{};
        bool settled=true;
        for (size_t side=0;side<2;++side) for (size_t i=0;i<7;++i) {
            const auto &axis=arms_[side].axes[i];
            const double q=data_->qpos[axis.qpos];
            const double desired=home_start_[side][i]*(1.0-blend);
            // Permit only the already accepted elbow offset on a monotonic
            // path back to zero; do not expand the target range for VR motion.
            const double lo=std::min(model_->jnt_range[axis.joint*2],arms_[side].last[i]);
            if (!valid_command(desired,q,lo,model_->jnt_range[axis.joint*2+1],0.10)) {
                disarm("return to zero following error or joint limit"); return;
            }
            next[side][i]=desired;
            settled &= std::abs(q)<=0.017453292519943295; // one degree
        }
        if (home_close_grippers_) for (size_t side=0;side<2;++side) {
            auto &arm=arms_[side]; const auto &axis=arm.axes[7];
            const double measured=data_->qpos[axis.qpos];
            const double previous=arm.gripper_sent ? arm.gripper_last : home_finger_start_[side];
            const double desired=home_finger_start_[side]*(1.0-blend);
            if (!valid_recovery_command(desired,measured,previous,model_->jnt_range[axis.joint*2],
                                        model_->jnt_range[axis.joint*2+1],0.008)) {
                disarm("return to zero gripper following error or joint limit"); return;
            }
            arm.gripper_last=desired; arm.gripper_sent=true;
            settled &= std::abs(measured)<=0.001; // 1 mm, report actual feedback in CLI
        }
        for (size_t side=0;side<2;++side) {
            if (home_close_grippers_) publish_gripper(arms_[side]);
            for (size_t i=0;i<7;++i) arms_[side].last[i]=next[side][i];
            arms_[side].sent=true; publish_command(arms_[side]);
        }
        home_settled_=(u>=1.0 && settled) ? home_settled_+std::clamp(dt,0.0,0.02) : 0;
        if (home_settled_>=0.5) {
            homing_=false;
            for (auto &arm:arms_) { arm.sent=false; arm.gripper_sent=false; }
            set_home_status("complete");
            RCLCPP_INFO(get_logger(),"Return to zero complete within 1 degree; VR remains disarmed");
        }
    }
    void tick() {
        const auto time = Clock::now();
        if (std::chrono::duration<double>(time - received_).count() > 0.15) invalidate();
        const double dt = std::chrono::duration<double>(time - last_tick_).count();
        last_tick_ = time;
        if (dt > 0.1) invalidate();
        if (live_output_ && armed_ && !healthy()) disarm("hardware health/controller heartbeat missing");
        if (homing_) tick_home(dt);
        for (auto &arm : arms_) {
            if (!homing_) tick_gripper(arm,dt);
            arm.teleop->tick(std::clamp(dt, 0.0, 0.02));
            if (!live_ || !arm.teleop->commanding()) {
                // A released or timed-out grip replaces the short trajectory with a
                // hold at the last bounded target, never a stale measured position.
                if (live_output_ && armed_ && arm.sent) { publish_command(arm); arm.sent=false; }
                continue;
            }
            trajectory_msgs::msg::JointTrajectory target;
            target.header.stamp = now();
            trajectory_msgs::msg::JointTrajectoryPoint point;
            for (const auto &axis : arm.axes) {
                target.joint_names.push_back(axis.name);
                point.positions.push_back(data_->ctrl[axis.motor]);
            }
            point.time_from_start.nanosec = 100000000;
            target.points.push_back(point);
            arm.preview->publish(target);
            if (live_output_ && armed_) {
                std::array<double,8> next{};
                bool valid=true;
                for (size_t i=0;i<8;++i) {
                    const auto &axis=arm.axes[i];
                    const double measured=data_->qpos[axis.qpos];
                    // Physical grippers have an independent button/heartbeat path.
                    // Ignore the shared simulation teleop's analog finger target.
                    if (i==7) { next[i]=measured; continue; }
                    const double desired=data_->ctrl[axis.motor];
                    const double lower=std::min(model_->jnt_range[axis.joint*2],measured);
                    const double upper=std::max(model_->jnt_range[axis.joint*2+1],measured);
                    if (!valid_command(desired, measured, lower, upper, i==7 ? 0.008 : 0.20)) {
                        RCLCPP_WARN(get_logger(), "%s command rejected: target=%.6f measured=%.6f error=%.6f bounds=[%.6f, %.6f]",
                            axis.name.c_str(), desired, measured, desired-measured, lower, upper);
                        valid=false;
                    }
                    next[i]=limited_step(desired, arm.sent ? arm.last[i] : measured,
                                         get_parameter("openarm.joint_speed_rad_s").as_double(), std::min(dt,0.02));
                }
                if (!valid) { disarm("command limits or following error"); continue; }
                arm.last=next; arm.sent=true; publish_command(arm);
            }
        }
        if (live_) {
            data_->time += std::max(0.0, dt);
            scene_->publish(data_.get());
        }
    }
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model_{nullptr, mj_deleteModel};
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data_{nullptr, mj_deleteData};
    std::array<Arm, 2> arms_;
    std::unique_ptr<BodyPosePublisher> scene_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedback_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool homing_=false;
    double home_duration_=0, home_elapsed_=0, home_settled_=0;
    Clock::time_point home_lease_{}, home_began_{};
    std::array<std::array<double,7>,2> home_start_{};
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr home_status_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr home_heartbeat_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr home_service_;
    bool live_ = false, live_output_=false, grippers_=false, armed_=false;
    bool home_close_grippers_=false;
    std::array<double,2> home_finger_start_{};
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr home_close_service_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr clutch_reset_;
    bool controllers_ok_=false, request_pending_=false;
    Clock::time_point controllers_at_{}, request_at_{};
    rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr controllers_;
    rclcpp::TimerBase::SharedPtr controller_timer_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arm_service_;
    int64_t last_stamp_ = 0;
    Clock::time_point received_{}, last_tick_ = Clock::now();
};
}
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try { rclcpp::spin(std::make_shared<vive_vr_ros2::HardwarePreview>()); }
    catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("hardware_preview"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
