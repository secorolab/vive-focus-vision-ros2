/* SPDX-License-Identifier: MIT */
// V1 CAN configuration and gripper conversion follow Enactic's OpenArm driver.
// Independent guarded implementation: no homing and position commands only.
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <thread>
#include <vector>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <openarm/can/socket/openarm.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include "vive_vr_ros2/hardware_guard.hpp"

namespace vive_vr_ros2 {
class GuardedOpenArm : public hardware_interface::SystemInterface {
    using Clock = std::chrono::steady_clock;
    using CallbackReturn = hardware_interface::CallbackReturn;
    using Return = hardware_interface::return_type;
public:
    ~GuardedOpenArm() override {
        stop_health_.store(true);
        if (health_thread_.joinable()) health_thread_.join();
    }
    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override {
        if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;
        try {
            const auto prefix = info.hardware_parameters.at("arm_prefix");
            if (prefix != "right_" && prefix != "left_") throw std::runtime_error("Invalid arm prefix");
            if (info.joints.size() != 8) throw std::runtime_error("V1 requires seven arm joints and one gripper");
            for (size_t i = 0; i < 8; ++i) {
                const auto expected = "openarm_" + prefix + (i == 7 ? "finger_joint1" : "joint" + std::to_string(i+1));
                if (info.joints[i].name != expected || info.joints[i].command_interfaces.size() != 1 ||
                    info.joints[i].command_interfaces[0].name != "position")
                    throw std::runtime_error("Unexpected joint order or command interface: " + info.joints[i].name);
                names_[i] = expected;
                lower_[i] = std::stod(info.hardware_parameters.at("lower" + std::to_string(i)));
                upper_[i] = std::stod(info.hardware_parameters.at("upper" + std::to_string(i)));
                if (!std::isfinite(lower_[i]) || !std::isfinite(upper_[i]) || lower_[i] >= upper_[i])
                    throw std::runtime_error("Invalid joint limits");
                if (i < 7) {
                    kp_[i] = std::stod(info.hardware_parameters.at("kp" + std::to_string(i+1)));
                    kd_[i] = std::stod(info.hardware_parameters.at("kd" + std::to_string(i+1)));
                    if (!std::isfinite(kp_[i]) || kp_[i] <= 0 || !std::isfinite(kd_[i]) || kd_[i] <= 0)
                        throw std::runtime_error("Invalid gains");
                }
            }
            node_ = std::make_shared<rclcpp::Node>("openarm_" + prefix + "guard");
            health_ = node_->create_publisher<std_msgs::msg::Bool>(
                "/vive_vr/hardware/" + prefix.substr(0, prefix.size()-1) + "/healthy",
                rclcpp::QoS(1).transient_local());
            arm_ = std::make_unique<openarm::can::socket::OpenArm>(info.hardware_parameters.at("can_interface"), true);
            using M = openarm::damiao_motor::MotorType;
            arm_->init_arm_motors({M::DM8009,M::DM8009,M::DM4340,M::DM4340,M::DM4310,M::DM4310,M::DM4310},
                                 {1,2,3,4,5,6,7}, {17,18,19,20,21,22,23});
            arm_->init_gripper_motor(M::DM4310,8,24);
            arm_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
            params_.resize(7);
            publish_health(false);
            // DDS reliable publication may block on discovery/backpressure. Never
            // run it in ros2_control's read/write thread.
            health_executor_.add_node(node_);
            health_timer_ = node_->create_wall_timer(std::chrono::milliseconds(20), [this] {
                const double age = std::chrono::duration<double>(Clock::now().time_since_epoch()).count()
                    - healthy_read_at_.load();
                std_msgs::msg::Bool msg;
                msg.data = health_value_.load() && age >= 0 && age <= 0.1;
                health_->publish(msg);
            });
            health_thread_ = std::thread([this] {
                try {
                    while (!stop_health_.load() && rclcpp::ok()) {
                        health_executor_.spin_some();
                        std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    }
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(node_->get_logger(), "Health reporting stopped: %s", e.what());
                }
            });
            return CallbackReturn::SUCCESS;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(rclcpp::get_logger("openarm_guard"), "%s", e.what());
            return CallbackReturn::ERROR;
        }
    }
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
        std::vector<hardware_interface::StateInterface> out;
        for (size_t i=0;i<8;++i) {
            out.emplace_back(names_[i], "position", &position_[i]);
            out.emplace_back(names_[i], "velocity", &velocity_[i]);
            out.emplace_back(names_[i], "effort", &effort_[i]);
        }
        return out;
    }
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
        std::vector<hardware_interface::CommandInterface> out;
        for (size_t i=0;i<8;++i) out.emplace_back(names_[i], "position", &command_[i]);
        return out;
    }
    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
        // No enable or set-zero commands. Require actual responses from every motor.
        for (int n=0;n<50;++n) {
            arm_->refresh_all(); arm_->recv_all(0);
            std::this_thread::sleep_for(std::chrono::milliseconds(4));
        }
        arm_->recv_all(0);
        if (!sample()) {
            RCLCPP_ERROR(node_->get_logger(), "Initial motor feedback validation failed: %s. Check CAN-FD interface setup, motor power and cabling before restarting.", sample_reason_.c_str());
            return CallbackReturn::ERROR;
        }
        return CallbackReturn::SUCCESS;
    }
    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
        if (fault_ || !sample()) return CallbackReturn::ERROR;
        // Capture before enable; never move to zero or silently rewrite calibration.
        sent_ = position_;
        command_ = position_; // Hold exactly at activation; return-to-zero is explicit.
        arm_->enable_all();
        active_ = true;
        try {
            // Allow the enable burst to drain before the first position batch.
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            transmit();
        } catch (const std::exception &e) { trip(e.what()); return CallbackReturn::ERROR; }
        for (int n=0;n<15;++n) {
            std::this_thread::sleep_for(std::chrono::milliseconds(4));
            arm_->recv_all(0);
        }
        if (!sample()) { trip("activation feedback failed"); return CallbackReturn::ERROR; }
        last_write_ = Clock::now();
        RCLCPP_INFO(node_->get_logger(), "Enabled at measured pose; no homing. Joint limit 0.3 rad/s, feedback timeout 100 ms");
        return CallbackReturn::SUCCESS;
    }
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
        // Ordinary operator shutdown disables torque. A latched communication fault
        // retains bounded last targets where communication still works to avoid a drop.
        if (active_ && !fault_) arm_->disable_all();
        active_ = false; publish_health(false);
        return CallbackReturn::SUCCESS;
    }
    CallbackReturn on_error(const rclcpp_lifecycle::State &) override {
        trip("controller manager hardware error");
        return CallbackReturn::SUCCESS;
    }
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override {
        if (arm_ && !fault_) arm_->disable_all();
        active_ = false; publish_health(false);
        return CallbackReturn::SUCCESS;
    }
    Return read(const rclcpp::Time &, const rclcpp::Duration &) override {
        if (fault_) return Return::ERROR;
        try {
            if (first_read_) {
                // Other components initialize sequentially; our activation sample may
                // already be old. Require new replies before the first write, rather
                // than granting a freshness exemption to cached values.
                first_read_=false;
                arm_->refresh_all();
                bool ready=false;
                for (int n=0;n<25;++n) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                    arm_->recv_all(0);
                    if (sample()) { ready=true; break; }
                }
                if (!ready) return trip("first-cycle motor refresh failed");
            }
            arm_->recv_all(0); // Non-blocking on every regular control cycle.
            // MIT position commands already solicit one state reply per motor.
            // Polling all motors again here doubles traffic and can exhaust a USB
            // adapter's short transmit queue. Drain replies without redundant polls.
            if (!sample()) return trip(sample_reason_.c_str());
            publish_health(active_);
            return Return::OK;
        } catch (const std::exception &e) { return trip(e.what()); }
    }
    Return write(const rclcpp::Time &, const rclcpp::Duration &) override {
        if (fault_ || !active_) return Return::ERROR;
        const auto now = Clock::now();
        const double dt = first_write_ ? 0.01 : std::chrono::duration<double>(now-last_write_).count();
        first_write_=false;
        last_write_ = now;
        if (dt <= 0 || dt > 0.1) return trip("control loop stalled for over 100 ms");
        for (size_t i=0;i<8;++i) {
            command_[i]=bounded_measured_hold(command_[i],position_[i],sent_[i],
                lower_[i],upper_[i],i==7 ? 0.00002 : 0.001);
            // Permit the small measured startup tolerance, but do not extend the URDF
            // envelope for newly commanded movement. Use the last SENT target,
            // never drifting feedback: an outside target can hold or move inward,
            // but cannot move farther outside or re-enter once back in range.
            if (!valid_recovery_command(command_[i], position_[i], sent_[i],
                                        lower_[i], upper_[i], i==7 ? 0.01 : 0.25)) {
                const std::string reason=names_[i]+" rejected command="+std::to_string(command_[i])+
                    " measured="+std::to_string(position_[i])+" last_sent="+std::to_string(sent_[i]);
                return trip(reason.c_str());
            }
        }
        for (size_t i=0;i<8;++i)
            sent_[i] = limited_step(command_[i], sent_[i], i==7 ? 0.005 : 0.30, std::min(dt,0.01));
        try {
            transmit();
            if (!arm_->is_bus_healthy()) return trip("CAN transmit error");
        } catch (const std::exception &e) { return trip(e.what()); }
        return Return::OK;
    }
private:
    bool sample() {
        if (!arm_->is_bus_healthy() || !arm_->is_link_running()) {
            const auto &bus=arm_->get_bus_status();
            sample_reason_="CAN bus/link failure: errno=" + std::to_string(bus.last_write_errno) +
                " no_buffer=" + std::to_string(bus.write_no_buffer.count) +
                " bus_off=" + std::to_string(bus.bus_off.count) +
                " ack_error=" + std::to_string(bus.ack_error.count);
            return false;
        }
        auto motors = arm_->get_arm().get_motors();
        const auto grip = arm_->get_gripper().get_motors();
        if (motors.size()!=7 || grip.size()!=1) return false;
        motors.push_back(grip[0]);
        std::array<double,8> p{}, v{}, e{};
        for (size_t i=0;i<8;++i) {
            const auto &stats = i==7 ? arm_->get_gripper().get_link_stats(0) : arm_->get_arm().get_link_stats(i);
            const auto &m = motors[i];
            if (!valid_motor({stats.ever_responded(), stats.malformed_frames != 0,
                              (m.get_error_code()>1 || (active_ && !m.is_enabled())), stats.since_last_response().count()/1e6,
                              m.get_position(), m.get_velocity()})) {
                sample_reason_=names_[i]+" feedback: age_us="+std::to_string(stats.since_last_response().count())+
                    " status="+std::to_string(m.get_error_code())+" malformed="+std::to_string(stats.malformed_frames);
                return false;
            }
            p[i] = i==7 ? m.get_position()*(-0.044/1.0472) : m.get_position();
            v[i] = i==7 ? m.get_velocity()*(-0.044/1.0472) : m.get_velocity();
            e[i] = i==7 ? 0.0 : m.get_torque();
            const double tolerance = i==7 ? 0.002 : 0.05;
            if (!std::isfinite(e[i]) || p[i]<lower_[i]-tolerance || p[i]>upper_[i]+tolerance) {
                sample_reason_=names_[i]+" out of range: "+std::to_string(p[i]); return false;
            }
        }
        position_=p; velocity_=v; effort_=e;
        return true;
    }
    void transmit() {
        const bool ok = paced_motor_batch(
            [] { return Clock::now(); },
            [](Clock::time_point until) { std::this_thread::sleep_until(until); },
            [this](int i) {
                if (!arm_->is_bus_healthy()) return false;
                if (i < 7) arm_->get_arm().mit_control_one(i, {kp_[i],kd_[i],sent_[i],0.0,0.0});
                else arm_->get_gripper().mit_control_one(0, {5.0,0.1,sent_[7]*(-1.0472/0.044),0.0,0.0});
                // Drain replies while spacing outgoing frames, instead of leaving
                // them queued until both arms finish their write cycles. This is
                // receive-only: no extra polling frames or command-rate increase.
                arm_->recv_all(0);
                return arm_->is_bus_healthy();
            });
        if (!ok) throw std::runtime_error("CAN batch failed or exceeded 30 ms send deadline");
    }
    Return trip(const char *reason) {
        if (!fault_) {
            fault_=true;
            publish_health(false);
            // Motors retain their last accepted bounded targets. Do not add more
            // traffic after a failed send or retry a partially transmitted batch.
            const auto &bus=arm_->get_bus_status();
            RCLCPP_ERROR(node_->get_logger(), "LATCHED FAULT: %s; CAN errno=%d no_buffer=%lu net_down=%lu other=%lu ack=%lu bus_off=%lu tx_overflow=%lu rx_overflow=%lu. Last targets retained; physical E-stop if needed; restart required",
                reason,bus.last_write_errno,bus.write_no_buffer.count,bus.write_net_down.count,
                bus.write_other.count,bus.ack_error.count,bus.bus_off.count,bus.tx_overflow.count,bus.rx_overflow.count);
            // Inspect cached link statistics only: never send recovery traffic on
            // the fault path. This distinguishes a single missing motor from a
            // pause affecting the whole arm without weakening the freshness gate.
            for (size_t i=0;i<8;++i) {
                const auto &link = i==7 ? arm_->get_gripper().get_link_stats(0)
                                       : arm_->get_arm().get_link_stats(i);
                RCLCPP_ERROR(node_->get_logger(), "Fault feedback snapshot: %s age_us=%ld sent=%lu replies=%lu rejected=%lu malformed=%lu",
                    names_[i].c_str(), static_cast<long>(link.since_last_response().count()),
                    static_cast<unsigned long>(link.commands_sent),
                    static_cast<unsigned long>(link.responses),
                    static_cast<unsigned long>(link.rejected_commands),
                    static_cast<unsigned long>(link.malformed_frames));
            }
        }
        return Return::ERROR;
    }
    void publish_health(bool ok) {
        if (ok) healthy_read_at_.store(std::chrono::duration<double>(Clock::now().time_since_epoch()).count());
        health_value_.store(ok);
    }
    std::string sample_reason_="invalid motor count";
    std::unique_ptr<openarm::can::socket::OpenArm> arm_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr health_;
    rclcpp::TimerBase::SharedPtr health_timer_;
    rclcpp::executors::SingleThreadedExecutor health_executor_;
    std::atomic<bool> health_value_{false}, stop_health_{false};
    std::atomic<double> healthy_read_at_{0.0};
    std::thread health_thread_;
    std::array<std::string,8> names_;
    std::array<double,8> position_{},velocity_{},effort_{},command_{},sent_{},lower_{},upper_{};
    std::array<double,7> kp_{},kd_{};
    std::vector<openarm::damiao_motor::MITParam> params_;
    bool fault_=false,active_=false,first_write_=true,first_read_=true;
    Clock::time_point last_write_{};
};
}
PLUGINLIB_EXPORT_CLASS(vive_vr_ros2::GuardedOpenArm, hardware_interface::SystemInterface)
