#include "vive_vr_ros2/openarm_teleop.hpp"
#include <thread>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>

void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

/* The speed bound the test pins, so the checks below do not track the shipped default. */
constexpr double kJointSpeed = 0.35;

int main(int argc, char** argv) {
    const std::string path =
      argc >= 2 ? argv[1] : std::string(TEST_SOURCE_DIR) + "/openarm_test_arm.xml";
    char error[2048]={};
    auto* m=mj_loadXML(path.c_str(),nullptr,error,sizeof(error));
    if(!m){std::cerr<<error;return 1;}
    auto*d=mj_makeData(m);mj_resetDataKeyframe(m,d,0);mj_forward(m,d);
    rclcpp::init(argc,argv);
    int result=0;
    try {
        rclcpp::NodeOptions options;
        options.parameter_overrides({rclcpp::Parameter("openarm.require_alignment", false),
                                     rclcpp::Parameter("openarm.joint_speed_rad_s", kJointSpeed)});
        auto node=std::make_shared<rclcpp::Node>("openarm_bridge_test", options);
        node->declare_parameter<std::string>("out_ns","/openarm_test");
        vive_vr_ros2::OpenArmTeleop bridge(*node,m,d);
        auto clutch=node->create_publisher<std_msgs::msg::Bool>(
            "/openarm_test/teleop/right/clutch",rclcpp::QoS(1).transient_local());
        auto delta=node->create_publisher<geometry_msgs::msg::TransformStamped>(
            "/openarm_test/teleop/right/delta",rclcpp::SensorDataQoS());
        auto gripper=node->create_publisher<std_msgs::msg::Float32>(
            "/openarm_test/teleop/right/gripper",rclcpp::SensorDataQoS());
        std::string reported, reason;
        auto reports=node->create_subscription<vive_vr_ros2::msg::TeleopStatus>(
            "/openarm_test/teleop/right/status",10,
            [&](vive_vr_ros2::msg::TeleopStatus::SharedPtr msg){reported=msg->state;reason=msg->stop_reason;});
        auto spin=[&]{for(int i=0;i<5;++i){rclcpp::spin_some(node);std::this_thread::sleep_for(std::chrono::milliseconds(2));}};
        auto press=[&](bool value){std_msgs::msg::Bool msg;msg.data=value;clutch->publish(msg);spin();};
        auto send=[&](double z, double age=0){
            geometry_msgs::msg::TransformStamped msg;
            msg.header.stamp=node->now()-rclcpp::Duration::from_seconds(age);
            msg.header.frame_id="right_tool_ref";msg.child_frame_id="right_tool_cmd";
            msg.transform.rotation.w=1;msg.transform.translation.z=z;
            delta->publish(msg);spin();
        };
        for(int i=0;i<100 && delta->get_subscription_count()==0;++i) spin();
        check(delta->get_subscription_count()>0,"DDS discovery failed");
        std::vector<double> initial(d->ctrl,d->ctrl+m->nu);
        press(true);send(0.02);bridge.tick(1./60);
        for(int i=0;i<m->nu;++i) check(d->ctrl[i]==initial[i],"engaged without first release");
        press(false);press(true);send(0);
        int finger=mj_name2id(m,mjOBJ_ACTUATOR,"openarm_right_finger_joint1_position");
        std_msgs::msg::Float32 resting_trigger;resting_trigger.data=0;
        for(int i=0;i<5;++i){
            send(0);gripper->publish(resting_trigger);spin();bridge.tick(1./60);
        }
        check(d->ctrl[finger]==initial[finger],"side grip alone moved the gripper");
        std_msgs::msg::Float32 squeezed_trigger;squeezed_trigger.data=1;
        gripper->publish(squeezed_trigger);spin();bridge.tick(1./60);
        check(d->ctrl[finger]<initial[finger],"rear trigger did not close the gripper");
        for(int i=0;i<80;++i){
            send(-0.04);
            std_msgs::msg::Float32 g;g.data=0;gripper->publish(g);spin();
            std::vector<double> previous(d->ctrl,d->ctrl+m->nu);
            bridge.tick(1./60);
            for(int a=0;a<m->nu;++a){
                std::string name=mj_id2name(m,mjOBJ_ACTUATOR,a);
                if(name.find("openarm_right_joint")==0)
                    check(std::abs(d->ctrl[a]-previous[a])<=kJointSpeed/60+1e-9,"target speed exceeded");
            }
            for(int step=0;step<8;++step)mj_step(m,d);
        }
        double moved=0;
        for(int i=0;i<m->nu;++i){
            std::string name=mj_id2name(m,mjOBJ_ACTUATOR,i);
            if(name.find("openarm_right_joint")==0) moved+=std::abs(d->ctrl[i]-initial[i]);
            if(name.find("openarm_left_")==0)check(d->ctrl[i]==initial[i],"left target changed");
        }
        check(moved>0.01,"right arm did not follow delta");
        check(d->ctrl[finger]>0.025,"gripper did not open");
        press(false);
        std::vector<double> held(d->ctrl,d->ctrl+m->nu);
        send(0.03);bridge.tick(1./60);
        for(int i=0;i<m->nu;++i)check(d->ctrl[i]==held[i],"release did not hold");
        press(true);send(0);bridge.tick(1./60);
        for(int i=0;i<m->nu;++i)check(std::abs(d->ctrl[i]-held[i])<0.006,"reclutch jumped");
        std::this_thread::sleep_for(std::chrono::milliseconds(300));bridge.tick(1./60);
        held.assign(d->ctrl,d->ctrl+m->nu);send(-0.02);bridge.tick(1./60);
        for(int i=0;i<m->nu;++i)check(d->ctrl[i]==held[i],"timeout reengaged without release");
        /* Regression: freshness is arrival, so a stamp may read however it likes. */
        press(false);press(true);send(0);bridge.tick(1./60);spin();
        check(reported=="engaged"||reported=="unreachable","fresh delta was not engaged");
        press(false);press(true);send(0);bridge.tick(1./60);spin();
        send(-0.02,5);bridge.tick(1./60);spin();
        check(reported=="engaged"||reported=="unreachable","delta dropped for an old stamp");
        send(-0.03,9);bridge.tick(1./60);spin();
        send(-0.04,3);bridge.tick(1./60);spin();
        check(reported=="engaged"||reported=="unreachable","delta dropped for a backwards stamp");
        send(std::numeric_limits<double>::quiet_NaN());bridge.tick(1./60);spin();
        check(reason=="invalid_target","invalid target reason missing");
        held.assign(d->ctrl,d->ctrl+m->nu);send(-0.02);bridge.tick(1./60);
        for(int i=0;i<m->nu;++i)check(d->ctrl[i]==held[i],"invalid delta failed to disengage");
        press(false);press(true);send(1.0);bridge.tick(1./60);spin();
        check(reason=="translation_limit","translation stop reason missing");
        held.assign(d->ctrl,d->ctrl+m->nu);send(-0.02);bridge.tick(1./60);
        for(int i=0;i<m->nu;++i)check(d->ctrl[i]==held[i],"oversized target failed to disengage");
        bridge.reset();press(true);send(-0.02);bridge.tick(1./60);
        for(int i=0;i<m->nu;++i)check(d->ctrl[i]==held[i],"reset reengaged without release");
        // Production alignment gate: a green state requires a fresh, matching orientation.
        rclcpp::NodeOptions aligned_options;
        aligned_options.parameter_overrides({rclcpp::Parameter("openarm.alignment_configured", true)});
        auto aligned_node=std::make_shared<rclcpp::Node>("alignment_gate_test", aligned_options);
        aligned_node->declare_parameter<std::string>("out_ns","/alignment_test");
        vive_vr_ros2::OpenArmTeleop gated(*aligned_node,m,d);
        auto cp=aligned_node->create_publisher<geometry_msgs::msg::PoseStamped>("/alignment_test/right/pose",rclcpp::SensorDataQoS());
        auto gp=aligned_node->create_publisher<std_msgs::msg::Bool>("/alignment_test/teleop/right/clutch",rclcpp::QoS(1).transient_local());
        auto dp=aligned_node->create_publisher<geometry_msgs::msg::TransformStamped>("/alignment_test/teleop/right/delta",rclcpp::SensorDataQoS());
        geometry_msgs::msg::PoseStamped alignment_pose;
        bool have_alignment_pose=false;
        auto guide_sub=aligned_node->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/alignment_test/teleop/right/alignment_pose",rclcpp::SensorDataQoS(),
            [&](geometry_msgs::msg::PoseStamped::SharedPtr msg){alignment_pose=*msg;have_alignment_pose=true;});
        std::string status;
        vive_vr_ros2::msg::TeleopStatus report;
        auto sub=aligned_node->create_subscription<vive_vr_ros2::msg::TeleopStatus>("/alignment_test/teleop/right/status",10,
            [&](vive_vr_ros2::msg::TeleopStatus::SharedPtr msg){status=msg->state;report=*msg;});
        auto pump=[&]{for(int i=0;i<8;++i){rclcpp::spin_some(aligned_node);std::this_thread::sleep_for(std::chrono::milliseconds(2));}};
        for(int i=0;i<100 && (cp->get_subscription_count()==0 || gp->get_subscription_count()==0 || dp->get_subscription_count()==0);++i)pump();
        auto button=[&](bool down){std_msgs::msg::Bool msg;msg.data=down;gp->publish(msg);pump();};
        auto state=[&]{gated.tick(1./60);pump();return status;};
        const int tcp=mj_name2id(m,mjOBJ_BODY,"openarm_right_hand_tcp");
        auto pose=[&](bool turned, bool displaced){
            mj_kinematics(m,d);
            geometry_msgs::msg::PoseStamped msg;msg.header.frame_id="world";msg.header.stamp=aligned_node->now();
            const auto*q=d->xquat+4*tcp;
            mjtNum wrong[4], turn[4]={0,1,0,0};mju_mulQuat(wrong,q,turn);
            if(turned)q=wrong;
            msg.pose.orientation.w=q[0];msg.pose.orientation.x=q[1];msg.pose.orientation.y=q[2];msg.pose.orientation.z=q[3];
            const auto*p=d->xpos+3*tcp;
            msg.pose.position.x=p[0]+(displaced?2.0:0.0);msg.pose.position.y=p[1];msg.pose.position.z=p[2];
            cp->publish(msg);pump();
        };
        button(false);check(state()=="tracking_lost","missing tracking displayed ready");
        pose(true,false);check(state()=="align_pose","mismatched orientation displayed ready");
        button(true);check(state()=="release_grip","misaligned grip engaged");
        // Relative engagement allows a comfortable hand position far from the robot.
        button(false);pose(false,true);check(state()=="ready","relative controller not ready at two metres");
        check(report.position_error_m>0.4,"status did not report the position error");
        check(report.position_tolerance_m==0,"relative mode advertised a proximity gate");
        button(true);check(state()=="waiting_delta","relative grip did not engage");
        button(false);pose(false,false);check(state()=="ready","aligned controller not ready");
        check(report.ready,"ready state did not set the ready flag");
        check(have_alignment_pose,"orientation guide not published");
        const auto &aq=alignment_pose.pose.orientation;
        const mjtNum target_q[4]={aq.w,aq.x,aq.y,aq.z};
        check(std::abs(mju_dot(target_q,d->xquat+4*tcp,4))>0.999,"guide orientation incorrect for identity pairing");
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        check(state()=="tracking_lost","stale controller displayed ready");
        pose(false,false);button(true);check(state()=="waiting_delta","aligned grip did not engage");
        auto rotation=[&](double angle){
            geometry_msgs::msg::TransformStamped msg;msg.header.stamp=aligned_node->now();
            msg.header.frame_id="right_tool_ref";msg.child_frame_id="right_tool_cmd";
            msg.transform.rotation.w=std::cos(angle/2);msg.transform.rotation.y=std::sin(angle/2);
            dp->publish(msg);pump();
        };
        rotation(0);state();rotation(1.57079632679);
        // Slewing towards a legal target is engaged, not unreachable: the gap is still closing.
        check(state()=="engaged","90 degree target reported as unreachable while slewing");
        rotation(1.9);check(state()=="release_grip","excessive rotation accepted");
        check(report.stop_reason=="rotation_limit","rotation stop reason missing");
        std::cout<<"PASS: alignment readiness, grip gating, stale tracking, 90-degree rotation and rotation bound\n";
        std::cout<<"PASS: release gating, delta motion, gripper, left hold, release, reclutch, timeout, stale and NaN rejection, reset\n";
    }catch(const std::exception&e){std::cerr<<e.what()<<'\n';result=1;}
    rclcpp::shutdown();mj_deleteData(d);mj_deleteModel(m);return result;
}
