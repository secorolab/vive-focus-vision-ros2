#include "vive_vr_ros2/openarm_teleop.hpp"
#include <thread>
#include <cmath>
#include <iostream>
#include <limits>

void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

int main(int argc, char** argv) {
    if (argc < 2) return 2;
    char error[2048]={};
    auto* m=mj_loadXML(argv[1],nullptr,error,sizeof(error));
    if(!m){std::cerr<<error;return 1;}
    auto*d=mj_makeData(m);mj_resetDataKeyframe(m,d,0);mj_forward(m,d);
    rclcpp::init(argc,argv);
    int result=0;
    try {
        rclcpp::NodeOptions options;
        options.parameter_overrides({rclcpp::Parameter("openarm.require_alignment", false)});
        auto node=std::make_shared<rclcpp::Node>("openarm_bridge_test", options);
        node->declare_parameter<std::string>("out_ns","/openarm_test");
        vive_vr_ros2::OpenArmTeleop bridge(*node,m,d);
        auto clutch=node->create_publisher<std_msgs::msg::Bool>(
            "/openarm_test/teleop/right/clutch",rclcpp::QoS(1).transient_local());
        auto delta=node->create_publisher<geometry_msgs::msg::TransformStamped>(
            "/openarm_test/teleop/right/delta",rclcpp::SensorDataQoS());
        auto gripper=node->create_publisher<std_msgs::msg::Float32>(
            "/openarm_test/teleop/right/gripper",rclcpp::SensorDataQoS());
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
                    check(std::abs(d->ctrl[a]-previous[a])<=0.35/60+1e-9,"target speed exceeded");
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
        press(false);press(true);send(0,2);bridge.tick(1./60);
        for(int i=0;i<m->nu;++i)check(d->ctrl[i]==held[i],"stale delta accepted");
        send(std::numeric_limits<double>::quiet_NaN());bridge.tick(1./60);
        held.assign(d->ctrl,d->ctrl+m->nu);send(-0.02);bridge.tick(1./60);
        for(int i=0;i<m->nu;++i)check(d->ctrl[i]==held[i],"invalid delta failed to disengage");
        press(false);press(true);send(1.0);bridge.tick(1./60);
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
        std::string status;
        auto sub=aligned_node->create_subscription<std_msgs::msg::String>("/alignment_test/teleop/right/status",10,
            [&](std_msgs::msg::String::SharedPtr msg){status=msg->data;});
        auto pump=[&]{for(int i=0;i<8;++i){rclcpp::spin_some(aligned_node);std::this_thread::sleep_for(std::chrono::milliseconds(2));}};
        for(int i=0;i<100 && (cp->get_subscription_count()==0 || gp->get_subscription_count()==0 || dp->get_subscription_count()==0);++i)pump();
        auto button=[&](bool down){std_msgs::msg::Bool msg;msg.data=down;gp->publish(msg);pump();};
        auto state=[&]{gated.tick(1./60);pump();return status;};
        auto pose=[&](bool match){
            mj_kinematics(m,d);
            geometry_msgs::msg::PoseStamped msg;msg.header.frame_id="world";msg.header.stamp=aligned_node->now();
            const auto*q=d->xquat+4*mj_name2id(m,mjOBJ_BODY,"openarm_right_hand_tcp");
            mjtNum wrong[4], turn[4]={0,1,0,0};mju_mulQuat(wrong,q,turn);
            if(!match)q=wrong;
            msg.pose.orientation.w=q[0];msg.pose.orientation.x=q[1];msg.pose.orientation.y=q[2];msg.pose.orientation.z=q[3];
            cp->publish(msg);pump();
        };
        button(false);check(state()=="tracking_lost","missing tracking displayed ready");
        pose(false);check(state()=="align_orientation","mismatched orientation displayed ready");
        button(true);check(state()=="release_grip","misaligned grip engaged");
        button(false);pose(true);check(state()=="ready","aligned controller not ready");
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        check(state()=="tracking_lost","stale controller displayed ready");
        pose(true);button(true);check(state()=="waiting_delta","aligned grip did not engage");
        auto rotation=[&](double angle){
            geometry_msgs::msg::TransformStamped msg;msg.header.stamp=aligned_node->now();
            msg.header.frame_id="right_tool_ref";msg.child_frame_id="right_tool_cmd";
            msg.transform.rotation.w=std::cos(angle/2);msg.transform.rotation.y=std::sin(angle/2);
            dp->publish(msg);pump();
        };
        rotation(0);state();rotation(1.57079632679);
        auto rotated=state();check(rotated=="engaged" || rotated=="unreachable","90 degree target tripped rotation bound");
        rotation(1.9);check(state()=="release_grip","excessive rotation accepted");
        std::cout<<"PASS: alignment readiness, grip gating, stale tracking, 90-degree rotation and rotation bound\n";
        std::cout<<"PASS: release gating, delta motion, gripper, left hold, release, reclutch, timeout, stale and NaN rejection, reset\n";
    }catch(const std::exception&e){std::cerr<<e.what()<<'\n';result=1;}
    rclcpp::shutdown();mj_deleteData(d);mj_deleteModel(m);return result;
}
