/* SPDX-License-Identifier: MIT */
#include "vive_vr_ros2/openarm_teleop.hpp"
#include <thread>
#include <iostream>
#include <vector>
#include <cmath>

void check(bool ok, const char *why) { if (!ok) throw std::runtime_error(why); }
int main(int argc, char **argv) {
    const std::string path = argc > 1 ? argv[1] : std::string(TEST_SOURCE_DIR) + "/openarm_test_arm.xml";
    char error[2048]{};
    auto *m = mj_loadXML(path.c_str(), nullptr, error, sizeof(error));
    if (!m) { std::cerr << error; return 1; }
    auto *d = mj_makeData(m); mj_resetDataKeyframe(m,d,0); mj_forward(m,d);
    rclcpp::init(argc,argv); int result=0;
    try {
        rclcpp::NodeOptions options;
        options.parameter_overrides({rclcpp::Parameter("openarm.require_alignment",false)});
        auto node=std::make_shared<rclcpp::Node>("dual_arm_check",options);
        node->declare_parameter<std::string>("out_ns","/dual_test");
        vive_vr_ros2::OpenArmTeleop right(*node,m,d,"right"), left(*node,m,d,"left");
        auto lc=node->create_publisher<std_msgs::msg::Bool>("/dual_test/teleop/left/clutch",rclcpp::QoS(1).transient_local());
        auto rc=node->create_publisher<std_msgs::msg::Bool>("/dual_test/teleop/right/clutch",rclcpp::QoS(1).transient_local());
        auto ld=node->create_publisher<geometry_msgs::msg::TransformStamped>("/dual_test/teleop/left/delta",rclcpp::SensorDataQoS());
        auto rd=node->create_publisher<geometry_msgs::msg::TransformStamped>("/dual_test/teleop/right/delta",rclcpp::SensorDataQoS());
        auto lg=node->create_publisher<std_msgs::msg::Float32>("/dual_test/teleop/left/gripper",rclcpp::SensorDataQoS());
        auto pump=[&]{for(int i=0;i<5;++i){rclcpp::spin_some(node);std::this_thread::sleep_for(std::chrono::milliseconds(2));}};
        for(int i=0;i<100 && (ld->get_subscription_count()==0 || rd->get_subscription_count()==0);++i)pump();
        check(ld->get_subscription_count()>0 && rd->get_subscription_count()>0,"dual discovery failed");
        auto press=[&](auto pub,bool value){std_msgs::msg::Bool b;b.data=value;pub->publish(b);pump();};
        auto send=[&](auto pub,const std::string &side,double angle){
            geometry_msgs::msg::TransformStamped t;t.header.frame_id=side+"_tool_ref";t.child_frame_id=side+"_tool_cmd";
            t.transform.rotation.w=std::cos(angle/2);t.transform.rotation.y=std::sin(angle/2);pub->publish(t);pump();
        };
        auto tick=[&]{left.tick(1./60);right.tick(1./60);};
        auto initial=std::vector<double>(d->ctrl,d->ctrl+m->nu);
        press(lc,false);press(rc,false);press(lc,true);send(ld,"left",0);tick();
        for(int i=0;i<12;++i){send(ld,"left",0.3);tick();}
        double moved=0;
        for(int i=0;i<m->nu;++i){
            std::string name=mj_id2name(m,mjOBJ_ACTUATOR,i);
            if(name.find("openarm_right_")==0)check(d->ctrl[i]==initial[i],"left input changed right arm");
            if(name.find("openarm_left_joint")==0)moved+=std::abs(d->ctrl[i]-initial[i]);
        }
        check(moved>1e-4,"left arm did not move");
        int lf=mj_name2id(m,mjOBJ_ACTUATOR,"openarm_left_finger_joint1_position");
        int rf=mj_name2id(m,mjOBJ_ACTUATOR,"openarm_right_finger_joint1_position");
        std_msgs::msg::Float32 g;g.data=0;lg->publish(g);pump();tick();
        g.data=1;lg->publish(g);pump();tick();
        check(d->ctrl[lf]<initial[lf],"left trigger did not close left gripper");
        check(d->ctrl[rf]==initial[rf],"left trigger changed right gripper");
        // Both clutches may remain engaged; right input must not replace the left reference.
        press(rc,true);send(rd,"right",0);tick();
        for(int i=0;i<12;++i){send(ld,"left",0.3);send(rd,"right",0.3);tick();}
        moved=0;
        for(int i=0;i<m->nu;++i){
            std::string name=mj_id2name(m,mjOBJ_ACTUATOR,i);
            if(name.find("openarm_right_joint")==0)moved+=std::abs(d->ctrl[i]-initial[i]);
        }
        check(moved>1e-4,"right arm stopped working with both clutches engaged");
        press(lc,false);tick();auto held=std::vector<double>(d->ctrl,d->ctrl+m->nu);
        for(int i=0;i<5;++i){send(rd,"right",0.4);tick();}
        for(int i=0;i<m->nu;++i)if(std::string(mj_id2name(m,mjOBJ_ACTUATOR,i)).find("openarm_left_")==0)
            check(d->ctrl[i]==held[i],"right input moved released left arm");
        std::cout<<"PASS: independent left/right IK, both clutches, left trigger, release isolation\n";
    } catch(const std::exception &e){std::cerr<<e.what()<<'\n';result=1;}
    rclcpp::shutdown();mj_deleteData(d);mj_deleteModel(m);return result;
}
