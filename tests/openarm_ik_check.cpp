#include "vive_vr_ros2/openarm_ik.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

int main(int argc, char** argv) {
    if (argc != 2) return 2;
    char error[2048] = {};
    mjModel* m = mj_loadXML(argv[1], nullptr, error, sizeof(error));
    if (!m) { std::cerr << error; return 1; }
    mjData* d = mj_makeData(m);
    mjData* desired = mj_makeData(m);
    mj_resetDataKeyframe(m, d, 0);
    mj_forward(m, d);
    int result = 0;
    try {
        vive_vr_ros2::OpenArmIk ik(m);
        std::vector<double> initial(d->qpos, d->qpos + m->nq);
        std::array<double, 7> solution;
        // A straight-down arm must be able to shorten (lift) despite its singular start.
        mjtNum lifted[3];mju_copy3(lifted,d->xpos+3*ik.tcp);lifted[2]+=0.02;
        check(ik.solve(d,lifted,d->xquat+4*ik.tcp,solution),"lift from home failed");
        // A reachable pose with both translation and rotation, generated independently by FK.
        mj_copyData(desired, m, d);
        desired->qpos[ik.qpos[1]] += 0.08;
        desired->qpos[ik.qpos[3]] += 0.10;
        desired->qpos[ik.qpos[5]] += 0.06;
        mj_forward(m, desired);
        check(ik.solve(d, desired->xpos+3*ik.tcp, desired->xquat+4*ik.tcp, solution),
              "reachable full-pose IK failed");
        for (int i=0;i<m->nq;++i) check(d->qpos[i]==initial[i], "IK changed live state");
        for (int i=0;i<7;++i) d->ctrl[ik.motors[i]]=solution[i];
        for (int step=0;step<2500;++step) mj_step(m,d);
        mj_forward(m,d);
        mjtNum dp[3]; mju_sub3(dp,d->xpos+3*ik.tcp,desired->xpos+3*ik.tcp);
        check(mju_norm3(dp)<0.005, "actuators did not reach IK position");
        mjtNum inverse[4], difference[4], angle[3];
        mju_negQuat(inverse,d->xquat+4*ik.tcp);
        mju_mulQuat(difference,desired->xquat+4*ik.tcp,inverse);
        if(difference[0]<0)for(auto& value:difference)value=-value;
        mju_quat2Vel(angle,difference,1.0);
        check(mju_norm3(angle)<0.06,"actuators did not reach IK orientation");
        for (int i=0;i<m->njnt;++i) {
            int q=m->jnt_qposadr[i];
            check(std::isfinite(d->qpos[q]), "nonfinite simulated joint");
            check(d->qpos[q]>=m->jnt_range[2*i]-0.001 &&
                  d->qpos[q]<=m->jnt_range[2*i+1]+0.001, "joint limit violated");
            std::string name=mj_id2name(m,mjOBJ_JOINT,i);
            if(name.find("openarm_left_")==0)
                check(std::abs(d->qpos[q]-initial[q])<1e-6,"left arm moved");
        }
        // Follow a reachable shoulder sweep to 90 degrees with position AND orientation.
        // FK supplies the reference path independently of the IK solver.
        mj_resetDataKeyframe(m,d,0);mj_forward(m,d);
        for(int step=1;step<=90;++step){
            mj_resetDataKeyframe(m,desired,0);
            desired->qpos[ik.qpos[1]]=step*3.141592653589793/180.;
            mj_forward(m,desired);
            check(ik.solve(d,desired->xpos+3*ik.tcp,desired->xquat+4*ik.tcp,solution),
                  "reachable 90-degree shoulder sweep failed");
            for(int i=0;i<7;++i)d->qpos[ik.qpos[i]]=solution[i];
            mj_forward(m,d);
        }
        mjtNum impossible[3]={100,100,100};
        check(!ik.solve(d,impossible,desired->xquat+4*ik.tcp,solution),"unreachable target accepted");
        for(int i=0;i<mjNWARNING;++i) check(d->warning[i].number==0,"MuJoCo warning");
        std::cout << "PASS: reachable pose, live-state isolation, motor tracking, limits, left hold, unreachable rejection\n";
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';result=1;}
    mj_deleteData(desired);mj_deleteData(d);mj_deleteModel(m);return result;
}
