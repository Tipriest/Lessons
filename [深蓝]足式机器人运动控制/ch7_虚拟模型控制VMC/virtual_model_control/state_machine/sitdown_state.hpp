/**
 * @file sitdown_state.hpp
 * @brief 
 * @author mazunwang
 * @version 1.0
 * @date 2024-12-10
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */

#ifndef SITDOWN_STATE_HPP_
#define SITDOWN_STATE_HPP_

#include "state_base.h"

class SitDownState : public StateBase{
private:
    float time_stamp_record_, run_time_;
    VecXf kp_, kd_;
    MatXf joint_cmd_;
    float sit_duration = 2.;
    Vec3f init_foot_pos_[4];
    Vec3f target_foot_pos_[4];

    Vec3f GetLegPos(Vec3f angle, int i){
        float l0, l1, l2;
        float s1, s2, s3;
        float x, y, z, zt;
        l0 = cp_ptr_->hip_len_; l1 = cp_ptr_->thigh_len_; l2 = cp_ptr_->shank_len_;
        if(i%2==0) l0 = -cp_ptr_->hip_len_;
        s1 = angle[0]; s2 = angle[1]; s3 = angle[2];

        x = l1 * sin(s2) + l2 * sin(s2 + s3);
        zt = -(l1 * cos(s2) + l2 * cos(s2 + s3));
        y = zt * sin(s1) - l0 * cos(s1);
        z = zt * cos(s1) + l0 * sin(s1);

        return Vec3f(x, y, z);
    }

    Vec3f InverseKinematic(Vec3f pos, int leg){
        Vec3f angles;
        float x, y, z;
        float l0, l1, l2;
        float s1, s2, s3;

        l0 = cp_ptr_->hip_len_; l1 = cp_ptr_->thigh_len_; l2 = cp_ptr_->shank_len_;
        if(leg%2==0) l0 = -cp_ptr_->hip_len_;
        x = pos[0]; y = pos[1]; z = pos[2];

        s1 = atan(y / z) - asin(l0 / sqrt(y*y + z*z));
        s3 = acos((x*x + y*y + z*z - l0*l0 - l1*l1 - l2*l2) / (2 * l1*l2));
        s2 = asin(x / sqrt((l1 + l2*cos(s3))*(l1 + l2*cos(s3)) + l2*sin(s3)*l2*sin(s3))) - atan(l2*sin(s3) / (l1 + l2*cos(s3)));

        angles << s1, s2, s3;
        return angles;
    }

    void GetRobotJointValue(){
        run_time_ = ri_ptr_->GetInterfaceTimeStamp();
    }

    void RecordJointData(){
        VecXf init_joint_pos = ri_ptr_->GetJointPosition();
        time_stamp_record_ = run_time_;
        for(int i=0;i<4;++i){
            init_foot_pos_[i] = GetLegPos(init_joint_pos.segment(i*3, 3), i);
        }
    }


public:
    SitDownState(const RobotType& robot_type, const std::string& state_name, 
        std::shared_ptr<ControllerData> data_ptr):StateBase(robot_type, state_name, data_ptr){
            kp_ = cp_ptr_->swing_leg_kp_.replicate(4, 1);
            kd_ = cp_ptr_->swing_leg_kd_.replicate(4, 1);
            joint_cmd_ = MatXf::Zero(12, 5);
            joint_cmd_.col(0) = kp_;
            joint_cmd_.col(2) = kd_;
            sit_duration = 2.0;
            for(int i=0;i<4;++i)
                target_foot_pos_[i] << 0, (1.-2.*(i%2))*cp_ptr_->hip_len_, -cp_ptr_->pre_height_;

        }
    ~SitDownState(){}


    virtual void OnEnter() {
        GetRobotJointValue();
        RecordJointData();
        StateBase::msfb_.UpdateCurrentState(RobotMotionState::SitDown);
        uc_ptr_->SetMotionStateFeedback(StateBase::msfb_);
    };
    virtual void OnExit() {
    }
    virtual void Run() {
        GetRobotJointValue();
        VecXf planning_joint_pos(12);
        VecXf planning_joint_pos_next(12);
        VecXf planning_joint_vel(12);
        Vec3f planning_foot_pos[4];
        Vec3f planning_foot_pos_next[4];
        float run_time = run_time_ - time_stamp_record_;

        for(int i=0;i<4;++i){
            for(int j=0;j<3;++j){
                planning_foot_pos[i][j] = GetCubicSplinePos(init_foot_pos_[i][j], 0, target_foot_pos_[i][j], 0, 
                                                run_time, sit_duration);
                planning_foot_pos_next[i][j] = GetCubicSplinePos(init_foot_pos_[i][j], 0, target_foot_pos_[i][j], 0, 
                                                run_time+0.001, sit_duration);
            }
            planning_joint_pos.segment(3*i, 3) = InverseKinematic(planning_foot_pos[i], i);
            planning_joint_pos_next.segment(3*i, 3) = InverseKinematic(planning_foot_pos_next[i], i);
        }
        planning_joint_vel = 1./0.001*(planning_joint_pos_next - planning_joint_pos);
        joint_cmd_.col(1) = planning_joint_pos;
        joint_cmd_.col(3) = planning_joint_vel;
        ri_ptr_->SetJointCommand(joint_cmd_);
    }
    virtual bool LoseControlJudge() {
        if(uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::JointDamping)) return true;
        return false;
    }
    virtual StateName GetNextStateName() {
        if(run_time_ - time_stamp_record_ <= sit_duration){
            return StateName::kSitDown;
        }else{
            return StateName::kJointDamping;
        }
        return StateName::kSitDown;
    }
};


#endif