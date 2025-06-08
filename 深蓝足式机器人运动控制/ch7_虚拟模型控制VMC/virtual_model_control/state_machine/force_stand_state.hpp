/**
 * @file force_stand_state.hpp
 * @brief quadruped robot stand by virtual model controller
 * @author mazunwang
 * @version 1.0
 * @date 2024-09-15
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */

#ifndef FORCE_STAND_STATE_HPP_
#define FORCE_STAND_STATE_HPP_

#include "state_base.h"
#include "eiquadprog.hpp"

class ForceStandState : public StateBase{
private:
    Vec3f rpy_, acc_, omg_;
    VecXf current_joint_pos_, current_joint_vel_;
    Mat3f rot_mat_;
    Vec3f fl_pos_local_, fr_pos_local_, hl_pos_local_, hr_pos_local_;
    Vec3f fl_vel_local_, fr_vel_local_, hl_vel_local_, hr_vel_local_;
    Vec3f fl_pos_global_, fr_pos_global_, hl_pos_global_, hr_pos_global_;
    Vec3f fl_vel_global_, fr_vel_global_, hl_vel_global_, hr_vel_global_;
    Vec3f com_pos_global_, com_vel_global_;
    Vec3f init_com_pos_, init_com_vel_;

    Vec3f init_rpy_;
    Mat3f init_rot_mat_;
    double time_stamp_record_, run_time_;
    Eigen::Matrix<float, 12, 5> joint_cmd_;
    float cmd_roll_, cmd_pitch_, cmd_yaw_, cmd_height_;
    float input_roll_ = 0, input_pitch_ = 0, input_yaw_ = 0, input_height_ = 0;
    float input_x_ = 0, input_y_ = 0;

    void GetProprioceptiveData(){
        run_time_ = ri_ptr_->GetInterfaceTimeStamp();
        current_joint_pos_ = ri_ptr_->GetJointPosition();
        current_joint_vel_ = ri_ptr_->GetJointVelocity();
        rpy_ = ri_ptr_->GetImuRpy();
        acc_ = ri_ptr_->GetImuAcc();
        omg_ = ri_ptr_->GetImuOmega();
        rot_mat_ = RotY(rpy_(1))*RotX(rpy_(0));
        GetLegPosLocalAndGlobal();
        cmd_pitch_ = uc_ptr_->GetUserCommand().forward_vel_scale*Deg2Rad(15.);
        cmd_yaw_ = uc_ptr_->GetUserCommand().side_vel_scale*Deg2Rad(15.);
        cmd_roll_ = uc_ptr_->GetUserCommand().turnning_vel_scale*Deg2Rad(10.);
        cmd_height_ = uc_ptr_->GetUserCommand().reserved_scale*0.1;
        cmd_height_ = LimitNumber(cmd_height_, -0.08, 0.05);
        if(fabs(cmd_pitch_) > Deg2Rad(10.) || fabs(cmd_yaw_) > Deg2Rad(10.)){
            cmd_height_ = LimitNumber(cmd_height_, -0.08, -0.05);
        }

        float delta = 0;

        delta = cmd_roll_ - input_roll_;
        delta = LimitNumber(delta, Deg2Rad(0.01));
        input_roll_ = input_roll_ + delta;

        delta = cmd_pitch_ - input_pitch_;
        delta = LimitNumber(delta, Deg2Rad(0.02));
        input_pitch_ = input_pitch_ + delta;

        delta = 0. - input_x_;
        delta = LimitNumber(delta, 3e-5);
        input_x_ = input_x_ + delta;

        delta = 0. - input_y_;
        delta = LimitNumber(delta, 3e-5);
        input_y_ = input_y_ + delta;

        delta = init_rpy_(2) + cmd_yaw_ - input_yaw_;
        delta = LimitNumber(delta, Deg2Rad(0.02));
        input_yaw_ = input_yaw_ + delta;

        delta = cp_ptr_->stand_height_ + cmd_height_ - input_height_;
        delta = LimitNumber(delta, 5e-5);
        input_height_ = input_height_ + delta;
    }

    void RecordJointData(){
        init_com_pos_ = com_pos_global_;
        init_com_vel_ = com_vel_global_;
        time_stamp_record_ = run_time_;
        init_rot_mat_ = rot_mat_;
        init_rpy_ = rpy_;

        input_roll_ = rpy_(0);
        input_pitch_ = rpy_(1);
        input_x_ = init_com_pos_(0);
        input_y_ = init_com_pos_(1);
        input_yaw_ = rpy_(2);
        input_height_ = init_com_pos_(2);
    }

    void GetLegPosLocalAndGlobal(){
        fl_pos_local_ = GetLegPos(current_joint_pos_.segment(0, 3), 0);
        fr_pos_local_ = GetLegPos(current_joint_pos_.segment(3, 3), 1);
        hl_pos_local_ = GetLegPos(current_joint_pos_.segment(6, 3), 2);
        hr_pos_local_ = GetLegPos(current_joint_pos_.segment(9, 3), 3);

        fl_vel_local_ = GetLegVel(current_joint_pos_.segment(0, 3), current_joint_vel_.segment(0, 3), 0);
        fr_vel_local_ = GetLegVel(current_joint_pos_.segment(3, 3), current_joint_vel_.segment(3, 3), 1);
        hl_vel_local_ = GetLegVel(current_joint_pos_.segment(6, 3), current_joint_vel_.segment(6, 3), 2);
        hr_vel_local_ = GetLegVel(current_joint_pos_.segment(9, 3), current_joint_vel_.segment(9, 3), 3);

        fl_pos_global_ = rot_mat_*(fl_pos_local_ + 0.5*Vec3f(cp_ptr_->body_len_x_, cp_ptr_->body_len_y_, 0));
        fr_pos_global_ = rot_mat_*(fr_pos_local_ + 0.5*Vec3f(cp_ptr_->body_len_x_, -cp_ptr_->body_len_y_, 0));
        hl_pos_global_ = rot_mat_*(hl_pos_local_ + 0.5*Vec3f(-cp_ptr_->body_len_x_, cp_ptr_->body_len_y_, 0));
        hr_pos_global_ = rot_mat_*(hr_pos_local_ + 0.5*Vec3f(-cp_ptr_->body_len_x_, -cp_ptr_->body_len_y_, 0));

        Mat3f dot_rot_mat = rot_mat_*CrossVector(omg_);
        fl_vel_global_ = rot_mat_*fl_vel_local_ + dot_rot_mat*fl_pos_local_;
        fr_vel_global_ = rot_mat_*fr_vel_local_ + dot_rot_mat*fr_pos_local_;
        hl_vel_global_ = rot_mat_*hl_vel_local_ + dot_rot_mat*hl_pos_local_;
        hr_vel_global_ = rot_mat_*hr_vel_local_ + dot_rot_mat*hr_pos_local_;

        com_pos_global_ = -(fl_pos_global_+fr_pos_global_+hl_pos_global_+hr_pos_global_)/4.;
        com_vel_global_ = -(fl_vel_global_+fr_vel_global_+hl_vel_global_+hr_vel_global_)/4.;
    }

    /**
     * @brief Get the Foot Position in HipX Frame object
     * @param  angle            leg joint position
     * @param  i                leg num FL:0, FR:1, HL:2, HR:3
     * @return Vec3f            foot position
     */
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

    /**
     * @brief Get the Leg Jacobian Matrix
     * @param  angle            leg joint position
     * @param  i                leg num FL:0, FR:1, HL:2, HR:3
     * @return Mat3f            Jacobian Matrix
     */
    Mat3f GetLegJacobian(Vec3f angle, int leg){
        float x = cp_ptr_->hip_len_, y = -cp_ptr_->thigh_len_, k = -cp_ptr_->shank_len_;
        if(leg == 1 || leg == 3) x = -cp_ptr_->hip_len_;
        float q1 = angle(0), q2 = angle(1), q3 = angle(2);
        float c1 = std::cos(q1), s1 = std::sin(q1);
        float c2 = std::cos(q2), s2 = std::sin(q2);
        float c3 = std::cos(q3), s3 = std::sin(q3);
        Mat3f J;
        J(0, 0) = 0;
        J(0, 1) = -k * (c2 * c3 - s2 * s3) - y * c2;
        J(0, 2) = -k * (c2 * c3 - s2 * s3);
        J(1, 0) = k * (c1 * c2 * c3 - c1 * s2 * s3) - x * s1 + y * c1 * c2;
        J(1, 1) = -k * (c2 * s1 * s3 + c3 * s1 * s2) - y * s1 * s2;
        J(1, 2) = -k * (c2 * s1 * s3 + c3 * s1 * s2);
        J(2, 0) = k * (s1 * s2 * s3 - c2 * c3 * s1) - x * c1 - y * c2 * s1;
        J(2, 1) = -k * (c1 * c2 * s3 + c1 * c3 * s2) - y * c1 * s2;
        J(2, 2) = -k * (c1 * c2 * s3 + c1 * c3 * s2);
        return J;
    }

    /**
     * @brief Get the Foot Velocity in HipX Frame object
     * @param  angle            leg joint position
     * @param  ang_vel          leg joint velocity
     * @param  i                leg num FL:0, FR:1, HL:2, HR:3
     * @return Vec3f            foot velocity
     */
    Vec3f GetLegVel(Vec3f angle, Vec3f ang_vel, int i){
        float l0, l1, l2;
        float s1, s2, s3;
        float ds1, ds2, ds3;
        float x, y, z, zt;
        float dx, dy, dzt, dz;
        l0 = cp_ptr_->hip_len_; l1 = cp_ptr_->thigh_len_; l2 = cp_ptr_->shank_len_;
        if(i%2==0) l0 = -cp_ptr_->hip_len_;
        s1 = angle[0]; s2 = angle[1]; s3 = angle[2];
        ds1 = ang_vel[0]; ds2 = ang_vel[1]; ds3 = ang_vel[2];

        dx = l1 * cos(s2) * ds2 + l2 * cos(s2 + s3) * (ds2 + ds3);
        zt = -(l1 * cos(s2) + l2 * cos(s2 + s3));
        dzt = l1 * sin(s2) * ds2 + l2 * sin(s2 + s3) * (ds2 + ds3);
        dy = dzt * sin(s1) + zt * cos(s1) * ds1 + l0 * sin(s1) * ds1;
        dz = dzt * cos(s1) - zt * sin(s1) * ds1 + l0 * cos(s1) * ds1;

        return Vec3f(dx, dy, dz);
    }

    /**
     * @brief Solve Equation A*f=b by Least Squares Method
     * @param  A                Matrix
     * @param  b                Matrix
     * @return VecXf            f
     */
    VecXf SolveEquationByPseudoInverse(const MatXf& A, const VecXf& b){
        if(A.rows() != 6 || A.cols() != 12 || b.rows() != 6){
            std::cerr << "input size error!!!" << std::endl;
            return VecXf::Zero(12);
        }

        /*
        you should calculate f by Least Squares Method
        */
        return VecXf::Zero(12);
    }

    /**
     * @brief Solve Equation A*f=b by Quadratic Programming
     * min ||Ax-b||w
     * subject to friction constrain
     * @param  A                matrix
     * @param  b                matrix
     * @param  w                weight matrix
     * @param  mu               ground friction coefficient
     * @param  alpha            regularization parameter
     * @return VecXf            f
     */
    VecXf SolveEquationByQP(const MatXf& A, const VecXf& b, const VecXf &w, float mu, float alpha=1e-4){
        if(A.rows() != 6 || A.cols() != 12 || b.rows() != 6){
            std::cerr << "input size error!!!" << std::endl;
            return VecXf::Zero(12);
        }
        MatXf weight = MatXf::Identity(6, 6);
        weight.diagonal() << w;
        MatXd CI(20, 12), ci0(20, 1), CE(12, 1), ce0(1, 1);
        VecXd F_solution;
        MatXd G = MatXd::Zero(6, 6);
        VecXd g0 = VecXd::Zero(6);

        /*
        you should complete quadratic programming function with friction constrain
        tips: input matrix A, b and w are float type, you can transfer to double by use "cast" function
        */
        CI.transposeInPlace();
        CE = Vec3d(0, 0, 1).replicate(4, 1);
        ce0 << -b(2);

        solve_quadprog(G, g0, CE, ce0, CI, ci0, F_solution);
        return F_solution.cast<float>();
    }

    void VirtualForceDistribution(){
        Vec3f target_base_pos = Vec3f(input_x_, input_y_, input_height_);
        Vec3f target_base_rpy = Vec3f(input_roll_, input_pitch_, input_yaw_);

        Vec3f pos_kp(40, 40, 350), ori_kp(500, 1500, 1500);
        Vec3f pos_kd(2, 2, 5), ori_kd(50, 80, 100);
        const float mass = 12.00;
        Mat3f inertia;
        inertia.setZero();
        inertia.diagonal() = Vec3f(0.03, 0.10, 0.10);

        Vec3f force, torque;
        Eigen::Matrix<float, 6, 12> A;
        Eigen::Matrix<float, 6, 1> b; 
        Eigen::Matrix<float, 12, 1> f;

        /*
        you should fulfill A and b matrix by virtual model control, make A*f equal to b.
        tips: you can use CrossVector function in util/basic_function.hpp
        */
        A << Mat3f::Identity(), Mat3f::Identity(), Mat3f::Identity(), Mat3f::Identity(),
             Mat3f::Identity(), Mat3f::Identity(), Mat3f::Identity(), Mat3f::Identity();
        force.setZero();
        torque.setZero();
        b << force, torque;

        /*
        change your control method
        */
        f = SolveEquationByPseudoInverse(A, b);
        // VecXf weight = VecXf::Ones(6);
        // f = SolveEquationByQP(A, b, weight, 0.3);

        for(int leg=0;leg<4;++leg){
            Mat3f leg_jacobian = GetLegJacobian(current_joint_pos_.segment(3*leg, 3), leg);
            Vec3f leg_force = f.segment(3*leg, 3);
            Vec3f leg_torque = -leg_jacobian.transpose()*rot_mat_.transpose()*leg_force;
            joint_cmd_.block(3*leg, 4, 3, 1) = leg_torque;
        }
    }


public:
    ForceStandState(const RobotType& robot_type, const std::string& state_name, 
        std::shared_ptr<ControllerData> data_ptr):StateBase(robot_type, state_name, data_ptr){
    }
    ~ForceStandState(){}

    virtual void OnEnter() {
        StateBase::msfb_.UpdateCurrentState(RobotMotionState::ForceStand);
        uc_ptr_->SetMotionStateFeedback(StateBase::msfb_);
        GetProprioceptiveData();
        RecordJointData();
    };
    virtual void OnExit() {
    }
    virtual void Run() {
        GetProprioceptiveData();
        joint_cmd_.setZero();
        VirtualForceDistribution();
        ri_ptr_->SetJointCommand(joint_cmd_);
    }
    virtual bool LoseControlJudge() {
        if(uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::JointDamping)) return true;
        Vec3f rpy = ri_ptr_->GetImuRpy();
        if(fabs(rpy(0)) > 25./180*M_PI || fabs(rpy(1)) > 30./180*M_PI){
            std::cout << "posture value: " << 180./M_PI*rpy.transpose() << std::endl;
            return true;
        }
        return false;
    }
    virtual StateName GetNextStateName() {
        if(run_time_ - time_stamp_record_ > 1.0 
            && uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::SitDown)) {
                return StateName::kSitDown;
            }
        return StateName::kForceStand;
    }
};

#endif