// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef QR_ROBOT_LITE3_SIM_H
#define QR_ROBOT_LITE3_SIM_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "geometry_msgs/WrenchStamped.h"

#include "robots/qr_robot.h"

#include <std_msgs/Float64.h>
#include "sensor_msgs/JointState.h"
#include "extern/deeprobotics_legged_sdk/include/robot_types.h"

#include <hardware_interface/joint_command_interface.h>

namespace Quadruped {

class qrRobotLite3Sim: public qrRobot {

public:

    /**
     * @brief Constructor of class qrRobotA1Sim
     * @param nhIn: ROS node handle.
     * @param privateNhIn: private ROS node handle.
     * @param configFilePath: config file path.
     */
    qrRobotLite3Sim(ros::NodeHandle &nhIn, ros::NodeHandle &privateNhIn, std::string configFilePath);

    ~qrRobotLite3Sim() = default;

    /**
     * @brief Send command to gazebo.
     * @param motorcmd: the commands to send.
     */
    void SendCommand(const std::array<float, 60> motorcmd);

    /**
     * @see qrRobot::ReceiveObservation
     */
    void ReceiveObservation() override;

    /**
     * @see qrRobot::ApplyAction
     */
    void ApplyAction(const Eigen::MatrixXf &motorCommands, MotorMode motorControlMode) override;

    /**
     * @see qrRobot::ApplyAction
     */
    void ApplyAction(const std::vector<qrMotorCommand> &motorCommands, MotorMode motorControlMode);

    /**
     * @see qrRobot::Step
     */
    void Step(const Eigen::MatrixXf &action, MotorMode motorControlMode) override;

    /**
     * @see qrRobot::BuildDynamicModel
     */
    virtual bool BuildDynamicModel() override;

    void ImuCallback(const sensor_msgs::Imu &msg);

    void FRfootCallback(const geometry_msgs::WrenchStamped &msg);

    void FLfootCallback(const geometry_msgs::WrenchStamped &msg);

    void RRfootCallback(const geometry_msgs::WrenchStamped &msg);

    void RLfootCallback(const geometry_msgs::WrenchStamped &msg);


     void jointStateCallback_(const sensor_msgs::JointState::ConstPtr& msg);
    /**
     * @brief ROS node handle.
     */
    ros::NodeHandle & nh;

    /**
     * @brief ROS private node handle.
     */
    ros::NodeHandle & privateNh;

    /**
     * @brief 12 joint command publishers.
     */
    ros::Publisher jointCmdPub[12];

    /**
     * @brief 12 joint state subscribers.
     */
    ros::Subscriber jointStateSub[12];

    /**
     * @brief 4 force sensor subscribers.
     */
    ros::Subscriber footForceSub[4];

    /**
     * @brief Gazebo IMU subscribers.
     */
    ros::Subscriber imuSub;
    

    //bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &nh);

    hardware_interface::JointHandle joint[12];
    double effort_command[12];
    //hardware_interface::EffortJointInterface *robot
    ros::Subscriber joint_sub;
    RobotCmd      robot_cmd;
    RobotState    robot_state;
    double  effort_last[12];
    double foot_contact_force_z[4];



};

} // namespace Quadruped

#endif // QR_ROBOT_Lite3_SIM_H
