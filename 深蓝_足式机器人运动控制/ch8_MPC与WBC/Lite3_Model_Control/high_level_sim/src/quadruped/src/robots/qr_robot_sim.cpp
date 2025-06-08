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

#include "robots/qr_robot_sim.h"
#include <vector>
#include <std_msgs/Float64.h>

using namespace std;
namespace Quadruped {

qrRobotSim::qrRobotSim(ros::NodeHandle &nhIn, ros::NodeHandle &privateNhIn, std::string robotName, std::string homeDir):
    qrRobot(robotName + "_sim", homeDir + "config/" + robotName + "_sim/robot.yaml"),
    nh(nhIn),
    privateNh(privateNhIn)
{
    std::cout << robotName <<std::endl;
    baseOrientation << 1.f, 0.f, 0.f, 0.f;
    baseRollPitchYaw << 0.f, 0.f, 0.f;
    baseRollPitchYawRate << 0.f, 0.f, 0.f;
    motorVelocities = Eigen::Matrix<float, 12, 1>::Zero();
    footForce << 0.f, 0.f, 0.f, 0.f;
    footContact << 1, 1, 1, 1;
    dq_last[12]=0.0;
    foot_contact_force_z[4]={0};
    effort_last[12]={0};
    std::cout << configFilePath << std::endl;

    robotConfig = YAML::LoadFile(configFilePath);

    std::cout << "file loaded..." << std::endl;

    robotName = robotConfig["name"].as<std::string>();
    isSim = robotConfig["is_sim"].as<bool>();
    totalMass = robotConfig["robot_params"]["total_mass"].as<float>();
    bodyMass = robotConfig["robot_params"]["body_mass"].as<float>();

    std::vector<float> totalInertiaVec = robotConfig["robot_params"]["total_inertia"].as<std::vector<float >>();
    totalInertia = Eigen::MatrixXf::Map(&totalInertiaVec[0], 3, 3);
    std::vector<float> bodyInertiaVec = robotConfig["robot_params"]["body_inertia"].as<std::vector<float >>();
    bodyInertia = Eigen::MatrixXf::Map(&bodyInertiaVec[0], 3, 3);

    std::vector<std::vector<float>> inertias = robotConfig["robot_params"]["links_inertia"].as<std::vector<std::vector<float>>>();
    std::vector<float> masses = robotConfig["robot_params"]["links_mass"].as<std::vector<float>>();
    std::vector<std::vector<float>> linksComPos_ = robotConfig["robot_params"]["links_com_pos"].as<std::vector<std::vector<float>>>();

    for (int i=0; i < 9; ++i) {
        std::cout << inertias[0][i] << std::endl;
    }

    for (int legId=0; legId<NumLeg;++legId) {
        Mat3<float> inertia = Mat3<float>::Zero();
        inertia = Eigen::MatrixXf::Map(&inertias[0][0], 3, 3);
        linkInertias.push_back(inertia); // hip link
        inertia = Eigen::MatrixXf::Map(&inertias[1][0], 3, 3);
        linkInertias.push_back(inertia); // thigh link
        inertia = Eigen::MatrixXf::Map(&inertias[2][0], 3, 3);
        linkInertias.push_back(inertia); // calf link

        linkMasses.push_back(masses[0]);
        linkMasses.push_back(masses[1]);
        linkMasses.push_back(masses[2]);

        linksComPos.push_back(linksComPos_[0]);
        linksComPos.push_back(linksComPos_[1]);
        linksComPos.push_back(linksComPos_[2]);
    }

    bodyHeight = robotConfig["robot_params"]["body_height"].as<float>();
    std::vector<float> abadLocation_ = robotConfig["robot_params"]["abad_location"].as<std::vector<float>>();
    abadLocation = Eigen::MatrixXf::Map(&abadLocation_[0], 3, 1);
    hipLength = robotConfig["robot_params"]["hip_l"].as<float>();
    upperLegLength = robotConfig["robot_params"]["upper_l"].as<float>();
    lowerLegLength = robotConfig["robot_params"]["lower_l"].as<float>();

    std::vector<std::vector<float>> defaultHipPositionList =
        robotConfig["robot_params"]["default_hip_positions"].as<std::vector<std::vector<float>>>();
    Eigen::Matrix<float, 3, 1> defaultHipPositionFR = Eigen::MatrixXf::Map(&defaultHipPositionList[0][0], 3, 1);
    Eigen::Matrix<float, 3, 1> defaultHipPositionFL = Eigen::MatrixXf::Map(&defaultHipPositionList[1][0], 3, 1);
    Eigen::Matrix<float, 3, 1> defaultHipPositionRL = Eigen::MatrixXf::Map(&defaultHipPositionList[2][0], 3, 1);
    Eigen::Matrix<float, 3, 1> defaultHipPositionRR = Eigen::MatrixXf::Map(&defaultHipPositionList[3][0], 3, 1);
    defaultHipPosition << defaultHipPositionFR, defaultHipPositionFL, defaultHipPositionRL, defaultHipPositionRR;

    float abadKp, abadKd, hipKp, hipKd, kneeKp, kneeKd;
    abadKp = robotConfig["motor_params"]["abad_p"].as<float>();
    abadKd = robotConfig["motor_params"]["abad_d"].as<float>();
    hipKp = robotConfig["motor_params"]["hip_p"].as<float>();
    hipKd = robotConfig["motor_params"]["hip_d"].as<float>();
    kneeKp = robotConfig["motor_params"]["knee_p"].as<float>();
    kneeKd = robotConfig["motor_params"]["knee_d"].as<float>();
    Eigen::Matrix<float, 3, 1> kps(abadKp, hipKp, kneeKp);
    Eigen::Matrix<float, 3, 1> kds(abadKd, hipKd, kneeKd);
    motorKps << kps, kps, kps, kps;
    motorKds << kds, kds, kds, kds;

    std::vector<float>
        jointDirectionList = robotConfig["motor_params"]["joint_directions"].as<std::vector<float >>();
    std::vector<float>
        jointOffsetList = robotConfig["motor_params"]["joint_offsets"].as<std::vector<float >>();
    jointDirection = Eigen::MatrixXf::Map(&jointDirectionList[0], 12, 1);
    jointOffset = Eigen::MatrixXf::Map(&jointOffsetList[0], 12, 1);

    float standUpAbAngle, standUpHipAngle, standUpKneeAngle;
    standUpAbAngle   = robotConfig["robot_params"]["default_standup_angle"]["ab"].as<float>();
    standUpHipAngle  = robotConfig["robot_params"]["default_standup_angle"]["hip"].as<float>();
    standUpKneeAngle = robotConfig["robot_params"]["default_standup_angle"]["knee"].as<float>();
    Eigen::Matrix<float, 3, 1> defaultStandUpAngle(standUpAbAngle, standUpHipAngle, standUpKneeAngle);
    standUpMotorAngles << defaultStandUpAngle, defaultStandUpAngle, defaultStandUpAngle, defaultStandUpAngle;

    float sitDownAbAngle, sitDownHipAngle, sitDownKneeAngle;
    sitDownAbAngle   = robotConfig["robot_params"]["default_sitdown_angle"]["ab"].as<float>();
    sitDownHipAngle  = robotConfig["robot_params"]["default_sitdown_angle"]["hip"].as<float>();
    sitDownKneeAngle = robotConfig["robot_params"]["default_sitdown_angle"]["knee"].as<float>();
    Eigen::Matrix<float, 3, 1> defaultSitDownAngle(sitDownAbAngle, sitDownHipAngle, sitDownKneeAngle);
    sitDownMotorAngles << defaultSitDownAngle, defaultSitDownAngle, defaultSitDownAngle, defaultSitDownAngle;

    controlParams["mode"] = robotConfig["controller_params"]["mode"].as<int>();
    Reset(); // reset com_offset

    imuSub = nh.subscribe("/trunk_imu", 1, &qrRobotSim::ImuCallback, this);

    std::cout << robotName + "_gazebo/FR_hip_controller/state" << std::endl;

    joint_sub=   nh.subscribe("/lite3_gazebo/joint_states", 1, &qrRobotSim::jointStateCallback_,this);



    footForceSub[0] = nh.subscribe("/lite3/FR_foot_contact/contact_force", 1, &qrRobotSim::FRfootCallback, this);
    footForceSub[1] = nh.subscribe("/lite3/FL_foot_contact/contact_force", 1, &qrRobotSim::FLfootCallback, this);
    footForceSub[2] = nh.subscribe("/lite3/RR_foot_contact/contact_force", 1, &qrRobotSim::RRfootCallback, this);
    footForceSub[3] = nh.subscribe("/lite3/RL_foot_contact/contact_force", 1, &qrRobotSim::RLfootCallback, this); 

  


     vector<std::string>  joint_command_topics= {
                                              
                                              "/lite3_gazebo/FL_HipX/command",
                                              "/lite3_gazebo/FL_HipY/command",
                                              "/lite3_gazebo/FL_Knee/command",
                                              
                                              "/lite3_gazebo/FR_HipX/command",
                                              "/lite3_gazebo/FR_HipY/command",
                                              "/lite3_gazebo/FR_Knee/command",

                                              "/lite3_gazebo/HL_HipX/command",
                                              "/lite3_gazebo/HL_HipY/command",
                                              "/lite3_gazebo/HL_Knee/command",

                                              "/lite3_gazebo/HR_HipX/command",
                                              "/lite3_gazebo/HR_HipY/command",
                                              "/lite3_gazebo/HR_Knee/command"
                                               };

      for(int i = 0; i < joint_command_topics.size(); i++){
          jointCmdPub[i] = nh.advertise<std_msgs::Float64>(joint_command_topics[i],1);
      }

    // ros::spinOnce();
    usleep(300000); // must wait 300ms, to get first state

    yawOffset = 0;//lowState.imu.rpy[2]; // todo
    std::cout << "yawOffset: " << yawOffset << std::endl;

    // timeStep = 1.0 / robotConfig["controller_params"]["freq"].as<int>();
    this->ResetTimer();
    lastResetTime = GetTimeSinceReset();
    initComplete = true;
    std::cout << "------ Robot for simulation init Complete ------" << std::endl;
}


bool qrRobotSim::BuildDynamicModel()
{
    std::vector<float> bodySize = robotConfig["robot_params"]["body_size"].as<std::vector<float>>(); // Length, Width, Height
    Vec3<float> bodyDims(bodySize[0], bodySize[1], bodySize[2]);

    // locations
    Vec3<float> _abadRotorLocation = {0.935f, 0.062f, 0.f}; // lite3
    Vec3<float> _abadLocation = abadLocation;
    Vec3<float> _hipLocation = Vec3<float>(0, hipLength, 0);
    Vec3<float> _hipRotorLocation = Vec3<float>(0, 0.0, 0); // lite3
    Vec3<float> _kneeLocation = Vec3<float>(0, 0, -upperLegLength);
    Vec3<float> _kneeRotorLocation = Vec3<float>(0, -0.50, 0); // lite3

    float scale_ = 1.0; //1e-2(sim);
    // rotor inertia if the rotor is oriented so it spins around the z-axis
    Mat3<float> rotorRotationalInertiaZ;
   
    rotorRotationalInertiaZ << 11, 0, 0, // lite3
                               0, 18, 0,
                               0, 0, 11;
    rotorRotationalInertiaZ.setIdentity();
    rotorRotationalInertiaZ = scale_*1e-6 * rotorRotationalInertiaZ;

    Mat3<float> RY = coordinateRotation<float>(CoordinateAxis::Y, M_PI / 2);
    Mat3<float> RX = coordinateRotation<float>(CoordinateAxis::X, M_PI / 2);
    Mat3<float> rotorRotationalInertiaX = RY * rotorRotationalInertiaZ * RY.transpose();
    Mat3<float> rotorRotationalInertiaY = RX * rotorRotationalInertiaZ * RX.transpose();

    // spatial inertias of leg links
    Mat3<float> abadRotationalInertia = linkInertias[0];
    Vec3<float> abadCOM(linksComPos[0][0],linksComPos[0][1],linksComPos[0][2]);
    std::cout << "abadCOM=" << abadCOM << std::endl;
    std::cout << "linkMasses[0]=" << linkMasses[0] << std::endl;
    std::cout << "abadRotationalInertia=" << abadRotationalInertia << std::endl;
    SpatialInertia<float> abadInertia(linkMasses[0], abadCOM, abadRotationalInertia);

    Mat3<float> hipRotationalInertia = linkInertias[1];
    Vec3<float> hipCOM(linksComPos[1][0],linksComPos[1][1],linksComPos[1][2]);
    SpatialInertia<float> hipInertia(linkMasses[1], hipCOM, hipRotationalInertia);
    std::cout << "linkMasses[1]=" << linkMasses[1] << std::endl;
    std::cout << "hipRotationalInertia=" << hipRotationalInertia << std::endl;

    Mat3<float> kneeRotationalInertia, kneeRotationalInertiaRotated;
    kneeRotationalInertiaRotated = linkInertias[2];
    kneeRotationalInertia = kneeRotationalInertiaRotated;//RY * kneeRotationalInertiaRotated * RY.transpose();

    Vec3<float> kneeCOM(linksComPos[2][0],linksComPos[2][1],linksComPos[2][2]);
    SpatialInertia<float> kneeInertia(linkMasses[2], kneeCOM, kneeRotationalInertia);
    std::cout << "linkMasses[2]=" << linkMasses[2] << std::endl;
    std::cout << "kneeRotationalInertia=" << kneeRotationalInertia << std::endl;

    // rotors
    Vec3<float> rotorCOM(0, 0, 0);
    float rotorMass = 0.0708; //  0.0708(lite3)
    SpatialInertia<float> rotorInertiaX(rotorMass, rotorCOM, rotorRotationalInertiaX);
    SpatialInertia<float> rotorInertiaY(rotorMass, rotorCOM, rotorRotationalInertiaY);
    auto& abadRotorInertia = rotorInertiaX;
    float abadGearRatio = 12; //1(sim) 12(lite3)
    auto& hipRotorInertia = rotorInertiaY;
    float hipGearRatio = 12; // 6, 12
    auto& kneeRotorInertia = rotorInertiaY;
    float kneeGearRatio = 18; // 9.33, 18
    float kneeLinkY_offset = 0.004;

    // body
    Mat3<float> bodyRotationalInertia = bodyInertia;
    Vec3<float> bodyCOM = comOffset;
    std::cout << "totalMass = " << totalMass << ", bodyMass = " << bodyMass << std::endl;
    SpatialInertia<float> bodyInertia_(bodyMass, bodyCOM, bodyRotationalInertia);

    const int baseID = 5;
    int bodyID = baseID;
    float sideSign = -1;
    Mat3<float> I3 = Mat3<float>::Identity();
    model.addBase(bodyInertia_);
    model.addGroundContactBoxPoints(bodyID, bodyDims);

    for (int legID = 0; legID < NumLeg; legID++) {
        // Ab/Ad joint
        //  int addBody(const SpatialInertia<T>& inertia, const SpatialInertia<T>& rotorInertia, T gearRatio,
            //  int parent, JointType jointType, CoordinateAxis jointAxis,
            //  const Mat6<T>& Xtree, const Mat6<T>& Xrot);
        bodyID++;

        Vec3<float> offsetZ(0,0,0);
        // if (legID>1) offsetZ << 0,0,+0.02; //todo

        Mat6<float> xtreeAbad = createSXform(I3, WithLegSigns(_abadLocation + offsetZ, legID));
        Mat6<float> xtreeAbadRotor = createSXform(I3, WithLegSigns(_abadRotorLocation, legID));
        if (sideSign < 0) {
            model.addBody(abadInertia.flipAlongAxis(CoordinateAxis::Y),
                          abadRotorInertia.flipAlongAxis(CoordinateAxis::Y),
                          abadGearRatio, baseID, JointType::Revolute,
                          CoordinateAxis::X, xtreeAbad, xtreeAbadRotor);
        } else {
            model.addBody(abadInertia, abadRotorInertia, abadGearRatio, baseID,
                          JointType::Revolute, CoordinateAxis::X, xtreeAbad,
                          xtreeAbadRotor);
        }
        // model.addGroundContactPoint(bodyID, withLegSigns(_hipLocation, legID));

        // Hip Joint
        bodyID++;
        Mat6<float> xtreeHip =
            createSXform(I3, //coordinateRotation(CoordinateAxis::Z, float(M_PI)),
                        WithLegSigns(_hipLocation, legID)); // 0, hipLength=0.085, 0
        Mat6<float> xtreeHipRotor =
            createSXform(coordinateRotation(CoordinateAxis::Z, float(M_PI)),
                        WithLegSigns(_hipRotorLocation, legID));
        if (sideSign < 0) {
            model.addBody(hipInertia.flipAlongAxis(CoordinateAxis::Y),
                          hipRotorInertia.flipAlongAxis(CoordinateAxis::Y),
                          hipGearRatio, bodyID - 1, JointType::Revolute,
                          CoordinateAxis::Y, xtreeHip, xtreeHipRotor);
        } else {
            model.addBody(hipInertia, hipRotorInertia, hipGearRatio, bodyID - 1,
                          JointType::Revolute, CoordinateAxis::Y, xtreeHip,
                          xtreeHipRotor);
        }

        // add knee ground contact point
        model.addGroundContactPoint(bodyID, Vec3<float>(0, 0, -upperLegLength));

        // Knee Joint
        bodyID++;
        Mat6<float> xtreeKnee = createSXform(I3, _kneeLocation);
        Mat6<float> xtreeKneeRotor = createSXform(I3, _kneeRotorLocation);
        if (sideSign < 0) {
            model.addBody(kneeInertia, //.flipAlongAxis(CoordinateAxis::Y),
                          kneeRotorInertia,//.flipAlongAxis(CoordinateAxis::Y),
                          kneeGearRatio, bodyID - 1, JointType::Revolute,
                          CoordinateAxis::Y, xtreeKnee, xtreeKneeRotor);

            model.addGroundContactPoint(bodyID, Vec3<float>(0, kneeLinkY_offset, -lowerLegLength), true);
        } else {
            model.addBody(kneeInertia, kneeRotorInertia, kneeGearRatio, bodyID - 1,
                          JointType::Revolute, CoordinateAxis::Y, xtreeKnee,
                          xtreeKneeRotor);

            model.addGroundContactPoint(bodyID, Vec3<float>(0, -kneeLinkY_offset, -lowerLegLength), true);
        }

        // add foot
        //model.addGroundContactPoint(bodyID, Vec3<T>(0, 0, -_kneeLinkLength), true);

        sideSign *= -1;
    }

    Vec3<float> g(0, 0, -9.81);
    model.setGravity(g);

    return true;
}


void qrRobotSim::ImuCallback(const sensor_msgs::Imu &msg)
{
    Eigen::Matrix<float, 4, 1> quaternion = {static_cast<float>(msg.orientation.w),
                                             static_cast<float>(msg.orientation.x),
                                             static_cast<float>(msg.orientation.y),
                                             static_cast<float>(msg.orientation.z)};
    Eigen::Matrix<float, 3, 1> rpy = robotics::math::quatToRPY(quaternion);

    robot_state.imu.angle_roll =rpy[0]*180.0/M_PI;
    robot_state.imu.angle_pitch=rpy[1]*180.0/M_PI;
    robot_state.imu.angle_yaw  =rpy[2]*180.0/M_PI;

    robot_state.imu.angular_velocity_roll =msg.angular_velocity.x;;
    robot_state.imu.angular_velocity_pitch=msg.angular_velocity.y;
    robot_state.imu.angular_velocity_yaw  =msg.angular_velocity.z;
   
    robot_state.imu.acc_x=msg.linear_acceleration.x;
    robot_state.imu.acc_y=msg.linear_acceleration.y;
    robot_state.imu.acc_z=msg.linear_acceleration.z;
}

void qrRobotSim::jointStateCallback_(const sensor_msgs::JointState::ConstPtr& msg){
    
  for(int i=0;i<12;i++)
  {
   robot_state.motor_state.joint_data[i].pos=msg->position[i];
   robot_state.motor_state.joint_data[i].vel=msg->velocity[i];


    if(isnan(msg->effort[i]))
   {
   robot_state.motor_state.joint_data[i].tor=effort_last[i];
   }
   else
   {
   robot_state.motor_state.joint_data[i].tor=msg->effort[i];
                              effort_last[i]=msg->effort[i];
   } 
   // robot_state.motor_state.joint_data[i].tor=effort_feedback[i];

   if(isnan(msg->position[i])||isnan(msg->velocity[i]))
    std::cout<<"jointstatecall_back exist nan value"<<std::endl;

  }



std::cout<<" msg->position[i]= "<<msg->position[0]<<"  "<<msg->position[1]<<"  "<<msg->position[2]<<"  "<<msg->position[3]<<"  "<<msg->position[4]<<"  "<<msg->position[5]<<" "<<msg->position[6]<<"  "<<msg->position[7]<<"  "<<msg->position[8]<<"  "<<msg->position[9]<<"  "<<msg->position[10]<<"  "<<msg->position[11]<<std::endl;

}


void qrRobotSim::FRfootCallback(const geometry_msgs::WrenchStamped &msg)
{
    foot_contact_force_z[0] = msg.wrench.force.z;
}


void qrRobotSim::FLfootCallback(const geometry_msgs::WrenchStamped &msg)
{
    foot_contact_force_z[1] = msg.wrench.force.z;
}


void qrRobotSim::RRfootCallback(const geometry_msgs::WrenchStamped &msg)
{
    foot_contact_force_z[2] = msg.wrench.force.z;
}


void qrRobotSim::RLfootCallback(const geometry_msgs::WrenchStamped &msg)
{
   foot_contact_force_z[3] = msg.wrench.force.z;
} 


void qrRobotSim::SendCommand(const std::array<float, 60> motorcmd)
{
      for (int motor_id = 0; motor_id < 12; motor_id++) {
        int motorId_ = ((motor_id/3)%2 == 0? motor_id+3: motor_id-3);
        //   int motorId_ = motor_id;

        robot_cmd.joint_cmd[motorId_].pos =- motorcmd[motor_id * 5];
        robot_cmd.joint_cmd[motorId_].kp  = motorcmd[motor_id * 5 + 1];
        robot_cmd.joint_cmd[motorId_].vel =- motorcmd[motor_id * 5 + 2];
        robot_cmd.joint_cmd[motorId_].kd  = motorcmd[motor_id * 5 + 3];
        robot_cmd.joint_cmd[motorId_].tor = -motorcmd[motor_id * 5 + 4];
    }
 //   std::cout<<" pos="<<motorcmd[10]<<" kp="<<motorcmd[10+1]<<" vel="<<motorcmd[10+2]<<" kd="<<motorcmd[10+3]<<" tor="<< motorcmd[10+4]<<std::endl;

    


    for (int m = 0; m < 12; m++) {
        std_msgs::Float64 command_message;
       // int m_ = ((m/3)%2 == 0? m+3: m-3);
         command_message.data  =robot_cmd.joint_cmd[m].kp*(robot_cmd.joint_cmd[m].pos-robot_state.motor_state.joint_data[m].pos);
         command_message.data +=robot_cmd.joint_cmd[m].kd*(robot_cmd.joint_cmd[m].vel-robot_state.motor_state.joint_data[m].vel);
         command_message.data +=robot_cmd.joint_cmd[m].tor;
         jointCmdPub[m].publish(command_message);
    }

  //  std::cout<<" robot_cmd.joint_cmd= "<<robot_cmd.joint_cmd[0].kp<<"  "<<robot_cmd.joint_cmd[0].kd<<std::endl;

    // ros::spinOnce();
    // usleep(1000);
}

void qrRobotSim::SendCommand666(const RobotCmd motorCommandsArray)
{
      

for (int motor_id = 0; motor_id < 12; motor_id++) {
        robot_cmd.joint_cmd[motor_id].pos = motorCommandsArray.joint_cmd[motor_id].pos;
        robot_cmd.joint_cmd[motor_id].kp  = motorCommandsArray.joint_cmd[motor_id].kp;
        robot_cmd.joint_cmd[motor_id].vel = motorCommandsArray.joint_cmd[motor_id].vel;
        robot_cmd.joint_cmd[motor_id].kd  = motorCommandsArray.joint_cmd[motor_id].kd;
        robot_cmd.joint_cmd[motor_id].tor = motorCommandsArray.joint_cmd[motor_id].tor;
    }

   std::cout<<"robot_cmd pos="<<robot_cmd.joint_cmd[0].pos<<"  "<<robot_cmd.joint_cmd[1].pos<<" "<<robot_cmd.joint_cmd[2].pos<<std::endl;

    for (int m = 0; m < 12; m++) {
        std_msgs::Float64 command_message;
       // int m_ = ((m/3)%2 == 0? m+3: m-3);
         command_message.data  =robot_cmd.joint_cmd[m].kp*(robot_cmd.joint_cmd[m].pos- robot_state.motor_state.joint_data[m].pos);
         command_message.data +=robot_cmd.joint_cmd[m].kd*(robot_cmd.joint_cmd[m].vel- robot_state.motor_state.joint_data[m].vel);
         command_message.data +=robot_cmd.joint_cmd[m].tor+robot_state.motor_state.joint_data[m].tor;
         jointCmdPub[m].publish(command_message);
    }

  //  std::cout<<" robot_cmd.joint_cmd= "<<robot_cmd.joint_cmd[0].kp<<"  "<<robot_cmd.joint_cmd[0].kd<<std::endl;

    // ros::spinOnce();
    // usleep(1000);
}



void qrRobotSim::ReceiveObservation()
{
  
    
    baseAccInBaseFrame << robot_state.imu.acc_x,
                          robot_state.imu.acc_y,
                          robot_state.imu.acc_z;
    stateDataFlow.baseLinearAcceleration = accFilter.CalculateAverage(baseAccInBaseFrame);
    
    std::array<float, 3> rpy;
    //std::copy(std::begin(robotstate.imu.rpy), std::end(robotstate.imu.rpy), std::begin(rpy));

    rpy[0]=robot_state.imu.angle_roll/(180.0/M_PI);
    rpy[1]=robot_state.imu.angle_pitch/(180.0/M_PI);
    rpy[2]=robot_state.imu.angle_yaw/(180.0/M_PI);

    // calibrated
    float calibratedYaw = rpy[2] - yawOffset;
    if (calibratedYaw >= M_PI) {
        calibratedYaw -= M_2PI;
    } else if (calibratedYaw <= -M_PI) {
        calibratedYaw += M_2PI;
    }
    // baseRollPitchYaw << rpy[0], rpy[1], calibratedYaw;
    if (abs(baseRollPitchYaw[2]-calibratedYaw) >= M_PI) {
        rpyFilter.Reset();
    }
    Vec3<float> rpyVec(rpy[0], rpy[1], calibratedYaw);
    baseRollPitchYaw = rpyFilter.CalculateAverage(rpyVec);
    if (baseRollPitchYaw[2] >= M_PI) {
        baseRollPitchYaw[2] -= M_2PI;
    } else if (baseRollPitchYaw[2] <= -M_PI) {
        baseRollPitchYaw[2] += M_2PI;
    }
    
    std::array<float, 3> gyro;
    //std::copy(std::begin(robotstate.imu.gyroscope), std::end(robotstate.imu.gyroscope), std::begin(gyro));
    gyro[0]=robot_state.imu.angular_velocity_roll;
    gyro[1]=robot_state.imu.angular_velocity_pitch;
    gyro[2]=robot_state.imu.angular_velocity_yaw;

    Vec3<float> gyroVec(gyro[0], gyro[1], gyro[2]);
    baseRollPitchYawRate = gyroFilter.CalculateAverage(gyroVec);

    // baseRollPitchYaw << rpyVec[0], rpyVec[1], calibratedYaw;
    baseOrientation = robotics::math::rpyToQuat(baseRollPitchYaw);
    // baseRollPitchYawRate << gyro[0], gyro[1], gyro[2];


 for (int motorId = 0; motorId < NumMotor; ++motorId) {
        int motorId_ = ((motorId/3)%2 == 0? motorId+3: motorId-3);
        //int motorId_ = motorId;
        motorAngles[motorId] = robot_state.motor_state.joint_data[motorId_].pos;
        motorVelocities[motorId] = robot_state.motor_state.joint_data[motorId_].vel;
      //  motorddq[motorId] = (robotstate.motor_state.joint_data[motorId].vel-dq_last[motorId])/0.001;
        motortorque[motorId] = robot_state.motor_state.joint_data[motorId_].tor;

       // dq_last[motorId]=robotstate.motor_state.joint_data[motorId_].vel;
    }

    motorAngles = jointDirection.cwiseProduct(motorAngles + jointOffset);
    motorVelocities = jointDirection.cwiseProduct(motorVelocities);
   // motorddq = jointDirection.cwiseProduct(motorddq);
    motortorque = jointDirection.cwiseProduct(motortorque);
   
    motorVelocities = motorVFilter.CalculateAverage(motorVelocities);


  // std::cout<<"jointOffset[0]= "<<jointOffset[0]<<"jointOffset[1]= "<<jointOffset[1]<<"jointOffset[2]= "<<jointOffset[2]<<std::endl;
    
    std::cout<<" motorAngles= "<<motorAngles[0]<<"  "<<motorAngles[1]<<"  "<<motorAngles[2]<<"  "<<motorAngles[3]<<"  "<<motorAngles[4]<<"  "<<motorAngles[5]<<" "<<motorAngles[6]<<"  "<<motorAngles[7]<<"  "<<motorAngles[8]<<"  "<<motorAngles[9]<<"  "<<motorAngles[10]<<"  "<<motorAngles[11]<<std::endl;

    //boost::array<int16_t, 4> force = state.footForce;
    footForce << foot_contact_force_z[0], foot_contact_force_z[1], foot_contact_force_z[2], foot_contact_force_z[3];
      std::cout<<"foot_contact_force= "<<foot_contact_force_z[0]<<"  "<<foot_contact_force_z[1]<<"  "<<foot_contact_force_z[2]<<"  "<<foot_contact_force_z[3]<<std::endl;
    for (int footId = 0; footId < NumLeg; footId++) {
        if (footForce[footId] >= 5) {
            footContact[footId] = true;
        } else {
            footContact[footId] = false;
        }
    }

    UpdateDataFlow();
}


void qrRobotSim::ApplyAction(const Eigen::MatrixXf &motorCommands,
                           MotorMode motorControlMode)
{
  std::array<float, 60> motorCommandsArray = {0};
  if (motorControlMode == POSITION_MODE) {
      Eigen::Matrix<float, 12, 1> motorCommandsShaped = motorCommands;
      motorCommandsShaped = jointDirection.cwiseProduct(motorCommandsShaped) - jointOffset;
      for (int motorId = 0; motorId < NumMotor; ++motorId) {
          motorCommandsArray[motorId * 5] = motorCommandsShaped[motorId];
          motorCommandsArray[motorId * 5 + 1] = motorKps[motorId];
          motorCommandsArray[motorId * 5 + 2] = 0;
          motorCommandsArray[motorId * 5 + 3] = motorKds[motorId];
          motorCommandsArray[motorId * 5 + 4] = 0;
      }
  } else if (motorControlMode == TORQUE_MODE) {
      Eigen::Matrix<float, 12, 1> motorCommandsShaped = motorCommands;
      motorCommandsShaped = jointDirection.cwiseProduct(motorCommandsShaped);
      for (int motorId = 0; motorId < NumMotor; ++motorId) {
          motorCommandsArray[motorId * 5] = 0;
          motorCommandsArray[motorId * 5 + 1] = 0;
          motorCommandsArray[motorId * 5 + 2] = 0;
          motorCommandsArray[motorId * 5 + 3] = 0;
          motorCommandsArray[motorId * 5 + 4] = motorCommandsShaped[motorId];
      }
  } else if (motorControlMode == HYBRID_MODE) {
      Eigen::Matrix<float, 5, 12> motorCommandsShaped = motorCommands;
      Eigen::Matrix<float, 12, 1> angles = motorCommandsShaped.row(POSITION).transpose();
      motorCommandsShaped.row(POSITION) = (jointDirection.cwiseProduct(angles) - jointOffset).transpose();
      Eigen::Matrix<float, 12, 1> vels = motorCommandsShaped.row(VELOCITY).transpose();
      motorCommandsShaped.row(VELOCITY) = jointDirection.cwiseProduct(vels).transpose();
      Eigen::Matrix<float, 12, 1> tuas = motorCommandsShaped.row(TORQUE).transpose();
      motorCommandsShaped.row(TORQUE) = jointDirection.cwiseProduct(tuas).transpose();

      for (int motorId = 0; motorId < NumMotor; ++motorId) {
          motorCommandsArray[motorId * 5] = motorCommandsShaped(POSITION, motorId);
          motorCommandsArray[motorId * 5 + 1] = motorCommandsShaped(KP, motorId);
          motorCommandsArray[motorId * 5 + 2] = motorCommandsShaped(VELOCITY, motorId);
          motorCommandsArray[motorId * 5 + 3] = motorCommandsShaped(KD, motorId);
          motorCommandsArray[motorId * 5 + 4] = motorCommandsShaped(TORQUE, motorId);
      }
  }

  for (int index=0; index< motorCommandsArray.size(); index++) {
      if (isnan(motorCommandsArray[index])) {
          motorCommandsArray[index] = 0.f;
      }
  }
  SendCommand(motorCommandsArray);
}
 



void qrRobotSim::ApplyAction666(const Eigen::MatrixXf &motorCommands, MotorMode motorControlMode)
{
    // std::array<float, 60> motorCommandsArray = {0};
    RobotCmd motorCommandsArray;
    // float t = GetTimeSinceReset();

    if (motorControlMode == POSITION_MODE) {
        Eigen::Matrix<float, 12, 1> motorCommandsShaped = motorCommands;
        lastMotorCommands.col(0) << motorCommandsShaped;

        motorCommandsShaped = jointDirection.cwiseProduct(motorCommandsShaped) - jointOffset;
        
        for (int motorId = 0; motorId < NumMotor; motorId++) {
            int motorId_ = (motorId/3)%2 == 0? motorId+3: motorId-3;
            motorCommandsArray.joint_cmd[motorId_].pos = motorCommandsShaped[motorId];
            motorCommandsArray.joint_cmd[motorId_].kp = motorKps[motorId];
            motorCommandsArray.joint_cmd[motorId_].vel = 0;
            motorCommandsArray.joint_cmd[motorId_].kd = motorKds[motorId];
            motorCommandsArray.joint_cmd[motorId_].tor = 0;
        }
       // lastMotorControlMode = POSITION_MODE;

    } else if (motorControlMode == TORQUE_MODE) {
        Eigen::Matrix<float, 12, 1> motorCommandsShaped = motorCommands;
        motorCommandsShaped = jointDirection.cwiseProduct(motorCommandsShaped);
        
        for (int motorId = 0; motorId < NumMotor; motorId++) {
            int motorId_ = (motorId/3)%2 == 0? motorId+3: motorId-3;
            float tau = motorCommandsShaped[motorId];
         
            motorCommandsArray.joint_cmd[motorId_].pos = 0;
            motorCommandsArray.joint_cmd[motorId_].kp = 0;
            motorCommandsArray.joint_cmd[motorId_].vel = 0;
            motorCommandsArray.joint_cmd[motorId_].kd = 0;
            motorCommandsArray.joint_cmd[motorId_].tor = tau;
        }
       // lastMotorControlMode = TORQUE_MODE;
    
    } else if (motorControlMode == HYBRID_MODE) {
        Eigen::Matrix<float, 5, 12> motorCommandsShaped = motorCommands;
        lastMotorCommands.col(0) = motorCommandsShaped.row(0);

                    Eigen::Matrix<float, 12, 1> angles = motorCommandsShaped.row(POSITION).transpose();
                    motorCommandsShaped.row(POSITION)  = (jointDirection.cwiseProduct(angles) - jointOffset).transpose();
                    Eigen::Matrix<float, 12, 1> vels   = motorCommandsShaped.row(VELOCITY).transpose();
                    motorCommandsShaped.row(VELOCITY)  = jointDirection.cwiseProduct(vels).transpose();
                    Eigen::Matrix<float, 12, 1> tuas   = motorCommandsShaped.row(TORQUE).transpose();
                    motorCommandsShaped.row(TORQUE)    = jointDirection.cwiseProduct(tuas).transpose();
        
        for (int motorId = 0; motorId < NumMotor; motorId++) {
            int motorId_ = (motorId/3)%2 == 0? motorId+3: motorId-3;
            motorCommandsArray.joint_cmd[motorId_].pos = motorCommandsShaped(POSITION, motorId);
            motorCommandsArray.joint_cmd[motorId_].kp  = motorCommandsShaped(KP, motorId);
            motorCommandsArray.joint_cmd[motorId_].vel = motorCommandsShaped(VELOCITY, motorId);
            motorCommandsArray.joint_cmd[motorId_].kd  = motorCommandsShaped(KD, motorId);
            motorCommandsArray.joint_cmd[motorId_].tor = motorCommandsShaped(TORQUE, motorId);
        }
       // lastMotorControlMode = HYBRID_MODE;
    }

    SendCommand666(motorCommandsArray);
   
}



void qrRobotSim::ApplyAction(const std::vector<qrMotorCommand> &motorCommands,
                           MotorMode motorControlMode)
{

}


void qrRobotSim::Step(const Eigen::MatrixXf &action,
                    MotorMode motorControlMode){
    ReceiveObservation();
    ApplyAction(action, motorControlMode);
   // std::cout<<"qrRobotSim::Step"<<std::endl;
}

} // namespace Quadruped
