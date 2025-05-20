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

#include "robots/qr_robot_lite3_sim.h"


#include <std_msgs/Float64.h>
#include <vector>
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"

#include <hardware_interface/joint_state_interface.h>
//#include <hardware_interface/effort_joint_interface.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>


namespace Quadruped {

qrRobotLite3Sim::qrRobotLite3Sim(ros::NodeHandle &nhIn, ros::NodeHandle &privateNhIn, std::string configFilePath):
    nh(nhIn),
    privateNh(privateNhIn)
{
    baseOrientation << 1.f, 0.f, 0.f, 0.f;
    baseRollPitchYaw << 0.f, 0.f, 0.f;
    baseRollPitchYawRate << 0.f, 0.f, 0.f;
    motorVelocities = Eigen::Matrix<float, 12, 1>::Zero();
    footForce << 0.f, 0.f, 0.f, 0.f;
    footContact << 1, 1, 1, 1;
    effort_last[12]={0};
    this->configFilePath = configFilePath;
    robotConfig = YAML::LoadFile(configFilePath);

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

    imuSub = nh.subscribe("/trunk_imu", 1, &qrRobotLite3Sim::ImuCallback, this);

    joint_sub= nh.subscribe("/lite3_gazebo/joint_states", 1, &qrRobotLite3Sim::jointStateCallback_,this);

    footForceSub[0] = nh.subscribe("/lite3/FR_foot_contact/contact_force", 1, &qrRobotLite3Sim::FRfootCallback, this);
    footForceSub[1] = nh.subscribe("/lite3/FL_foot_contact/contact_force", 1, &qrRobotLite3Sim::FLfootCallback, this);
    footForceSub[2] = nh.subscribe("/lite3/RR_foot_contact/contact_force", 1, &qrRobotLite3Sim::RRfootCallback, this);
    footForceSub[3] = nh.subscribe("/lite3/RL_foot_contact/contact_force", 1, &qrRobotLite3Sim::RLfootCallback, this);



         std::vector<std::string>  joint_command_topics= {
                                              
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
 std::vector<std::string>  joint_name111= {
                                              
                                              "FL_HipX",
                                              "FL_HipY",
                                              "FL_Knee",
                                              
                                              "FR_HipX",
                                              "FR_HipY",
                                              "FR_Knee",

                                              "HL_HipX",
                                              "HL_HipY",
                                              "HL_Knee",

                                              "HR_HipX",
                                              "HR_HipY",
                                              "HR_Knee"
                                              
                                               };

/*       std::vector<hardware_interface::JointHandle> effort_handles;

  for (size_t i = 0; i < joint_name111.size(); ++i) 
  {
    effort_handles.push_back(hardware_interface::JointHandle(joint_state_interface.getHandle(joint_name111[i]), &effort_command111[i]));
  }
       */




      for(int i = 0; i < joint_command_topics.size(); i++){
          jointCmdPub[i] = nh.advertise<std_msgs::Float64>(joint_command_topics[i],1);
      }


/* const std::vector<std::string> GazeboSpawner::controller_list = {
  "joint_states_controller",

  "FL_HipX", "FL_HipY", "FL_Knee",
  "FR_HipX", "FR_HipY", "FR_Knee",
  "HL_HipX", "HL_HipY", "HL_Knee",
  "HR_HipX", "HR_HipY", "HR_Knee"
  
  };//1 */
      //  joint = robot->getHandle(joint_name);



    // ros::spinOnce();
    usleep(300000); // must wait 300ms, to get first state

    yawOffset = 0;//lowState.imu.rpy[2]; // todo
    std::cout << "yawOffset: " << yawOffset << std::endl;

    // timeStep = 1.0 / robotConfig["controller_params"]["freq"].as<int>();
    this->ResetTimer();
    lastResetTime = GetTimeSinceReset();
    initComplete = true;
    std::cout << "-------Lite3Sim init Complete-------" << std::endl;
}


bool qrRobotLite3Sim::BuildDynamicModel()
{
    // we assume the cheetah's body (not including rotors) can be modeled as a
    // uniformly distributed box.
    std::vector<float> bodySize = robotConfig["robot_params"]["body_size"].as<std::vector<float>>(); // Length, Width, Height
    Vec3<float> bodyDims(bodySize[0], bodySize[1], bodySize[2]);

    // locations
    Vec3<float> _abadRotorLocation = {0.14f, 0.047f, 0.f};
    Vec3<float> _abadLocation = {0.1805f, 0.047f, 0.f};
    Vec3<float> _hipLocation = Vec3<float>(0, hipLength, 0);
    Vec3<float> _hipRotorLocation = Vec3<float>(0, 0.04, 0);
    Vec3<float> _kneeLocation = Vec3<float>(0, 0, -upperLegLength);
    Vec3<float> _kneeRotorLocation = Vec3<float>(0, 0, 0);

    float scale_ = 1e-2;
    // rotor inertia if the rotor is oriented so it spins around the z-axis
    Mat3<float> rotorRotationalInertiaZ;
    rotorRotationalInertiaZ << 33, 0, 0,
                               0, 33, 0,
                               0, 0, 63;
    rotorRotationalInertiaZ.setIdentity();
    rotorRotationalInertiaZ = scale_*1e-6 * rotorRotationalInertiaZ;

    Mat3<float> RY = coordinateRotation<float>(CoordinateAxis::Y, M_PI / 2);
    Mat3<float> RX = coordinateRotation<float>(CoordinateAxis::X, M_PI / 2);
    Mat3<float> rotorRotationalInertiaX = RY * rotorRotationalInertiaZ * RY.transpose();
    Mat3<float> rotorRotationalInertiaY = RX * rotorRotationalInertiaZ * RX.transpose();

    // spatial inertias
    Mat3<float> abadRotationalInertia;
    abadRotationalInertia << 469.2, -9.4, -0.342,
                             -9.4, 807.5, -0.466,
                             -0.342, -0.466,  552.9;
    // abadRotationalInertia.setIdentity();
    abadRotationalInertia = abadRotationalInertia * 1e-6;
    // Vec3<float> abadCOM(0, 0.036, 0);  // mini-cheetah
    Vec3<float> abadCOM(-0.0033, 0, 0);
    SpatialInertia<float> abadInertia(0.696, abadCOM, abadRotationalInertia);

    Mat3<float> hipRotationalInertia;
    hipRotationalInertia << 5529, 4.825, 343.9,
                            4.825, 5139.3, 22.4,
                            343.9, 22.4, 1367.8;
    // hipRotationalInertia.setIdentity();
    hipRotationalInertia = hipRotationalInertia * 1e-6;
    // Vec3<float> hipCOM(0, 0.016, -0.02);
    Vec3<float> hipCOM(-0.003237, -0.022327, -0.027326); // left, for right filp y-axis value.
    SpatialInertia<float> hipInertia(1.013, hipCOM, hipRotationalInertia);
    std::cout << "hipInertia -----" <<std::endl;
    std::cout << hipInertia.getInertiaTensor() << std::endl;
    std::cout << hipInertia.flipAlongAxis(CoordinateAxis::Y).getInertiaTensor() << std::endl;
    std::cout << "----- hipInertia " <<std::endl;


    Mat3<float> kneeRotationalInertia, kneeRotationalInertiaRotated;
    kneeRotationalInertiaRotated << 2998, 0,   -141.2,
                                    0,    3014, 0,
                                    -141.2, 0,   32.4;
    // kneeRotationalInertiaRotated.setIdentity();
    kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
    kneeRotationalInertia = kneeRotationalInertiaRotated;//RY * kneeRotationalInertiaRotated * RY.transpose();
    // Vec3<float> kneeCOM(0, 0, -0.061);
    Vec3<float> kneeCOM(0.006435, 0, -0.107);
    SpatialInertia<float> kneeInertia(0.166, kneeCOM, kneeRotationalInertia);

    Vec3<float> rotorCOM(0, 0, 0);
    float rotorMass = 1e-8; //0.055
    SpatialInertia<float> rotorInertiaX(rotorMass, rotorCOM, rotorRotationalInertiaX);
    SpatialInertia<float> rotorInertiaY(rotorMass, rotorCOM, rotorRotationalInertiaY);

    Mat3<float> bodyRotationalInertia;
    bodyRotationalInertia << 15853, 0, 0,
                             0, 37799, 0,
                             0, 0, 45654;
    bodyRotationalInertia = bodyRotationalInertia * 1e-6;
    Vec3<float> bodyCOM(0, 0, 0);
    // Vec3<float> bodyCOM(0, 0.004, -0.0005);
    SpatialInertia<float> bodyInertia(6, bodyCOM, bodyRotationalInertia);

    model.addBase(bodyInertia);
    // add contact for the cheetah's body
    model.addGroundContactBoxPoints(5, bodyDims);

    const int baseID = 5;
    int bodyID = baseID;
    float sideSign = -1;

    Mat3<float> I3 = Mat3<float>::Identity();

    auto& abadRotorInertia = rotorInertiaX;
    float abadGearRatio = 1; // 6
    auto& hipRotorInertia = rotorInertiaY;
    float hipGearRatio = 1; // 6
    auto& kneeRotorInertia = rotorInertiaY;
    float kneeGearRatio = 1; // 9.33
    float kneeLinkY_offset = 0.004;

    // loop over 4 legs
    for (int legID = 0; legID < 4; legID++) {
        // Ab/Ad joint
        //  int addBody(const SpatialInertia<T>& inertia, const SpatialInertia<T>&
        //  rotorInertia, T gearRatio,
        //              int parent, JointType jointType, CoordinateAxis jointAxis,
        //              const Mat6<T>& Xtree, const Mat6<T>& Xrot);
        bodyID++;
        Mat6<float> xtreeAbad = createSXform(I3, WithLegSigns(_abadLocation, legID));
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
                          kneeRotorInertia.flipAlongAxis(CoordinateAxis::Y),
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

    bool test_fb = false;
    if (test_fb) {
        FBModelState<float> fb;
        // for (size_t i(0); i < 3; ++i) {
        // _state.bodyVelocity[i] = omegaBody[i]; // in body frame
        // _state.bodyVelocity[i + 3] = vBody[i];

        //     for (size_t leg(0); leg < 4; ++leg) {
        //         _state.q[3 * leg + i] = q[3 * leg + i];
        //         _state.qd[3 * leg + i] = dq[3 * leg + i];
        //         _full_config[3 * leg + i + 6] = _state.q[3 * leg + i];
        //     }
        // }
        printf("339\n");
        fb.bodyOrientation << 1,0,0,0;//0.896127, 0.365452,0.246447,-0.0516205;
        // fb.bodyPosition << 0.00437649, 0.000217693, 0.285963;
        fb.bodyVelocity <<  3,3,3, 0.2, 0.1, 0.1;
        fb.bodyPosition.setZero();
        printf("343\n");
        fb.q.resize(12,1);
        fb.q.setZero();
        fb.q << 0.2, 0, -0.2,
                0.2, 0, -0.2, // 0, 0.7, 0,
                0, 0.3, 0.5, // 0, 0.8, 0
                0, 0.3, 0.5,
        fb.qd.resize(12, 1);
        fb.qd.setZero();
        // fb.qd << 0.2, 0, 0,
        //             0.1, 0, 0.,
        //             0, -0.3, 0.6,
        //             0, 0.3, 1;

        printf("346\n");
        model.setState(fb);
        printf("348\n");
        model.forwardKinematics();
        model.massMatrix();
        Eigen::MatrixXf A;
        Eigen::Matrix<float,18,1> dq;
        dq << fb.bodyVelocity, fb.qd;
        // model.generalizedGravityForce();
        // model.generalizedCoriolisForce();
        A = model.getMassMatrix();
        for (int i=0; i <18 ; ++i) {
            for (int j=0; j<18; ++j) {
                if (A(i,j)<1e-6) A(i,j) = 0;
            }
        }
        std::cout << "A = \n" << A << std::endl;
        float energy = 0.5*dq.dot(A*dq);
        std::cout << "energy = " << energy << std::endl;

        // model.getPosition(8);
        printf("351\n");
        throw std::domain_error("finished!!!!");
    }
    return true;
}


void qrRobotLite3Sim::ImuCallback(const sensor_msgs::Imu &msg)
{


    Eigen::Matrix<float, 4, 1> quaternion = {static_cast<float>(msg.orientation.w),
                                             static_cast<float>(msg.orientation.x),
                                             static_cast<float>(msg.orientation.y),
                                             static_cast<float>(msg.orientation.z)};
    Eigen::Matrix<float, 3, 1> rpy = robotics::math::quatToRPY(quaternion);

    robot_state.imu.angle_roll  =rpy[0]*180.0/M_PI;
    robot_state.imu.angle_pitch =rpy[1]*180.0/M_PI;
    robot_state.imu.angle_yaw   =rpy[2]*180.0/M_PI;

    robot_state.imu.angular_velocity_roll=msg.angular_velocity.x;;
    robot_state.imu.angular_velocity_pitch=msg.angular_velocity.y;
    robot_state.imu.angular_velocity_yaw=msg.angular_velocity.z;
   
    robot_state.imu.acc_x=msg.linear_acceleration.x;
    robot_state.imu.acc_y=msg.linear_acceleration.y;
    robot_state.imu.acc_z=msg.linear_acceleration.z;

   // std::cout<<"ImuCallback is working"<<std::endl;


}

/*  bool qrRobotLite3Sim::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
    {  
         for(int i=0;i<12;i++)
  {
        joint[i] = robot->getHandle(joint_name111[i]);

  }


    } */

 void qrRobotLite3Sim::jointStateCallback_(const sensor_msgs::JointState::ConstPtr& msg){

  for(int i=0;i<12;i++)
  {
   robot_state.motor_state.joint_data[i].pos=msg->position[i];
   robot_state.motor_state.joint_data[i].vel=msg->velocity[i];
  // robot_state.motor_state.joint_data[i].tor=effort_handles[i].getEffort();

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


   Eigen::Matrix<float, 12, 1>pos_temp,vel_temp,tor_temp;
   
   for(int i=0;i<12;i++)
    {
  //  pos_temp[i]=robot_state.motor_state.joint_data[i].pos;
   // vel_temp[i]=robot_state.motor_state.joint_data[i].vel;
   // tor_temp[i]=robot_state.motor_state.joint_data[i].tor;
    }

   // pos_temp = JointPosFilter.CalculateAverage(pos_temp);
 //   vel_temp = JointVelFilter.CalculateAverage(vel_temp);
   // tor_temp = JointTorFilter.CalculateAverage(tor_temp);

   for(int i=0;i<12;i++)
    {
    //robot_state.motor_state.joint_data[i].pos=pos_temp[i];
  //  robot_state.motor_state.joint_data[i].vel=vel_temp[i];
   // robot_state.motor_state.joint_data[i].tor=tor_temp[i];
    }


//std::cout<<"jointStateCallback_  is working"<<std::endl;

std::cout<<" msg->velocity[i]= "<<msg->velocity[0]<<"  "<<msg->velocity[1]<<"  "<<msg->velocity[2]<<"  "<<msg->velocity[3]<<"  "<<msg->velocity[4]<<"  "<<msg->velocity[5]<<" "<<msg->velocity[6]<<"  "<<msg->velocity[7]<<"  "<<msg->velocity[8]<<"  "<<msg->velocity[9]<<"  "<<msg->velocity[10]<<"  "<<msg->velocity[11]<<std::endl;

} 



void qrRobotLite3Sim::FRfootCallback(const geometry_msgs::WrenchStamped &msg)
{
    foot_contact_force_z[0] = msg.wrench.force.z;
}


void qrRobotLite3Sim::FLfootCallback(const geometry_msgs::WrenchStamped &msg)
{
    foot_contact_force_z[1] = msg.wrench.force.z;
}


void qrRobotLite3Sim::RRfootCallback(const geometry_msgs::WrenchStamped &msg)
{
    foot_contact_force_z[2] = msg.wrench.force.z;
}


void qrRobotLite3Sim::RLfootCallback(const geometry_msgs::WrenchStamped &msg)
{
    foot_contact_force_z[3] = msg.wrench.force.z;
}


void qrRobotLite3Sim::SendCommand(const std::array<float, 60> motorcmd)
{
   for (int motor_id = 0; motor_id < 12; motor_id++) 
   {
        int motorId_ = (motor_id/3)%2 == 0? motor_id+3: motor_id-3;
        robot_cmd.joint_cmd[motorId_].pos = motorcmd[5*motor_id+0];
        robot_cmd.joint_cmd[motorId_].kp  = motorcmd[5*motor_id+1];
        robot_cmd.joint_cmd[motorId_].vel = motorcmd[5*motor_id+2];
        robot_cmd.joint_cmd[motorId_].kd  = motorcmd[5*motor_id+3];
        robot_cmd.joint_cmd[motorId_].tor = motorcmd[5*motor_id+4];
    }

  


    for (int m = 0; m < 12; m++) {
         std_msgs::Float64 command_message;
         
         command_message.data  =1.2*robot_cmd.joint_cmd[m].kp*(robot_cmd.joint_cmd[m].pos-robot_state.motor_state.joint_data[m].pos);
         command_message.data +=1.5*robot_cmd.joint_cmd[m].kd*(robot_cmd.joint_cmd[m].vel-robot_state.motor_state.joint_data[m].vel);
         command_message.data +=5.0*robot_cmd.joint_cmd[m].tor+.0*robot_state.motor_state.joint_data[m].tor; 
         
     //    std::cout<<"command_message.data= "<<command_message.data<<"  "<<robot_cmd.joint_cmd[m].kp<<"  "<<robot_cmd.joint_cmd[m].kd<<"  "<<robot_cmd.joint_cmd[m].pos<<"  "<< robot_state.motor_state.joint_data[m].pos<<std::endl;
         jointCmdPub[m].publish(command_message);
    }


}


void qrRobotLite3Sim::ReceiveObservation()
{






    baseAccInBaseFrame << robot_state.imu.acc_x,
                          robot_state.imu.acc_y,
                          robot_state.imu.acc_z;
    stateDataFlow.baseLinearAcceleration = accFilter.CalculateAverage(baseAccInBaseFrame);

    std::array<float, 3> rpy;
  rpy[0]=robot_state.imu.angle_roll/57.3;
  rpy[1]=robot_state.imu.angle_pitch/57.3;
  rpy[2]=robot_state.imu.angle_yaw/57.3;

    // calibrated
    float calibratedYaw = rpy[2] - yawOffset;
    if (calibratedYaw >= M_PI) {
        calibratedYaw -= M_2PI;
    } else if (calibratedYaw <= -M_PI) {
        calibratedYaw += M_2PI;
    }
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
    gyro[0]=robot_state.imu.angular_velocity_roll;
    gyro[1]=robot_state.imu.angular_velocity_pitch;
    gyro[2]=robot_state.imu.angular_velocity_yaw;

    Vec3<float> gyroVec(gyro[0], gyro[1], gyro[2]);
    baseRollPitchYawRate = gyroFilter.CalculateAverage(gyroVec);

    baseOrientation = robotics::math::rpyToQuat(baseRollPitchYaw);

    for (int motorId = 0; motorId < NumMotor; ++motorId) {
        int motorId_ = (motorId/3)%2 == 0? motorId+3: motorId-3;

        motorAngles[motorId]     = robot_state.motor_state.joint_data[motorId_].pos;
        motorVelocities[motorId] = robot_state.motor_state.joint_data[motorId_].vel;
      //  motorddq[motorId] = state.motorState[motorId].ddq;
        motortorque[motorId]     = robot_state.motor_state.joint_data[motorId_].tor;
    }

    motorAngles     = jointDirection.cwiseProduct(motorAngles + jointOffset);
    motorVelocities = jointDirection.cwiseProduct(motorVelocities);
   // motorddq    = jointDirection.cwiseProduct(motorddq);
    motortorque     = jointDirection.cwiseProduct(motortorque);

    footForce << foot_contact_force_z[0], foot_contact_force_z[1], foot_contact_force_z[2], foot_contact_force_z[3];

    for (int footId = 0; footId < NumLeg; footId++) {
        if (footForce[footId] >= 5) 
        {
            footContact[footId] = true;
        } else 
        {
            footContact[footId] = false;
        }
    }

    UpdateDataFlow();
}


void qrRobotLite3Sim::ApplyAction(const Eigen::MatrixXf &motorCommands, MotorMode motorControlMode)
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

   std::cout<<" POSITION_MODE "<<std::endl;

  } else if (motorControlMode == TORQUE_MODE) {
      Eigen::Matrix<float, 12, 1> motorCommandsShaped = motorCommands;
      motorCommandsShaped = jointDirection.cwiseProduct(motorCommandsShaped);
      for (int motorId = 0; motorId < NumMotor; ++motorId) {

          motorCommandsArray[motorId * 5    ] = 0;
          motorCommandsArray[motorId * 5 + 1] = 0;
          motorCommandsArray[motorId * 5 + 2] = 0;
          motorCommandsArray[motorId * 5 + 3] = 0;
          motorCommandsArray[motorId * 5 + 4] = motorCommandsShaped[motorId];

      }

    std::cout<<" TORQUE_MODE "<<std::endl;

  } else if (motorControlMode == HYBRID_MODE) {
      Eigen::Matrix<float, 5, 12> motorCommandsShaped = motorCommands;
      Eigen::Matrix<float, 12, 1> angles = motorCommandsShaped.row(POSITION).transpose();
      motorCommandsShaped.row(POSITION) = (jointDirection.cwiseProduct(angles) - jointOffset).transpose();
      Eigen::Matrix<float, 12, 1> vels = motorCommandsShaped.row(VELOCITY).transpose();
      motorCommandsShaped.row(VELOCITY) = jointDirection.cwiseProduct(vels).transpose();
      Eigen::Matrix<float, 12, 1> tuas = motorCommandsShaped.row(TORQUE).transpose();
      motorCommandsShaped.row(TORQUE) = jointDirection.cwiseProduct(tuas).transpose();

      for (int motorId = 0; motorId < NumMotor; ++motorId) {
          motorCommandsArray[motorId * 5]     = motorCommandsShaped(POSITION, motorId);
          motorCommandsArray[motorId * 5 + 1] = motorCommandsShaped(KP, motorId);
          motorCommandsArray[motorId * 5 + 2] = motorCommandsShaped(VELOCITY, motorId);
          motorCommandsArray[motorId * 5 + 3] = motorCommandsShaped(KD, motorId);
          motorCommandsArray[motorId * 5 + 4] = motorCommandsShaped(TORQUE, motorId);
      }

    std::cout<<" HYBRID_MODE "<<motorCommandsShaped(KP, 1)<<"  "<<motorCommandsShaped(KD, 1)<<std::endl;

  }

  for (int index=0; index< motorCommandsArray.size(); index++) {
      if (isnan(motorCommandsArray[index])) 
      {
          motorCommandsArray[index] = 0.f;
          std::cout<<" motorCommandsArray  exist nan value "<<std::endl;
      }
  }
  SendCommand(motorCommandsArray);
}


void qrRobotLite3Sim::ApplyAction(const std::vector<qrMotorCommand> &motorCommands, MotorMode motorControlMode)
{
  std::array<float, 60> motorCommandsArray = {0};

  for (int motorId = 0; motorId < NumMotor; motorId++) {
      motorCommandsArray[motorId * 5] = motorCommands[motorId].p * jointDirection(motorId) - jointOffset(motorId);
      motorCommandsArray[motorId * 5 + 1] = motorCommands[motorId].Kp;
      motorCommandsArray[motorId * 5 + 2] = motorCommands[motorId].d * jointDirection(motorId);
      motorCommandsArray[motorId * 5 + 3] = motorCommands[motorId].Kd;
      motorCommandsArray[motorId * 5 + 4] = motorCommands[motorId].tua * jointDirection(motorId);
  }
  // robotInterface.SendCommand(motorCommandsArray);
}


void qrRobotLite3Sim::Step(const Eigen::MatrixXf &action, MotorMode motorControlMode)
{
    ReceiveObservation();
    ApplyAction(action, motorControlMode);
    std::cout<<"qrRobotLite3Sim "<<std::endl;
}

} // namespace Quadruped
