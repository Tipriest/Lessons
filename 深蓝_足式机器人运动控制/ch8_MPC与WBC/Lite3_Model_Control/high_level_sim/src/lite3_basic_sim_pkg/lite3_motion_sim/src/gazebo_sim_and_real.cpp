/**
 * @file main.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-09-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "udp_socket.hpp"
#include "udp_server.hpp"
#include "command_list.h"
#include "parse_cmd.h"
#include "send_to_robot.h"
#include "motion_example.h"

#include <iostream>
#include <time.h>
#include <string.h>

#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>
#include <std_msgs/Float64.h>
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"


#include "sim_fun.h"

using namespace std;

 int data_is_rec=1;

 vector<double>  joint_angle__init_(12, 0.0);
 vector<double>  joint_angle_(12, 0.0);
 vector<double>  joint_vel_(12, 0.0);

void jointStateCallback_(const sensor_msgs::JointState::ConstPtr& msg){
   joint_angle_ = msg->position;
   joint_vel_   = msg->velocity;
}

int main(int argc, char* argv[])
{
  double now_time,start_time;
  RobotCmd robot_joint_cmd;
  Time_Tool my_set_Timer;
  memset(&robot_joint_cmd, 0, sizeof(robot_joint_cmd));
  SendToRobot* send2robot_cmd = new SendToRobot;                      ///< Create send thread
  ParseCMD* robot_data_rec    = new ParseCMD;                         ///< Create a receive resolution thread 

  MotionExample robot_set_up_demo;                                    ///< Demos for testing can be deleted by yourself
  RobotState *robot_data = &robot_data_rec->get_recv();

  send2robot_cmd->init();
  robot_data_rec->startWork();
  my_set_Timer.time_init(1);                                          ///< Timer initialization, input: cycle; Unit: ms

  send2robot_cmd->robot_state_init();                                 ///< Return all joints to zero and gain control

  start_time = my_set_Timer.get_start_time();                         ///< Obtain time for algorithm usage
 // robot_set_up_demo.GetInitData(robot_data->motor_state,0.000);       ///< Obtain all joint states once before each stage (action)
  
  int time_tick = 0;

/********************************************************/

  int is_sim_mode=1; // 0: real_mode; 1: sim_mode

  RobotState  RobotState_sim;
  memset(&RobotState_sim, 0, sizeof(RobotState_sim));

  RobotState  *RobotState_temp;

  int loopcount = 0;

  ros::init(argc, argv, "gazebo_lite_motion_sim_and_real_node");
  ros::NodeHandle node_handle;

  ros::Subscriber joint_sub = node_handle.subscribe("/lite3_gazebo/joint_states", 1, jointStateCallback_);
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
                                              "/lite3_gazebo/HR_Knee/command" };
      
      vector<ros::Publisher> joint_cmd_publishers;
      for(int i = 0; i < joint_command_topics.size(); i++){
          ros::Publisher new_publisher = node_handle.advertise<std_msgs::Float64>(joint_command_topics[i],1);
          joint_cmd_publishers.push_back(new_publisher);
      }

  ros::Duration(2.0).sleep();

  if(is_sim_mode){

      ros::spinOnce();

      UpdateDate(joint_angle_,joint_vel_,RobotState_sim);

      RobotState_temp=  &RobotState_sim;
      // robot_set_up_demo.GetInitData(RobotState_temp->motor_state,0.000);
    
  }
  else{
      RobotState_temp=robot_data;
      // robot_set_up_demo.GetInitData(RobotState_temp->motor_state,0.000);       ///< Obtain all joint states once before each stage (action)
  }
  now_time = my_set_Timer.get_now_time(start_time); 
  robot_set_up_demo.GetInitData(RobotState_temp->motor_state, now_time);
  while(1){

    if(!is_sim_mode){
        if(data_is_rec==0){
             cout<<"  No data from the robot was received!!!!!! "<<endl;
             continue;
      } 
    }

    if(!ros::ok()){
         break;
    }

    if (my_set_Timer.time_interrupt() == 1){                          ///< Time interrupt flag, return 1, cycle not reached, return 0, reach a cycle
      continue;
    }
    now_time = my_set_Timer.get_now_time(start_time);  
    
                ///< Get the current time
/*******A simple demo that stands up (for testing and can be deleted by yourself)*********/
    time_tick++;
    
if(is_sim_mode)
{
    ros::spinOnce();
    UpdateDate(joint_angle_,joint_vel_,RobotState_sim);
}

    if(time_tick < 3000){
      robot_set_up_demo.PreStandUp(robot_joint_cmd,now_time,*RobotState_temp);///< Stand up and prepare for action
      // cout << loopcount<<"  "<< robot_joint_cmd.joint_cmd[0].pos << std::endl;
    } 
    if(time_tick == 3000){
      robot_set_up_demo.GetInitData(RobotState_temp->motor_state, now_time);///< Obtain all joint states once before each stage (action)
    }
    if(time_tick >= 3000 ){
      robot_set_up_demo.StandUp(robot_joint_cmd,now_time,*RobotState_temp);///< Full stand up
    }
    if(time_tick >= 30000){
      if(!is_sim_mode){
      send2robot_cmd->control_get(1);                                 ///< Return the control right, input: 1. Original algorithm control of the robot 2. SDK control PS: over 5ms, no data set sent_ Send (cmd), you will lose control, you need to resend to obtain control
      }
    }

/*********A simple demo that stands up (for testing and can be deleted by yourself)*******/
    if(!is_sim_mode){
        send2robot_cmd->set_send(robot_joint_cmd);               
    }
    else{
        for(int j = 0; j < joint_command_topics.size(); j++){
            std_msgs::Float64 command_message;
           command_message.data = 60*(robot_joint_cmd.joint_cmd[j].pos - RobotState_temp->motor_state.joint_data[j].pos)+0.7*(robot_joint_cmd.joint_cmd[j].vel-RobotState_temp->motor_state.joint_data[j].vel); //send pos to robot_position_controllers_lite3.yaml, then obtain torque (PD), then torque is sent to robot in gazebo.
            joint_cmd_publishers[j].publish(command_message);
          }
    }

  loopcount++;
  cout << loopcount<<"  "<<robot_joint_cmd.joint_cmd[0].pos<<"  "<<robot_joint_cmd.joint_cmd[1].pos<<"  "<<robot_joint_cmd.joint_cmd[2].pos<<"  "<<RobotState_temp->motor_state.joint_data[0].pos<<"  "<<RobotState_temp->motor_state.joint_data[0].vel << "   8" << endl;
   // rate_timer.sleep();
   // cout << robot_joint_cmd.fr_leg[2].tor<<"  "<<loopcount << endl;
  }
  return 0;
} 


