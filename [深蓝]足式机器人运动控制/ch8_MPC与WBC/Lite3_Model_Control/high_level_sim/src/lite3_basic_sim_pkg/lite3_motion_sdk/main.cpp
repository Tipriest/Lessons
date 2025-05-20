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

using namespace std;

int main(int argc, char* argv[]){
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
  robot_set_up_demo.GetInitData(robot_data->motor_state,0.000);       ///< Obtain all joint states once before each stage (action)
  
/********************************************************/
  int time_tick = 0;
  int loopcount=0;
  while(1){
    if (my_set_Timer.time_interrupt() == 1){                          ///< Time interrupt flag, return 1, cycle not reached, return 0, reach a cycle
      continue;
    }
     now_time = my_set_Timer.get_now_time(start_time);                ///< Get the current time

/*******A simple demo that stands up (for testing and can be deleted by yourself)*********/
    time_tick++;
    if(time_tick < 1000){
      robot_set_up_demo.PreStandUp(robot_joint_cmd,now_time,*robot_data);///< Stand up and prepare for action
    } 
    if(time_tick == 1000){
      robot_set_up_demo.GetInitData(robot_data->motor_state,now_time);///< Obtain all joint states once before each stage (action)
    }
    if(time_tick >= 1000 ){
      robot_set_up_demo.StandUp(robot_joint_cmd,now_time,*robot_data);///< Full stand up
    }
    if(time_tick >= 10000){
      send2robot_cmd->control_get(1);                                 ///< Return the control right, input: 1. Original algorithm control of the robot 2. SDK control PS: over 5ms, no data set sent_ Send (cmd), you will lose control, you need to resend to obtain control
    }
/*********A simple demo that stands up (for testing and can be deleted by yourself)*******/
     send2robot_cmd->set_send(robot_joint_cmd);               
    // send2robot_cmd->set_send(robot_joint_cmd);
    loopcount++;
    cout << robot_joint_cmd.fr_leg[2].tor<<"  "<<loopcount << endl;
  }
  return 0;
} 