
#include "sim_fun.h"
#include "robot_types.h"
#include <vector>
using namespace std;

void UpdateDate(const vector<double> &joint_angle,const vector<double> & joint_vel,RobotState &RobotState_sim)
{
         for(int i=0;i<3;i++){
        RobotState_sim.motor_state.fl_leg[i].pos=joint_angle[i];
        RobotState_sim.motor_state.fr_leg[i].pos=joint_angle[3+i];
        RobotState_sim.motor_state.hl_leg[i].pos=joint_angle[6+i];
        RobotState_sim.motor_state.hr_leg[i].pos=joint_angle[9+i];

        RobotState_sim.motor_state.joint_data[i  ].pos=joint_angle[i];
        RobotState_sim.motor_state.joint_data[3+i].pos=joint_angle[3+i];
        RobotState_sim.motor_state.joint_data[6+i].pos=joint_angle[6+i];
        RobotState_sim.motor_state.joint_data[9+i].pos=joint_angle[9+i];

        RobotState_sim.motor_state.joint_data[i  ].vel=joint_vel[i];
        RobotState_sim.motor_state.joint_data[3+i].vel=joint_vel[3+i];
        RobotState_sim.motor_state.joint_data[6+i].vel=joint_vel[6+i];
        RobotState_sim.motor_state.joint_data[9+i].vel=joint_vel[9+i];
        }
        
}