/**
 * @file robot_types.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-09-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "parse_cmd.h"
#include "command_list.h"
using namespace std;
extern int data_is_rec;
CommandList command_list_,upload_command_list_;
double joint_data[12];
void ParseCMD::work()
{
	// int count = 0;

  EthCommand c;
  CommandMessage cm;
  Command cmd_test;

  UDPServer udpServer;
  timespec test_time;


  RobotCmd test_cmd;

  udpServer.onRawMessageReceived = [&](const char* message, int length, string ipv4, uint16_t port) {
    clock_gettime(1,&test_time);
  // if(manager_->command_list_.get_command_front(c)>0){
  //   HeatData* heat_data = 0;
    memcpy(&cm,message,sizeof(cm));
    Command nc(cm.command.code,cm.command.paramters_size, cm.data_buffer);
    if(cm.command.type == command_type::CommandType::kMessValues){
      switch (cm.command.code){
        case ROBOT_STATE_CMD:
          clock_gettime(1,&test_time);
          memcpy(&state_rec, cm.data_buffer, sizeof(state_rec));
          break;
      default:
        break;
      }
    }
    if(message==nullptr||length==0){
        data_is_rec=0;
    }
  };
    // Bind the server to a port.
    udpServer.Bind(LOCAL_PORT, [](int errorCode, string errorMessage) {
    // BINDING FAILED:
    cout << errorCode << " : " << errorMessage << endl;
  });


	while (1){
    sleep(1);
	}
}

void ParseCMD::startWork()
{
  std::thread work_thread(std::bind(&ParseCMD::work, this));
	work_thread.detach();
}
RobotState& ParseCMD::get_recv(){
  return state_rec;
}
