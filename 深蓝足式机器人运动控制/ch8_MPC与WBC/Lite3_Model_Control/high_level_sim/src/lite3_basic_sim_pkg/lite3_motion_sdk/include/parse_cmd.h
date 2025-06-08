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


#ifndef PARSE_CMD_H_
#define PARSE_CMD_H_

#include <iostream>
#include <cmath>
#include <stdint.h>
#include <array>
#include <thread>
#include <unistd.h>
#include <time.h>
#include <chrono>
#include <sys/timerfd.h>
#include <sys/epoll.h>

#include "robot_types.h"
#include "udp_server.hpp"
#define LOCAL_PORT 43897 /**< 43897 Local Port Number.*/

#define ROBOT_STATE_CMD 0x0906 /**< Command code for receiving robot data.*/

/**
 * @brief A class used to receive robot data.
 */
class ParseCMD{
  private:
    RobotState state_rec; /**< Used to save received data.*/
  public:
   /**
    * @brief Create the accepted thread using the work()
    */
    void startWork(); /**< Create the accepted thread using the work().*/

   /**
    * @brief Receive robot data and save it
    */
    void work();

    RobotState& get_recv(); /**< Save the obtained data in st.*/
};

#endif  // PARSE_CMD_H_