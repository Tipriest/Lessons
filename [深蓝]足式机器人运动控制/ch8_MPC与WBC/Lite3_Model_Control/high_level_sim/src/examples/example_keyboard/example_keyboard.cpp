#include <thread>

#include "quadruped/ros/qr_telekeyboard.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_control");
    ros::NodeHandle nh;
    qrTeleKeyboard *keyboard = new qrTeleKeyboard(nh);
    std::cout << "---------Keyboard start receving---------" << std::endl;
    std::cout << "You can use:\n"
    // "'k'             to enable/disable control\n"
    "'l'             to torque-stance\n"
    "'j'             to change the gait\n"
    "'u'             to position-stand up/down\n"
    "'i'             to exit after sit down\n\n"

    "'w' 'a' 's' 'd' to control the robot's movement\n"
    "'q' 'e'         to control the robot's rotation\n\n"

    "'Ctrl+C'        to exit keyboard_control" << std::endl;
    std::thread keyboardTh(&qrTeleKeyboard::run, keyboard);
    std::thread keyboardTh_default(&qrTeleKeyboard::run_default, keyboard);
    keyboardTh.join();
    keyboardTh_default.join();
    ros::shutdown();
    return 0;
    
}