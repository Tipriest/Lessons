#ifndef CUSTOM_TYPES_H_
#define CUSTOM_TYPES_H_

#include "common_types.h"

namespace types{
    enum RobotType{
        Lite3,
        X30,
    };

    enum RemoteCommandType{
        kKeyBoard = 0,
        kRetroid,
        kSkydroid,
    };

    enum RobotMotionState{
        WaitingForStand = 0,
        StandingUp      = 1,
        JointDamping    = 2,
        ForceStand      = 3,
        SitDown         = 4,

        RLControlMode   = 6,
    };

    enum StateName{
        kInvalid      = -1,
        kIdle         = 0,
        kStandUp      = 1,
        kJointDamping = 2,
        kForceStand   = 3,
        kSitDown      = 4,

        kRLControl    = 6,
    };
    

    inline std::string GetAbsPath(){
        char buffer[PATH_MAX];
        if(getcwd(buffer, sizeof(buffer)) != NULL){
            return std::string(buffer);
        }
        return "";
    }
};

#endif