"""
 * @file kinematic.py
 * @brief simulation in pybullet
 * @author mazunwang
 * @version 1.0
 * @date 2024-10-15
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
"""


import numpy as np
import time
from math import *
from lite3_config import Lite3Config

urdfFileName = Lite3Config.urdf_path
PI = np.pi

def rotX(roll):
    res = np.array([[1, 0, 0], [0, cos(roll), -sin(roll)], [0, sin(roll), cos(roll)]])
    return res

def rotY(pitch):
    res = np.array([[cos(pitch), 0, sin(pitch)], [0, 1, 0], [-sin(pitch), 0, cos(pitch)]])
    return res

def rotZ(yaw):
    res = np.array([[cos(yaw), -sin(yaw), 0], [sin(yaw), cos(yaw), 0], [0, 0, 1]])
    return res

def legForwardKinematic(angle, leg):
    """calculate lite3 leg forward kinematic

    Args:
        angle: joint positon of 3 joints in one leg
        leg (int): 0:FL, 1:FR, 2:HL, 3:HR
        
    Returns: 
        [vec3]:foot position in base frame
    """
    lHip = Lite3Config.hip_len
    lThigh = Lite3Config.thigh_len
    lShank = Lite3Config.shank_len
    bodyLenX = Lite3Config.body_len_x
    bodyLenY = Lite3Config.body_len_y
    bodyLenZ = Lite3Config.body_len_z

    bodyVec = np.array([bodyLenX, bodyLenY, bodyLenZ]).reshape(3, 1)
    hipVec = np.array([0, lHip, 0])
    if leg%2==1:
        hipVec = np.array([0, -lHip, 0])
        bodyVec[1, 0] = -bodyLenY
    if leg/2>=1:
        bodyVec[0, 0] = -bodyLenX
    hipVec = hipVec.reshape((3, 1))
    thighVec = np.array([0, 0, -lThigh]).reshape((3, 1))
    shankVec = np.array([0, 0, -lShank]).reshape((3, 1))
    '''
            you need to calculate the leg position in hipx frame 
        and return a numpy array in shape (3, 1)
    '''
    # 左边的腿
    if leg % 2 == 0:
        l1 = lHip
    else:
        l1 = -lHip
    l2 = - lThigh
    l3 = - lShank
    theta1 = -angle[0]
    theta2 = -angle[1]
    theta3 = -angle[2]
    
    footPos = np.array([0, 0, 0.1]).reshape((3, 1))
    footPos[0] = l3 * sin(theta2 + theta3) + l2 * sin(theta2)
    footPos[1] = -l3 * sin(theta1)*cos(theta2+theta3) + l1 * cos(theta1) - l2 * cos(theta2) * sin(theta1)
    footPos[2] = l3 * cos(theta1)*cos(theta2+theta3) + l1 * sin(theta1) + l2 * cos(theta1) * cos(theta2)

    # print(f"l1 = {l1}, l2 = {l2}, l3 = {l3}")
    # print(f"theta1 = {theta1}, theta2 = {theta2}, theta3 = {theta3}")
    # print(f"footPos[0] = {footPos[0]}, footPos[1] = {footPos[1]}, footPos[2] = {footPos[2]}")
    
    return footPos+bodyVec

def legInverseKinematic(pos, leg):
    """calculate lite3 leg inverse kinematic

    Args:
        pos (vec3): foot position in base frame
        leg (int) : 0:FL, 1:FR, 2:HL, 3:HR
    
    Returns: 
        [vec3]:joint angle
    """
    lHip = Lite3Config.hip_len
    lThigh = Lite3Config.thigh_len
    lShank = Lite3Config.shank_len
    bodyLenX = Lite3Config.body_len_x
    bodyLenY = Lite3Config.body_len_y
    bodyLenZ = Lite3Config.body_len_z

    bodyVec = np.array([bodyLenX, bodyLenY, bodyLenZ]).reshape((3, 1))
    if leg%2==1:
        bodyVec[1, 0] = -bodyLenY
    if leg/2>=1:
        bodyVec[0, 0] = -bodyLenX
    legPos = pos-bodyVec 
    px, py, pz = legPos[0, 0], legPos[1, 0], legPos[2, 0]
    
    
    '''
        you need to calculate the hipy and knee joint position by inverse kinematics
    '''
    l0=sqrt(px**2+py**2+pz**2-lHip**2)
    if leg%2==0:
        l1 = lHip
    else:
        l1 = -lHip
    l2 = - lThigh
    l3 = - lShank
    
    L = sqrt(py**2 + pz**2 - l1**2)
    thetaHipx = acos(lHip/sqrt(py**2 + pz**2)) - atan(abs(pz/py))
    if leg%2==0:
        thetaHipx = - thetaHipx
    else:
        thetaHipx = thetaHipx
    # thetaHipx = - atan2(pz*l1 + py * L, py * l1 - pz * L)
    thetaKnee = pi - acos((lThigh**2+lShank**2-l0**2)/(2*lThigh*lShank))
    # print(f"px = {px}, py = {py}, pz = {pz}")
    a1 = py*sin(-thetaHipx)-pz*cos(-thetaHipx)
    a2 = px
    m1 = l3*sin(-thetaKnee)
    m2 = l3*cos(-thetaKnee) + l2
    thetaHipy = -atan2(a1*m1+a2*m2, a2*m1-a1*m2)
    # print(f"l0 = {l0}, lHip = {lHip}, lThigh = {lThigh}, lShank = {lShank}")
    return np.array([thetaHipx, thetaHipy, thetaKnee]).reshape(3, 1)
        
    
if __name__=='__main__':
    
    # 加载仿真器
    import pybullet as p
    import pybullet_data as pyd
    
    p.connect(p.GUI) # connect to simulation server
    p.setGravity(0, 0, -9.81) # set gravity
    p.setAdditionalSearchPath(pyd.getDataPath()) 
    
    # load ground
    floor = p.loadURDF("plane.urdf")
    zInit = 0.5
    startPos = [0.0, 0, zInit]
    urdfFlags = p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT | p.URDF_USE_INERTIA_FROM_FILE
    robot = p.loadURDF(urdfFileName, startPos, flags=urdfFlags, useFixedBase=True)
    
    # get joint numbers
    numOfJoints = p.getNumJoints(robot)
    jointPos = []
    print("numOfJoints:  ", numOfJoints)

    rollId = p.addUserDebugParameter(" roll", -PI/2.0, PI/2.0, 0)
    pitchId = p.addUserDebugParameter(" pitch", -PI/2.0, PI/2.0, 0)
    yawId = p.addUserDebugParameter(" yaw", -PI/2.0, PI/2.0, 0)
    xWorld = p.addUserDebugParameter(" X", -1, 1, 0)
    yWorld = p.addUserDebugParameter(" Y", -1, 1, 0)
    zWorld = p.addUserDebugParameter(" Z", 0.0, 2, zInit)
    
    jointIndexList = []
    for j in range(numOfJoints):
        jointInfo = p.getJointInfo(robot, j)
        jointName = jointInfo[1].decode("UTF-8")
        lower = jointInfo[8]
        upper = jointInfo[9]
        if upper < lower:
            upper = PI
            lower = -PI
        default = (lower+upper)/2.
        if lower*upper <= 0:
            default=0
        if jointInfo[2]==p.JOINT_REVOLUTE:
            jointIndexList.append(j)
            jointPos.append(p.addUserDebugParameter(' '+jointName, lower, upper, default))
        print(j, " : ", jointName, " qIndex : ", jointInfo[3], " uIndex : ", jointInfo[4], "lower : ", lower, 'upper : ', upper)

    dt = 1./1000.
    runCnt = 0
    numOfRevolute = len(jointIndexList)
    jointPosVec = [0]*numOfRevolute
    p.setTimeStep(dt)
    # start simulation
    
    legNum=1
    
    
    while True:
        roll = p.readUserDebugParameter(rollId)
        pitch = p.readUserDebugParameter(pitchId)
        yaw = p.readUserDebugParameter(yawId)
        xSet = p.readUserDebugParameter(xWorld)
        ySet = p.readUserDebugParameter(yWorld)
        zSet = p.readUserDebugParameter(zWorld)
        quat = p.getQuaternionFromEuler([roll, pitch, yaw])
        p.resetBasePositionAndOrientation(robot, [xSet, ySet, zSet], quat)
        for i in range(numOfRevolute):
            jointPosVec[i] = p.readUserDebugParameter(jointPos[i])
            p.resetJointState(robot, jointIndexList[i], jointPosVec[i])#关节限制不存在了
        '''
            choose one leg to test 
        '''
        # legNum=1
        jointPosInput=np.array(jointPosVec[legNum*3:legNum*3+3]).reshape(3, 1)
        rotMat = rotZ(yaw)@rotY(pitch)@rotX(roll)

        footPosCalculate = legForwardKinematic(jointPosInput, legNum)
        jointPosCalculate = legInverseKinematic(footPosCalculate, legNum)

        footInfo= p.getLinkState(robot, 4*legNum+3)
        footPosWorld = np.array(footInfo[0]).reshape(3, 1)
        basePosWorld = np.array([xSet, ySet, zSet]).reshape(3, 1)
        np.set_printoptions(precision=3, suppress=True)
        if runCnt%10==0:
            '''
                two numpy array will be equal if your calculation is correct
            '''
            print(f'leg {legNum} FK sim&cal:   ', footPosWorld.T, ' | ', (basePosWorld+rotMat@footPosCalculate).T)
            print(f'leg {legNum} IK inv&obs:   ', jointPosInput.T, ' | ', jointPosCalculate.T)
        
        runCnt = runCnt + 1
        p.stepSimulation()
        time.sleep(0.001)
