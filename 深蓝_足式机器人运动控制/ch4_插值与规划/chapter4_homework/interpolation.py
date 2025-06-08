"""
 * @file interpolation.py
 * @brief simulation in pybullet for lite3 leg interpolation
 * @author mazunwang
 * @version 1.0
 * @date 2024-10-15
 *
 * @copyright Copyright (c) 2024  DeepRobotics
"""

import pybullet as p
import numpy as np
import pybullet_data as pd
from math import *
import time
import matplotlib.pyplot as plt
#from kinematic import legInverseKinematic #打开则表示启用依赖第三章；关闭则开启第四章独立完成
from lite3_config import Lite3Config  # 保留对 Lite3Config 的依赖
from typing import Tuple
from scipy.special import comb

urdfFileName = Lite3Config.urdf_path
PI = np.pi
lHip = Lite3Config.hip_len
lThigh = Lite3Config.thigh_len
lShank = Lite3Config.shank_len
bodyLenX = Lite3Config.body_len_x
bodyLenY = Lite3Config.body_len_y
bodyLenZ = Lite3Config.body_len_z

def getCubicSplinePos(x0: float, v0: float, xf: float, vf: float, t: float, T: float) -> float:
    """get cubic spline position by initial state and final state

    Args:
        x0 (float): initial position
        v0 (float): initial velocity
        xf (float): final position
        vf (float): final velocity
        t (float): run time
        T (float): period

    Returns:
        float: position at time t
    """
    # TODO: Implement cubic spline calculation
    if t > T:
        print("error! t > T!")
        return xf
    A = np.array([[pow(0, 3), pow(0, 2), pow(0, 1), pow(0, 0)], 
                 [pow(T, 3), pow(T, 2), pow(T, 1), pow(T, 0)], 
                 [3 * pow(0, 2), 2 * pow(0, 1), 1, 0], 
                 [3 * pow(T, 2), 2 * pow(T, 1), 1, 0]])
    b = np.array([x0, xf, v0, vf])
    corr = np.linalg.solve(A, b)
    x0 = corr[0]*pow(t, 3)+corr[1]*pow(t, 2)+corr[2]*pow(t, 1)+corr[3]*pow(t, 0)
    return x0  # Placeholder return value

def getLegSwingTrajectoryPos(p0: np.ndarray, v0: np.ndarray, 
                             pf: np.ndarray, vf: np.ndarray, 
                             runTime: float, period: float, swingHeight: float) -> np.ndarray:
    """calculate the foot position by interpolation

    Args:
        p0 (np.ndarray): initial foot position at base frame
        v0 (np.ndarray): initial foot velocity at base frame
        pf (np.ndarray): final foot position at base frame
        vf (np.ndarray): final foot velocity at base frame
        runTime (float): run time
        period (float): swing duration.
        swingHeight (float): foot swing height.

    Returns:
        np.ndarray: foot position at runTime
    """
    # TODO: Implement foot position calculation based on cubic spline
    # if runTime < 0:
    #     return p0
    # elif runTime > period:
    #     return pf
    # px, py, pz = p0[0, 0], p0[1, 0], p0[2, 0]
    runTime = runTime % (2 * period)
    # print(f"runTime={runTime}")
    
    if runTime <= period:
        px = runTime/period*(pf[0, 0] - p0[0, 0]) + p0[0, 0]
        py = runTime/period*(pf[1, 0] - p0[1, 0]) + p0[1, 0]
    else:
        px = pf[0, 0] - (runTime - period)/period*(pf[0, 0] - p0[0, 0])
        py = pf[1, 0] - (runTime - period)/period*(pf[1, 0] - p0[1, 0])

    # if runTime <= 0.5 * period:
    #     pz = p0[2, 0] + (runTime - 0)/period * 2  * swingHeight
    # elif runTime <= period:
    #     pz = p0[2, 0] + swingHeight - (runTime - 0.5 * period)/period * 2 * swingHeight
    # elif runTime <= 1.5 * period:
    #     pz = p0[2, 0] + ((runTime - period) - 0)/period * 2  * swingHeight
    # elif runTime <= 2 * period:
    #     pz = p0[2, 0] + swingHeight - ((runTime - period) - 0.5 * period)/period * 2 * swingHeight
    A = np.array([[pow(0, 2), pow(0, 1), pow(0, 0)], 
                  [pow(period, 2), pow(period, 1), pow(period, 0)],
                  [pow(period/2, 2), pow(period/2, 1), pow(period/2, 0)]])
    b = np.array([p0[2, 0], pf[2, 0], p0[2, 0]+swingHeight])
    pz_corr = np.linalg.solve(A, b)
    if runTime <= period:
        pz = getCubicSplinePos(p0[2, 0],2*pz_corr[0]*0+pz_corr[1],pf[2, 0],2*pz_corr[0]*period+pz_corr[1], runTime, period)
    else:
        runTime = 2*period - runTime 
        pz = getCubicSplinePos(p0[2, 0],2*pz_corr[0]*0+pz_corr[1],pf[2, 0],2*pz_corr[0]*period+pz_corr[1], runTime, period)
    # print("px:", p0.reshape(1, 3))
    # print("pf:", pf.reshape(1, 3))
    # print("pz_corr: ", pz_corr)
    # pz = 0
    
    
    
    
    
    # print(f"px:{px}, py:{py}, pz: {pz}")
    return np.array([[px], [py], [pz]])  # Placeholder return value


if __name__ == "__main__":
    # Initialize PyBullet
    p.connect(p.GUI) 
    p.setGravity(0, 0, -9.81) 
    p.setAdditionalSearchPath(pd.getDataPath()) 

    # Load ground
    floor = p.loadURDF("plane.urdf") 
    zInit = 0.5
    startPos = [0.0, 0, zInit]
    urdfFlags = p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT | p.URDF_USE_INERTIA_FROM_FILE
    robot = p.loadURDF(urdfFileName, startPos, flags=urdfFlags, useFixedBase=True)

    numOfJoints = p.getNumJoints(robot)
    jointPos = []
    print("numOfJoints:  ", numOfJoints)

    # Set initial joint angles
    jointIndexList = []
    for j in range(numOfJoints):
        jointInfo = p.getJointInfo(robot, j)
        jointName = jointInfo[1].decode("UTF-8")
        lower = jointInfo[8]
        upper = jointInfo[9]
        if upper < lower:
            upper = PI
            lower = -PI
        default = (lower + upper) / 2.
        if lower * upper <= 0:
            default = 0
        if jointInfo[2] == p.JOINT_REVOLUTE:
            jointIndexList.append(j)
            p.setJointMotorControl2(robot, j, p.VELOCITY_CONTROL, force=0)
        print(f"{j} : {jointName} qIndex : {jointInfo[3]} uIndex : {jointInfo[4]} lower : {lower} upper : {upper}")

    # Create obstacle
    box_half_extents = [0.04 / 2, 0.2 / 2, 0.20 / 2]  # Box dimensions
    collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=box_half_extents)
    visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=box_half_extents)
    p.createMultiBody(baseMass=0.1, 
                    baseCollisionShapeIndex=collision_shape, 
                    baseVisualShapeIndex=visual_shape, 
                    basePosition=[bodyLenX - 0.03, 0.2, box_half_extents[2]])

    dt = 1. / 1000.
    runCnt = 0

    numOfRevolute = len(jointIndexList)
    p.setTimeStep(dt)
    flInitPos = np.array([-0.15 + bodyLenX, 0.02 + bodyLenY + lHip, -0.35]).reshape((3, 1))
    flInitVel = np.zeros((3, 1))
    flFinalPos = np.array([0.10 + bodyLenX, 0.00 + bodyLenY + lHip, -0.35]).reshape((3, 1))
    flJointPosCmd = Lite3Config.legInverseKinematic(flInitPos, 0)  # 使用接口函数

    # Set robot initial pose
    quat = p.getQuaternionFromEuler([0, 0, 0])
    p.resetBasePositionAndOrientation(robot, [0, 0, 0.5], quat)

    startTime = 1.
    runTime = 0

    swingPeriod = 1.
    swingHeight = 0.2

    runTimeList = []
    flFootPosList = []
    flJointPosCmdList = []
    flJointVelCmdList = []
    flJointPosList = []
    flJointVelList = []

    # # Simulation loop
    while runTime < startTime + swingPeriod + 1.:
        jointStates = p.getJointStates(robot, jointIndexList)
        jointPos = np.array([jointStates[i][0] for i in range(12)]).reshape(12, 1)
        jointVel = np.array([jointStates[i][1] for i in range(12)]).reshape(12, 1)
        runCnt += 1
        runTime = runCnt * 0.001
        if runTime >= startTime:
            flFootPos = getLegSwingTrajectoryPos(flInitPos, np.zeros((3, 1)), flFinalPos, np.zeros((3, 1)), runTime - startTime, swingPeriod, swingHeight)
            flFootPosNext = getLegSwingTrajectoryPos(flInitPos, np.zeros((3, 1)), flFinalPos, np.zeros((3, 1)), runTime - startTime + dt, swingPeriod, swingHeight)
            flJointPosCmd = Lite3Config.legInverseKinematic(flFootPos, 0)  # 使用接口函数
            flJointPosNextCmd = Lite3Config.legInverseKinematic(flFootPosNext, 0)  # 使用接口函数
            flJointVelCmd = (flJointPosNextCmd - flJointPosCmd) / dt

            if runTime - startTime <= swingPeriod + 0.01:  # Collect data
                runTimeList.append(runTime)
                flFootPosList.append(flFootPos.reshape(3).tolist())
                flJointPosCmdList.append(flJointPosCmd.reshape(3).tolist())
                flJointVelCmdList.append(flJointVelCmd[:3, 0])
                flJointVelList.append(jointVel[:3, 0])
                flJointPosList.append(jointPos[:3, 0])
            for i in range(3):			
                tau = 100 * (flJointPosCmd[i, 0] - jointPos[i, 0]) + 2.5 * (flJointVelCmd[i, 0] - jointVel[i, 0])
                p.setJointMotorControl2(robot, jointIndexList[i], 
                                        controlMode=p.TORQUE_CONTROL, 
                                        force=tau)
        else:
            for i in range(3):
                p.resetJointState(robot, jointIndexList[i], targetValue=flJointPosCmd[i, 0])	

        p.stepSimulation()
        time.sleep(dt)


    # Simulation loop
    # while True:
    #     jointStates = p.getJointStates(robot, jointIndexList)
    #     jointPos = np.array([jointStates[i][0] for i in range(12)]).reshape(12, 1)
    #     jointVel = np.array([jointStates[i][1] for i in range(12)]).reshape(12, 1)
    #     runCnt += 1
    #     runTime = runCnt * 0.001
    #     if runTime >= startTime:
    #         flFootPos = getLegSwingTrajectoryPos(flInitPos, np.zeros((3, 1)), flFinalPos, np.zeros((3, 1)), runTime - startTime, swingPeriod, swingHeight)
    #         flFootPosNext = getLegSwingTrajectoryPos(flInitPos, np.zeros((3, 1)), flFinalPos, np.zeros((3, 1)), runTime - startTime + dt, swingPeriod, swingHeight)
    #         flJointPosCmd = Lite3Config.legInverseKinematic(flFootPos, 0)  # 使用接口函数
    #         flJointPosNextCmd = Lite3Config.legInverseKinematic(flFootPosNext, 0)  # 使用接口函数
    #         flJointVelCmd = (flJointPosNextCmd - flJointPosCmd) / dt

    #         if runTime - startTime <= swingPeriod + 0.01:  # Collect data
    #             runTimeList.append(runTime)
    #             flFootPosList.append(flFootPos.reshape(3).tolist())
    #             flJointPosCmdList.append(flJointPosCmd.reshape(3).tolist())
    #             flJointVelCmdList.append(flJointVelCmd[:3, 0])
    #             flJointVelList.append(jointVel[:3, 0])
    #             flJointPosList.append(jointPos[:3, 0])
    #         for i in range(3):			
    #             tau = 100 * (flJointPosCmd[i, 0] - jointPos[i, 0]) + 2.5 * (flJointVelCmd[i, 0] - jointVel[i, 0])
    #             p.setJointMotorControl2(robot, jointIndexList[i], 
    #                                     controlMode=p.TORQUE_CONTROL, 
    #                                     force=tau)
    #     else:
    #         for i in range(3):
    #             p.resetJointState(robot, jointIndexList[i], targetValue=flJointPosCmd[i, 0])	

    #     p.stepSimulation()
    #     time.sleep(dt)




    # 绘图
    '''
        Plot leg swing trajectory and joint data
    '''
    fig, axes = plt.subplots(nrows=3, ncols=1, sharex=True)
    fig.suptitle('FL Foot Position')
    titleName = ['X', 'Y', 'Z']
    for i in range(3):
        axes[i].plot(runTimeList, [row[i] for row in flFootPosList])
        axes[i].set_title(titleName[i])
        axes[i].grid()
    axes[-1].set_xlabel('time/s')

    fig, axes = plt.subplots(nrows=3, ncols=1, sharex=True)
    fig.suptitle('FL Joint Angle')
    titleName = ['HipX', 'HipY', 'Knee']
    for i in range(3):
        axes[i].plot(runTimeList, [row[i] for row in flJointPosCmdList], label="cmd")
        axes[i].plot(runTimeList, [row[i] for row in flJointPosList], label="real")
        axes[i].legend(loc="upper right")
        axes[i].set_title(titleName[i])
        axes[i].grid()
    axes[-1].set_xlabel('time/s')

    fig, axes = plt.subplots(nrows=3, ncols=1, sharex=True)
    fig.suptitle('FL Joint Vel')
    titleName = ['HipX', 'HipY', 'Knee']
    for i in range(3):
        axes[i].plot(runTimeList, [vec[i] for vec in flJointVelCmdList], label="cmd")
        axes[i].plot(runTimeList, [vec[i] for vec in flJointVelList], label="real")
        axes[i].legend(loc="upper right")
        axes[i].set_title(titleName[i])
        axes[i].grid()
    axes[-1].set_xlabel('time/s')
    plt.show()