# E:\leggedrobot\leggedRobotics 5.5\leggedRobotics\utils\private\hidden\inverse_kinematics.py
import numpy as np
from math import acos, sqrt, atan2

def rotX(roll):
    """计算绕X轴的旋转矩阵"""
    return np.array([[1, 0, 0],
                     [0, np.cos(roll), -np.sin(roll)],
                     [0, np.sin(roll), np.cos(roll)]])

def rotY(pitch):
    """计算绕Y轴的旋转矩阵"""
    return np.array([[np.cos(pitch), 0, np.sin(pitch)],
                     [0, 1, 0],
                     [-np.sin(pitch), 0, np.cos(pitch)]])

def rotZ(yaw):
    """计算绕Z轴的旋转矩阵"""
    return np.array([[np.cos(yaw), -np.sin(yaw), 0],
                     [np.sin(yaw), np.cos(yaw), 0],
                     [0, 0, 1]])

def legForwardKinematic(angle, leg):
    """计算Lite3机器人腿的正运动学

    Args:
        angle: 关节位置的数组，包含一条腿的3个关节的角度
        leg (int): 0:FL, 1:FR, 2:HL, 3:HR

    Returns:
        [vec3]: 脚部在基座坐标系中的位置
    """
    lHip = 0.09735  # 髋部长度
    lThigh = 0.2    # 大腿长度
    lShank = 0.21012 # 小腿长度
    bodyLenX = 0.1745
    bodyLenY = 0.062
    bodyLenZ = 0

    bodyVec = np.array([bodyLenX, bodyLenY, bodyLenZ]).reshape(3, 1)
    hipVec = np.array([0, lHip, 0]).reshape((3, 1))

    # 根据腿的编号调整髋部位置
    if leg % 2 == 1:  # 左腿
        hipVec[1, 0] = -lHip
        bodyVec[1, 0] = -bodyLenY
    if leg // 2 == 1:  # 后腿
        bodyVec[0, 0] = -bodyLenX

    # 创建大腿和小腿向量
    thighVec = np.array([0, 0, -lThigh]).reshape((3, 1))
    shankVec = np.array([0, 0, -lShank]).reshape((3, 1))

    # 计算脚部在髋部坐标系中的位置
    p_foot_hipy = np.array([[0], [0], [-lThigh]]) + rotY(-angle[2]) @ shankVec
    p_foot_hipx = hipVec + rotY(-angle[1]) @ p_foot_hipy

    # 计算脚部在根坐标系中的位置
    footPos = rotX(-angle[0]) @ p_foot_hipx

    return footPos + bodyVec

def legInverseKinematic(pos, leg):
    """计算Lite3机器人某条腿的逆运动学。

    Args:
        pos (vec3): 脚部在基座坐标系中的位置，为一个形状为(3, 1)的numpy数组。
        leg (int): 指定腿的编号，0:FL, 1:FR, 2:HL, 3:HR

    Returns:
        [vec3]: 关节角度，为一个形状为(3, 1)的numpy数组。
    """
    lHip = 0.09735
    lThigh = 0.2
    lShank = 0.21012
    bodyLenX = 0.1745
    bodyLenY = 0.062
    bodyLenZ = 0

    bodyVec = np.array([bodyLenX, bodyLenY, bodyLenZ]).reshape((3, 1))
    if leg % 2 == 1:  # 左腿
        bodyVec[1, 0] = -bodyLenY
        lHip = -lHip
    if leg // 2 == 1:  # 后腿
        bodyVec[0, 0] = -bodyLenX

    legPos = pos - bodyVec
    px, py, pz = legPos[0, 0], legPos[1, 0], legPos[2, 0]

    # 计算l1和theta2
    l1 = sqrt(py ** 2 + pz ** 2)
    if l1 == 0:
        raise ValueError("l1 cannot be zero, check the leg position.")

    theta2 = atan2(pz, py)  # 使用atan2以确保正确的象限

    # 计算theta3
    if lHip > l1:
        raise ValueError("lHip cannot be greater than l1.")

    theta3 = acos(lHip / l1)  # 限制输入范围以避免计算错误
    thetaHipx = theta2 - theta3

    # 计算l0
    l0 = sqrt(px ** 2 + py ** 2 + pz ** 2 - lHip ** 2)

    # 计算d'_{yz}和c
    d_prime_yz = sqrt(py ** 2 + pz ** 2 - lHip ** 2)
    if d_prime_yz < 0:
        raise ValueError("d'_{yz} cannot be negative.")

    c = (l0 ** 2 - lShank ** 2 - lThigh ** 2) / (2 * lThigh)

    # 计算alpha_1和alpha_2
    alpha_1 = acos(min(max((lThigh + c) / l0, -1), 1))  # 限制输入范围
    alpha_2 = atan2(px, d_prime_yz)

    # 计算thetaHipy
    thetaHipy = alpha_2 - alpha_1

    # 计算thetaKnee
    theta4 = acos(min(max((lThigh ** 2 + lShank ** 2 - l0 ** 2) / (2 * lThigh * lShank), -1), 1))  # 限制输入范围
    thetaKnee = np.pi - theta4

    # 限制关节角度在合理范围内
    thetaHipx = np.clip(thetaHipx, -np.pi, np.pi)
    thetaHipy = np.clip(thetaHipy, -2.67, 0.314)  # 假设hipy的范围是[-2.67, 0.314]
    thetaKnee = np.clip(thetaKnee, 0.524, 2.792)  # 假设膝关节的范围是[0.524, 2.792]

    # 检查hipx的范围
    if thetaHipx < -np.pi or thetaHipx > np.pi:
        raise ValueError("thetaHipx out of range.")

    # 返回计算得到的关节角度
    return np.array([thetaHipx, thetaHipy, thetaKnee]).reshape(3, 1)