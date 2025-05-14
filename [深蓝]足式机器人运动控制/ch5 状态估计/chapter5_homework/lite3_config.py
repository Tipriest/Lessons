# E:\leggedrobot\leggedRobotics 5.5\leggedRobotics\lite3_config.py
from utils.private.hidden.inverse_kinematics import legForwardKinematic, legInverseKinematic  # 直接导入函数
import numpy as np

class Lite3Config:
    hip_len = 0.09735
    thigh_len = 0.2
    shank_len = 0.21012
    body_len_x = 0.1745
    body_len_y = 0.062
    body_len_z = 0
    "/home/tipriest/Documents/Lessons/[深蓝]足式机器人运动控制/ch3_kinematic/leggedRobotics/urdf_model/Lite3/urdf/Lite3.urdf"

    @staticmethod
    def legForwardKinematic(angle, leg):
        """公共接口函数，调用私有的正运动学计算。"""
        return legForwardKinematic(angle, leg)  # 直接调用函数

    @staticmethod
    def legInverseKinematic(pos, leg):
        """公共接口函数，调用私有的逆运动学计算。"""
        return legInverseKinematic(pos, leg)  # 直接调用函数

    @staticmethod
    def rotX(roll):
        """计算绕X轴的旋转矩阵"""
        return np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])

    @staticmethod
    def rotY(pitch):
        """计算绕Y轴的旋转矩阵"""
        return np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])

    @staticmethod
    def rotZ(yaw):
        """计算绕Z轴的旋转矩阵"""
        return np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])