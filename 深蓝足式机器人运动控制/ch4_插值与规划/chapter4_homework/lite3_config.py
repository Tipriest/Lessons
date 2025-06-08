# lite3_config.py
import numpy as np
from math import acos, sqrt, atan2

class Lite3Config:
    hip_len = 0.09735
    thigh_len = 0.2
    shank_len = 0.21012
    body_len_x = 0.1745
    body_len_y = 0.062
    body_len_z = 0
    urdf_path = "/home/tipriest/Documents/Lessons/[深蓝]足式机器人运动控制/ch3_kinematic/leggedRobotics/urdf_model/Lite3/urdf/Lite3.urdf"

    @staticmethod
    def legInverseKinematic(pos: np.ndarray, leg: int) -> np.ndarray:
        """计算Lite3机器人某条腿的逆向运动学。

        Args:
            pos (vec3): 脚部在基座坐标系中的位置，为一个形状为(3, 1)的numpy数组。
            leg (int): 指定腿的编号，0:FL, 1:FR, 2:HL, 3:HR

        Returns:
            [vec3]: 关节角度，为一个形状为(3, 1)的numpy数组。
        """
        # 从配置中获取各个部分的长度
        lHip = Lite3Config.hip_len
        lThigh = Lite3Config.thigh_len
        lShank = Lite3Config.shank_len
        bodyLenX = Lite3Config.body_len_x
        bodyLenY = Lite3Config.body_len_y
        bodyLenZ = Lite3Config.body_len_z

        # 根据机身尺寸调整腿的位置
        bodyVec = np.array([bodyLenX, bodyLenY, bodyLenZ]).reshape((3, 1))
        if leg % 2 == 1:  # 左腿
            bodyVec[1, 0] = -bodyLenY
            lHip = -lHip
        if leg // 2 == 1:  # 后腿
            bodyVec[0, 0] = -bodyLenX

        legPos = pos - bodyVec
        px, py, pz = legPos[0, 0], legPos[1, 0], legPos[2, 0]
        # 计算hipx关节的角度
        theta2 = atan2(abs(pz), py)
        l1 = sqrt(py ** 2 + pz ** 2)
        theta3 = acos(lHip / l1)
        thetaHipx = theta2 - theta3
        l0 = sqrt(px ** 2 + py ** 2 + pz ** 2 - lHip ** 2)
        d_prime_yz = sqrt(py ** 2 + pz ** 2 - lHip ** 2)
        c = (l0 ** 2 - lShank ** 2 - lThigh ** 2) / (2 * lThigh)
        alpha_1 = acos((lThigh + c) / l0)
        alpha_2 = -atan2(px, d_prime_yz)
        # 计算hipy关节的角度
        thetaHipy = -alpha_1 - alpha_2
        # 计算knee关节的角度
        theta4 = acos((lThigh ** 2 + lShank ** 2 - l0 ** 2) / (2 * lThigh * lShank))
        thetaKnee = np.pi - theta4

        return np.array([[thetaHipx], [thetaHipy], [thetaKnee]])