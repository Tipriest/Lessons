# E:\leggedrobot\leggedRobotics 5.5\leggedRobotics\kalman_filter5.1.py
import numpy as np
import pandas as pd
from math import *
import matplotlib.pyplot as plt
#from kinematic import rotX, rotY, rotZ, legForwardKinematic
from lite3_config import Lite3Config  # 使用新的配置文件

def crossMat(v):
    v = v.reshape(3)
    res = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return res

def getLegJacobian(angle, leg):
    x, y, k = Lite3Config.hip_len, -Lite3Config.thigh_len, -Lite3Config.shank_len
    if leg == 1 or leg == 3:
        x = -x
    q1, q2, q3 = angle[0], angle[1], angle[2]
    c1, s1 = cos(q1), sin(q1)
    c2, s2 = cos(q2), sin(q2)
    c3, s3 = cos(q3), sin(q3)
    J = np.array([[0, -k * ((c2) * (c3) - (s2) * (s3)) - y * (c2), -k * ((c2) * (c3) - (s2) * (s3))],
                  [k * ((c1) * (c2) * (c3) - (c1) * (s2) * (s3)) - x * (s1) + y * (c1) * (c2),
                   -k * ((c2) * (s1) * (s3) + (c3) * (s1) * (s2)) - y * (s1) * (s2),
                   -k * ((c2) * (s1) * (s3) + (c3) * (s1) * (s2))],
                  [k * ((s1) * (s2) * (s3) - (c2) * (c3) * (s1)) - x * (c1) - y * (c2) * (s1),
                   -k * ((c1) * (c2) * (s3) + (c1) * (c3) * (s2)) - y * (c1) * (s2),
                   -k * ((c1) * (c2) * (s3) + (c1) * (c3) * (s2))]])
    return J

class DataRead():
    def __init__(self, file_path):
        data = pd.read_csv(file_path, header=1, encoding='utf-8')
        self.data_mat = data.iloc[:, :].values
        self.time = self.data_mat[:, 2] - self.data_mat[0, 2]
        self.joint_pos = self.data_mat[:, 3:15]
        self.joint_vel = self.data_mat[:, 15:27]
        self.joint_tau = self.data_mat[:, 27:39]
        self.rpy = self.data_mat[:, 63:66]
        self.acc = self.data_mat[:, 66:69]
        self.omg = self.data_mat[:, 69:72]

        # Add sensor noise
        self.rpy += np.random.normal(0, 0.1, self.rpy.shape)
        self.acc += np.random.normal(0, 1.0, self.acc.shape)
        self.omg += np.random.normal(0, 0.8, self.omg.shape)
        self.joint_pos += np.random.normal(0, 0.03, self.joint_pos.shape)
        self.joint_vel += np.random.normal(0, 1.6, self.joint_vel.shape)
        self.joint_tau += np.random.normal(0, 1.0, self.joint_tau.shape)

        self.state = self.data_mat[:, 72]
        self.data_length = len(self.time)

class SimpleKalmanFilter():
    def __init__(self, dt=0.001) -> None:
        self.dt = dt
        eye3 = np.identity(3)

        self.A = np.zeros((6, 6))
        self.A[0:3, 0:3] = eye3
        self.A[0:3, 3:6] = self.dt * eye3
        self.A[3:6, 3:6] = eye3

        self.B = np.zeros((6, 3))
        self.B[:3, :] = 0.5 * self.dt * self.dt * eye3
        self.B[3:6, :] = self.dt * eye3

        self.H = np.zeros((4, 6))
        self.H[:, 2::] = np.identity(4)

        self.P = 1 * np.identity(6)
        self.Z = np.zeros((4, 1))

        # Process noise covariance
        self.Q = np.zeros((6, 6))
        self.Q[0:3, 0:3] = 1e-1 * eye3  # Increased process noise
        self.Q[3:6, 3:6] = 1e-1 * eye3  # Increased process noise

        # Measurement noise covariance
        self.R = np.zeros((4, 4))
        self.R = np.diag([0.5, 2.0, 2.0, 2.0])  # Reduced measurement noise

        self.velocity_record = np.zeros((3, 1))
        self.height_record = 0
        self.off_land_cnt = 0
        self.X = np.zeros((6, 1))
        self.x_predict = np.zeros((6, 1))

        self.pos_world = np.zeros((3, 1))
        self.vel_world = np.zeros((3, 1))
        self.vel_body = np.zeros((3, 1))
        self.contact_sum = 0

    def run(self, rBody, aBody, xyz_pos, xyz_vel, contact):
        g = np.array([[0], [0], [-9.81]])
        a = rBody @ aBody + g
        for i in range(4):
            if contact[i, 0] > 0.1 and contact[i, 0] < 0.9:
                contact[i, 0] = 1
            else:
                contact[i, 0] = 0
        contact_sum = np.sum(contact)

        if contact_sum != 0:
            pos_world = np.zeros((3, 1))
            vel_world = np.zeros((3, 1))
            for i in range(4):
                pos_world += contact[i, 0] * np.asmatrix(xyz_pos[i, :]).T
                vel_world += contact[i, 0] * np.asmatrix(xyz_vel[i, :]).T
            pos_world /= contact_sum
            vel_world /= contact_sum
            self.Z = np.vstack((pos_world[2, 0], vel_world))
        else:
            self.off_land_cnt += 1
            off_land_time = self.off_land_cnt * self.dt
            vel_world = self.velocity_record + off_land_time * g
            height = self.height_record + off_land_time * self.velocity_record[2, 0] + 0.5 * off_land_time * off_land_time * g[2, 0]
            self.Z = np.vstack((height, vel_world))

        # 需要计算卡尔曼增益以完成卡尔曼滤波
        # TODO: 计算卡尔曼增益 K_k
        # TODO: 更新状态估计
        # TODO: 更新协方差矩阵
        
        self.X = self.A @ self.X + self.B @ a
        self.x_predict = self.X

        self.pos_world = self.X[:3, :]
        self.vel_world = self.X[3:, :]
        self.vel_body = rBody.T @ self.vel_world
        self.vel_body_predict = rBody.T @ self.x_predict[3:, :]
        self.vel_body_obs = rBody.T @ self.Z[1:, :]

        
        if contact_sum != 0:
            self.off_land_cnt = 0
            self.velocity_record = self.X[3::, :]
            self.height_record = self.X[2, 0]

    def init(self, height):
        self.X = np.zeros((6, 1))
        self.X[2, 0] = height
        self.velocity_record = np.zeros((3, 1))
        self.height_record = 0
        self.off_land_cnt = 0
        self.pos_world = self.X[:3, :]
        self.vel_world = self.X[3:, :]
        self.vel_body = np.zeros((3, 1))

if __name__ == '__main__':
    skf = SimpleKalmanFilter()
    file_path = "./data/kf_data.csv"
    dr = DataRead(file_path)

    t = []
    data_length = dr.data_length
    skf_odom = np.zeros((data_length, 3))
    skf_vel_body = np.zeros((data_length, 3))
    skf_vel_pred = np.zeros((data_length, 3))
    skf_vel_obs = np.zeros((data_length, 3))
    skf.init(0.32)
    for i in range(data_length):
        joint_pos = np.array(dr.joint_pos[i, :]).reshape(12, 1)
        joint_vel = np.array(dr.joint_vel[i, :]).reshape(12, 1)
        joint_tau = np.array(dr.joint_tau[i, :]).reshape(12, 1)
        rot_mat = Lite3Config.rotZ(dr.rpy[i, 2]) @ Lite3Config.rotY(dr.rpy[i, 1]) @ Lite3Config.rotX(dr.rpy[i, 0])  # 使用公共接口
        acc = np.array(dr.acc[i, :]).reshape((3, 1))
        omega_body = np.array(dr.omg[i, :]).reshape((3, 1))
        leg_pos = np.zeros((4, 3))
        leg_vel = np.zeros((4, 3))
        contact = np.zeros((4, 1))
        for j in range(4):
            J = getLegJacobian(joint_pos[3 * j:3 * j + 3, 0], j)
            vel_local = (J @ joint_vel[3 * j:3 * j + 3, 0]).reshape(3, 1)
            pos_local = Lite3Config.legForwardKinematic(joint_pos[3 * j:3 * j + 3, 0].flatten(), j)  # 使用公共接口
            leg_pos[j, :] = -(rot_mat @ pos_local).T
            leg_vel[j, :] = -(crossMat(rot_mat @ omega_body) @ pos_local + rot_mat @ vel_local).T
            # Foot contact detection
            esti_force = (rot_mat @ np.linalg.inv(J.T)) @ (-joint_tau[3 * j:3 * j + 3, 0].T)
            if esti_force[2] >= 15:
                contact[j, 0] = 0.5
        skf.run(rot_mat, acc, leg_pos, leg_vel, contact)

        t.append(dr.time[i])
        skf_odom[i, :] = skf.pos_world.T
        skf_vel_body[i, :] = skf.vel_body.T
        skf_vel_pred[i, :] = skf.vel_body_predict.T
        skf_vel_obs[i, :] = skf.vel_body_obs.T

    # Plotting results
    fig1, axes = plt.subplots(nrows=3, ncols=1, sharex=True, num='XYZ')
    fig1.suptitle('Robot Odometery', fontsize=25)
    title_name = r'$\it{X}$'
    axe = axes[0]
    axe.plot(t, skf_odom[:, 0], label=r'$\it{x}$', color='r')
    axe.legend(loc="upper right")
    axe.set_title(title_name, fontsize=15)
    axe.set_ylabel('x/m', fontsize=18)
    axe.grid()
    title_name = r'$\it{Y}$'
    axe = axes[1]
    axe.plot(t, skf_odom[:, 1], label=r'$\it{y}$', color='r')
    axe.legend(loc="upper right")
    axe.set_title(title_name, fontsize=15)
    axe.set_ylabel('y/m', fontsize=18)
    axe.grid()
    title_name = r'$\it{Z}$'
    axe = axes[2]
    axe.plot(t, skf_odom[:, 2], label=r'$\it{z}$', color='r')
    axe.legend(loc="upper right")
    axe.set_title(title_name, fontsize=15)
    axe.set_ylabel('z/m', fontsize=18)
    axe.set_xlabel('time/s', fontsize=18)
    axe.grid()

    fig2, axes = plt.subplots(nrows=3, ncols=1, sharex=True, num='Vel')
    fig2.suptitle('Robot Velocity', fontsize=25)
    title_name = r'$\it{v_x}$'
    axe = axes[0]
    axe.plot(t, skf_vel_obs[:, 0], label=r'$\it{obs}$', color='b')
    axe.plot(t, skf_vel_body[:, 0], label=r'$\it{filt}$', color='r', alpha=0.9)
    axe.plot(t, skf_vel_pred[:, 0], label=r'$\it{pred}$', color='g', linestyle=':')

    axe.legend(loc="upper right")
    axe.set_title(title_name, fontsize=15)
    axe.set_ylabel('vx/(m/s)', fontsize=18)
    axe.grid()
    title_name = r'$\it{V_y}$'
    axe = axes[1]
    axe.plot(t, skf_vel_obs[:, 1], label=r'$\it{obs}$', color='c')
    axe.plot(t, skf_vel_body[:, 1], label=r'$\it{filt}$', color='r', alpha=0.9)
    axe.plot(t, skf_vel_pred[:, 1], label=r'$\it{pred}$', color='g', linestyle=':')

    axe.legend(loc="upper right")
    axe.set_title(title_name, fontsize=15)
    axe.set_ylabel('vy/(m/s)', fontsize=18)
    axe.grid()
    title_name = r'$\it{V_z}$'
    axe = axes[2]
    axe.plot(t, skf_vel_obs[:, 2], label=r'$\it{obs}$', color='b')
    axe.plot(t, skf_vel_body[:, 2], label=r'$\it{filt}$', color='r', alpha=0.9)
    axe.plot(t, skf_vel_pred[:, 2], label=r'$\it{pred}$', color='g', linestyle=':')

    axe.legend(loc="upper right")
    axe.set_title(title_name, fontsize=15)
    axe.set_ylabel('vz/(m/s)', fontsize=18)
    axe.set_xlabel('time/s', fontsize=18)
    axe.grid()

    fig3 = plt.figure(num='2D')
    title_name = r'$\it{XoY}$'
    ax = fig3.add_subplot(111)
    ax.plot(skf_odom[:, 0], skf_odom[:, 1], label=r'$\it{skf}$', color='r')
    ax.legend(loc="upper right")
    ax.axis('equal')
    ax.grid()
    plt.show()
