#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

import rospy
import time
import numpy as np
from scipy import signal
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from nlink_parser.msg import LinktrackNodeframe3


class imu_observer:

    def __init__(self):

        rospy.on_shutdown(self.cleanup)

        self.imu = rospy.Subscriber("/mavros/imu/data", Imu, self.callback)
        self.uwb = rospy.Subscriber("/nlink_linktrack_nodeframe3", LinktrackNodeframe3, self.callback1)
        self.mocap = rospy.Subscriber("/mocap/pose_01", PoseStamped, self.callback2)

        self.x_pre = np.zeros((6, 1))
        self.x = np.zeros((6, 1))
        self.u = np.zeros((3, 1))
        self.imu_acce = np.zeros((3, 1))

        self.nt = 15
        self.nt_imu_acce = np.zeros((3, 15))
        self.nt_imu_acce_fin = np.zeros((3, 15))
        self.nt_ds = np.zeros((15, 1))
        self.nt_ds_ex = np.zeros((1, 15))
        self.nt_v = np.zeros((15, 1))
        self.nt_x = np.zeros((6, 15))
        self.nt_opt_x = np.zeros((6, 15))
        self.vis_pos = np.zeros((6, 15))

        self.mocap_x = 0.0
        self.mocap_y = 0.0
        self.mocap_z = 0.0

        self.dt_pre = 0.0
        self.dt = 0.04

        self.ds_pre = 0.0
        self.ds = 0.0

        self.m = 0
        self.n = 0

        self.kk = 0.0
        self.g_1 = 0.0
        self.g_2 = 0.0
        self.h_1 = 0.0

    def callback(self, imu):

        # the imu data of angular
        imu_ow = imu.orientation.w
        imu_ox = imu.orientation.x
        imu_oy = imu.orientation.y
        imu_oz = imu.orientation.z
        imu_quaternion = [imu_ow, imu_ox, imu_oy, imu_oz]
        imu_rotation = self.quaternion_to_rotation(imu_quaternion)

        # the linear acceleration
        imu_acce_x = imu.linear_acceleration.x
        imu_acce_y = imu.linear_acceleration.y
        imu_acce_z = imu.linear_acceleration.z
        imu_acce = np.matrix([imu_acce_x, imu_acce_y, imu_acce_z]).T
        imu_acce = np.linalg.pinv(imu_rotation) * imu_acce
        self.imu_acce[2, 0] = imu_acce[2, 0] - 9.81
        self.imu_acce[1, 0] = imu_acce[1, 0] - 0.001
        self.imu_acce[0, 0] = imu_acce[0, 0] + 0.014

    def callback1(self, uwb):

        # if self.mocap_z < 0.2:
        #     return

        # define the low filter
        b, a = signal.butter(3, 0.04, 'low')
        b1, a1 = signal.butter(3, 0.02, 'low')

        if self.m == 0:
            self.dt_pre = time.time()
            self.ds = uwb.nodes[0].dis
            self.m = self.m + 1
            return

        elif self.m < self.nt:

            # self.dt = time.time() - self.dt_pre
            # self.dt_pre = time.time()

            # update the acceleration
            for i in range(3):
                self.nt_imu_acce[i, self.m - 1] = self.imu_acce[i, 0]
            self.ds_pre = self.ds
            self.ds = uwb.nodes[0].dis
            print(self.ds)

            self.nt_ds[self.m - 1] = self.ds
            self.nt_ds_ex[0, self.m - 1] = self.ds
            self.m = self.m + 1
            return

        else:
            # self.dt = time.time() - self.dt_pre
            # self.dt_pre = time.time()

            self.ds_pre = self.ds
            self.ds = uwb.nodes[0].dis
            print(self.ds)

            for i in range(self.nt - 1):
                self.nt_imu_acce[:, i] = self.nt_imu_acce[:, i + 1]
                self.nt_ds[i] = self.nt_ds[i + 1]

            for i in range(3):
                self.nt_imu_acce[i, self.nt - 1] = self.imu_acce[i, 0]
                self.nt_imu_acce[i] = self.nt_imu_acce[i] - np.mean(self.nt_imu_acce[i])
                self.nt_imu_acce[i] = self.nt_imu_acce[i] * 0.5
            self.nt_imu_acce[0] = signal.filtfilt(b, a, self.nt_imu_acce[0])
            self.nt_imu_acce[1] = signal.filtfilt(b, a, self.nt_imu_acce[1])
            self.nt_imu_acce[2] = signal.filtfilt(b, a, self.nt_imu_acce[2])
            # self.nt_imu_acce[2] = self.nt_imu_acce[2] * 0.1
            # self.nt_imu_acce[0] = -self.nt_imu_acce[0]

            self.nt_ds_ex[0, self.nt - 1] = self.ds
            self.nt_ds_ex = signal.filtfilt(b1, a1, self.nt_ds_ex)

            for i in range(self.nt):
                self.nt_ds[i] = self.nt_ds_ex[0, i]

            # get the original state with the transition function
            if self.m == self.nt:
                for i in range(self.nt):
                    ax = self.nt_imu_acce[0][i]
                    ay = self.nt_imu_acce[1][i]
                    az = self.nt_imu_acce[2][i]
                    x1 = self.update_x(ax, ay, az, self.x_pre)
                    for j in range(6):
                        self.nt_x[j][i] = x1[j]
                        self.x_pre[j] = x1[j]
                self.m = self.m + 1
                return
            else:
                for i in range(self.nt - 1):
                    self.nt_x[:, i] = self.nt_x[:, i + 1]
                ax = self.nt_imu_acce[0][self.nt - 1]
                ay = self.nt_imu_acce[1][self.nt - 1]
                az = self.nt_imu_acce[2][self.nt - 1]
                x1 = self.update_x(ax, ay, az, self.x_pre)
                for j in range(6):
                    self.nt_x[j][self.nt - 1] = x1[j]
                    self.x_pre[j] = x1[j]
                print(self.x_pre)

                if self.n == 0:
                    # the designment of nonlinear-observer
                    # output & par_derivative
                    # y = (self.x_pre[0, 0] + 3.535) * (self.x_pre[0, 0] + 3.535)
                    # y = (self.x_pre[1, 0] + 0.29) * (self.x_pre[1, 0] + 0.29) + y
                    # y = (self.x_pre[2, 0] + 0.52) * (self.x_pre[2, 0] + 0.52) + y
                    y = (self.x_pre[0, 0] + 3.89) * (self.x_pre[0, 0] + 3.89)
                    y = (self.x_pre[1, 0] + 0.14) * (self.x_pre[1, 0] + 0.14) + y
                    y = (self.x_pre[2, 0] + 0.48) * (self.x_pre[2, 0] + 0.48) + y
                    self.n = self.n + 1
                else:
                    # the designment of nonlinear-observer
                    # output & par_derivative
                    y = (self.x[0, 0] + 3.89) * (self.x[0, 0] + 3.89)
                    y = (self.x[1, 0] + 0.14) * (self.x[1, 0] + 0.14) + y
                    y = (self.x[2, 0] + 0.48) * (self.x[2, 0] + 0.48) + y
                y = np.sqrt(y)
                if y > 1:
                    kk = (self.nt_ds[self.nt - 1] - y) / y
                    self.kk = kk
                else:
                    kk = 0.1

                # How to solve state function
                # add the gain directly ?
                # half-dimension observer

                G = np.zeros((6, 6))
                H = np.zeros((6, 3))
                for i in range(3):
                    G[i, i] = np.exp(kk * self.dt)
                    G[i, i + 3] = (1 / kk) * (np.exp(kk * self.dt) - 1)
                    G[i + 3, i + 3] = 1
                    H[i, i] = (1 / (kk * kk)) * (np.exp(kk * self.dt) - 1) - (1 / kk) * self.dt
                    H[i + 3, i] = self.dt
                self.g_1 = G[0, 0]
                self.g_2 = G[0, 3]
                self.h_1 = H[0, 0]
                # print(G)
                # print(H)

                if self.n == 1:
                    x1 = self.x_pre
                    x = np.matrix(G) * x1 + H * np.matrix([self.nt_imu_acce[0][self.nt - 1],
                                                           self.nt_imu_acce[1][self.nt - 1],
                                                           self.nt_imu_acce[2][self.nt - 1]]).T
                    self.n = self.n + 1
                else:
                    x1 = self.x
                    x = np.matrix(G) * x1 + H * np.matrix([self.nt_imu_acce[0][self.nt - 1],
                                                           self.nt_imu_acce[1][self.nt - 1],
                                                           self.nt_imu_acce[2][self.nt - 1]]).T

                for i in range(6):
                    self.x[i, 0] = x[i]

                print(x)
                print(" ")


                imu_x = str(self.x_pre[0, 0])
                imu_y = str(self.x_pre[1, 0])
                imu_z = str(self.x_pre[2, 0])
                imu_vx = str(self.x_pre[3, 0])
                imu_vy = str(self.x_pre[4, 0])
                imu_vz = str(self.x_pre[5, 0])
                ground_x = str(self.mocap_x)
                ground_y = str(self.mocap_y)
                ground_z = str(self.mocap_z)
                obs_x = str(self.x[0, 0])
                obs_y = str(self.x[1, 0])
                obs_z = str(self.x[2, 0])
                obs_vx = str(self.x[3, 0])
                obs_vy = str(self.x[4, 0])
                obs_vz = str(self.x[5, 0])
                kk = str(float(kk))
                kk_ori = str(float(self.kk))
                g_1 = str(self.g_1)
                g_2 = str(self.g_2)
                h_1 = str(self.h_1)
                d = str(float(self.nt_ds[self.nt - 1]))
                ds_real = np.sqrt(
                    self.mocap_x * self.mocap_x + self.mocap_y * self.mocap_y + self.mocap_z * self.mocap_z)
                ds_real = str(ds_real)

                if abs(self.mocap_x) > 0.14:
                    file_handle = open('/home/yuzhu/2.txt', mode='a+')
                    file_handle.write(ground_x)
                    file_handle.write('    ')
                    file_handle.write(ground_y)
                    file_handle.write('    ')
                    file_handle.write(ground_z)
                    file_handle.write('    ')
                    file_handle.write(imu_x)
                    file_handle.write('    ')
                    file_handle.write(imu_y)
                    file_handle.write('    ')
                    file_handle.write(imu_z)
                    file_handle.write('    ')
                    file_handle.write(obs_x)
                    file_handle.write('    ')
                    file_handle.write(obs_y)
                    file_handle.write('    ')
                    file_handle.write(obs_z)
                    file_handle.write('    ')

                    file_handle.write(imu_vx)
                    file_handle.write('    ')
                    file_handle.write(imu_vy)
                    file_handle.write('    ')
                    file_handle.write(imu_vz)
                    file_handle.write('    ')
                    file_handle.write(obs_vx)
                    file_handle.write('    ')
                    file_handle.write(obs_vy)
                    file_handle.write('    ')
                    file_handle.write(obs_vz)
                    file_handle.write('    ')

                    file_handle.write(kk)
                    file_handle.write('    ')
                    file_handle.write(kk_ori)
                    file_handle.write('    ')
                    file_handle.write(g_1)
                    file_handle.write('    ')
                    file_handle.write(g_2)
                    file_handle.write('    ')
                    file_handle.write(h_1)
                    file_handle.write('    ')
                    file_handle.write(d)
                    file_handle.write('    ')
                    file_handle.write(ds_real)
                    file_handle.write('\n')
                    file_handle.close()

    def callback2(self, mocap):

        self.mocap_x = mocap.pose.position.x
        self.mocap_y = mocap.pose.position.y
        self.mocap_z = mocap.pose.position.z

    def update_x(self, ax, ay, az, x):

        A = np.eye(6)
        B = np.zeros((6, 3))

        for i in range(3):
            A[i][i + 3] = self.dt
            B[i][i] = self.dt * self.dt * 0.5
            B[i + 3][i] = self.dt
        x1 = np.matrix(A) * x + B * np.matrix([ax, ay, az]).T
        return x1

    def func(self):
        v = lambda x: np.sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2])
        return v
    # def alpha_diff(self, f, x, h):

   # change the quaternion to the rotation matrix
    def quaternion_to_rotation(self, quat):
        q = np.mat(quat)
        n = np.dot(q, q.T)
        q = np.sqrt(2.0 / n) * q
        q = np.outer(q, q)
        rot_matrix = np.array([[1.0 - q[2, 2] - q[3, 3], q[1, 2] + q[3, 0], q[1, 3] - q[2, 0]],
                                [q[1, 2] - q[3, 0], 1.0 - q[1, 1] - q[3, 3], q[2, 3] + q[1, 0]],
                                [q[1, 3] + q[2, 0], q[2, 3] - q[1, 0], 1.0 - q[1, 1] - q[2, 2]]], dtype=np.float64)
        return rot_matrix

    def cleanup(self):
        print("shutting down the node")


if __name__ == '__main__':

    try:
        # init the node
        rospy.init_node("mhe")
        rospy.loginfo("starting the node")
        imu_observer()
        rospy.spin()

    except KeyboardInterrupt:
        print("shutting down the node")

