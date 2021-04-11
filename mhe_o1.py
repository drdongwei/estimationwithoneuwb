#!/usr/bin/env python2.7
# -*- coding:utf-8 -*-

import rospy
import numpy as np
import math
import time
import matplotlib.pyplot as plt
from scipy import signal
from scipy.optimize import *
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu
from nlink_parser.msg import LinktrackNodeframe3


class mhe:

    def __init__(self):

        rospy.on_shutdown(self.cleanup)

        self.imu = rospy.Subscriber("/mavros/imu/data", Imu, self.callback)
        self.vision_pose = rospy.Subscriber("/mavros/vision_pose/pose", PoseStamped, self.callback1)
        self.mocap = rospy.Subscriber("/mocap/pose", PoseStamped, self.callback2)
        self.uwb_frame3 = rospy.Subscriber("/nlink_linktrack_nodeframe3", LinktrackNodeframe3, self.callback3)

        self.imu_acce = np.zeros((3, 1))
        self.obs = np.zeros((3, 1))

        self.mocap_x = 0.0
        self.mocap_y = 0.0
        self.mocap_z = 0.0
        self.obs_x = 0.0
        self.obs_y = 0.0
        self.obs_z = 0.0

        self.nt = 15
        self.nt_imu_acce = np.zeros((3, 15))
        self.nt_imu_acce_fin = np.zeros((3, 15))
        self.nt_ds = np.zeros((15, 1))
        self.nt_ds_ex = np.zeros((1, 15))
        self.nt_v = np.zeros((15, 1))
        self.nt_x = np.zeros((6, 15))
        self.nt_opt_x = np.zeros((6, 15))
        self.vis_pos = np.zeros((6, 15))

        self.x = np.zeros((6, 1))
        self.x_pre = np.zeros((6, 1))
        self.x_ = np.zeros((6, 1))
        self.dt = 0.04

        self.ds_pre = 0.0
        self.ds = 0.0

        self.m = 0
        self.n = 0

    def callback(self, imu):

        # the imu data of angular
        imu_ow = imu.orientation.w
        imu_ox = imu.orientation.x
        imu_oy = imu.orientation.y
        imu_oz = imu.orientation.z
        imu_quaternion = [imu_ow, imu_ox, imu_oy, imu_oz]
        imu_rotation = self.quaternion_to_rotation(imu_quaternion)

        # the imu data of linear acceleration
        imu_acce_x = imu.linear_acceleration.x
        imu_acce_y = imu.linear_acceleration.y
        imu_acce_z = imu.linear_acceleration.z
        imu_acce = np.matrix([imu_acce_x, imu_acce_y, imu_acce_z]).T
        imu_acce = np.linalg.pinv(imu_rotation) * imu_acce
        self.imu_acce[2, 0] = imu_acce[2, 0] - 9.81
        self.imu_acce[1, 0] = imu_acce[1, 0] - 0.001
        self.imu_acce[0, 0] = imu_acce[0, 0] + 0.01

    def callback1(self, pose):

        # the observation data of pose
        self.obs_x = pose.pose.position.x - 0.035
        self.obs_y = pose.pose.position.y - 0.29
        self.obs_z = pose.pose.position.z - 0.02

    def callback2(self, mocap):

        self.mocap_x = mocap.pose.position.x
        self.mocap_y = mocap.pose.position.y
        self.mocap_z = mocap.pose.position.z

    # get the distance from the uwb
    def callback3(self, uwb):

        # define the low filter
        b, a = signal.butter(3, 0.04, 'low')
        b1, a1 = signal.butter(3, 0.02, 'low')

        # restore a data stream
        # if self.mocap_z < 0.1:
        #     return

        if self.m == 0:
            self.vis_pos[0, self.m] = self.obs_x
            self.vis_pos[1, self.m] = self.obs_y
            self.vis_pos[2, self.m] = self.obs_z

            self.ds = uwb.nodes[0].dis
            self.m = self.m + 1
            return

        elif self.m < self.nt + 1:

            # compute the velocity by difference
            self.vis_pos[3, self.m - 1] = (self.obs_x - self.vis_pos[0, self.m - 1]) / self.dt
            self.vis_pos[4, self.m - 1] = (self.obs_y - self.vis_pos[1, self.m - 1]) / self.dt
            self.vis_pos[5, self.m - 1] = (self.obs_z - self.vis_pos[2, self.m - 1]) / self.dt

            # update the position matrix with the vision information
            self.vis_pos[0, self.m - 1] = self.obs_x
            self.vis_pos[1, self.m - 1] = self.obs_y
            self.vis_pos[2, self.m - 1] = self.obs_z

            # update the acceleration
            for i in range(3):
                self.nt_imu_acce[i, self.m - 1] = self.imu_acce[i, 0]
            self.ds_pre = self.ds
            self.ds = uwb.nodes[0].dis

            # initialize the velocity with the distance
            self.nt_v[self.m - 1] = abs(self.ds - self.ds_pre) / self.dt
            self.nt_ds[self.m - 1] = self.ds
            self.nt_ds_ex[0, self.m - 1] = self.ds
            self.m = self.m + 1
            return

        else:
            start = time.clock()

            # update the pose information with the obtained vision information
            v_x = (self.obs_x - self.vis_pos[0, self.nt - 1]) / self.dt
            v_y = (self.obs_y - self.vis_pos[1, self.nt - 1]) / self.dt
            v_z = (self.obs_z - self.vis_pos[2, self.nt - 1]) / self.dt

            for i in range(self.nt - 1):
                self.vis_pos[:, i] = self.vis_pos[:, i + 1]
            self.vis_pos[0, self.nt - 1] = self.obs_x
            self.vis_pos[1, self.nt - 1] = self.obs_y
            self.vis_pos[2, self.nt - 1] = self.obs_z
            self.vis_pos[3, self.nt - 1] = v_x
            self.vis_pos[4, self.nt - 1] = v_y
            self.vis_pos[5, self.nt - 1] = v_z

            # update the pose information with the acceleration and imu information

            self.ds_pre = self.ds
            self.ds = uwb.nodes[0].dis

            for i in range(self.nt - 1):
                self.nt_imu_acce[:, i] = self.nt_imu_acce[:, i + 1]
                self.nt_ds[i] = self.nt_ds[i + 1]
                self.nt_v[i] = self.nt_v[i + 1]
            for i in range(3):
                self.nt_imu_acce[i, self.nt - 1] = self.imu_acce[i, 0]
                self.nt_imu_acce[i] = self.nt_imu_acce[i] - np.mean(self.nt_imu_acce[i])
                self.nt_imu_acce[i] = self.nt_imu_acce[i] * 0.5
            self.nt_imu_acce[0] = signal.filtfilt(b, a, self.nt_imu_acce[0])
            self.nt_imu_acce[1] = signal.filtfilt(b, a, self.nt_imu_acce[1])
            self.nt_imu_acce[2] = signal.filtfilt(b, a, self.nt_imu_acce[2])
            self.nt_imu_acce[2] = self.nt_imu_acce[2] * 0.5

            # print(self.nt_imu_acce)
            # print(" ")

            # if self.m == self.nt + 1:
            #     for i in range(self.nt - 1):
            #         self.nt_imu_acce[:, i] = self.nt_imu_acce[:, i + 1]
            #         self.nt_ds_ex[0, i] = self.nt_ds_ex[0, i + 1]
            #         self.nt_v[i] = self.nt_v[i + 1]
            #
            #     for j in range(3):
            #         self.nt_imu_acce[j, self.nt - 1] = self.imu_acce[j, 0]
            #         self.nt_imu_acce[j] = signal.filtfilt(b, a, self.nt_imu_acce[j])
            #         self.nt_imu_acce[j] = self.nt_imu_acce[j] - np.mean(self.nt_imu_acce[j])
            #
            # else:
            #     for i in range(self.nt - 1):
            #         self.nt_imu_acce[:, i] = self.nt_imu_acce[:, i + 1]
            #         self.nt_imu_acce_fin[:, i] = self.nt_imu_acce[:, i + 1]
            #         self.nt_ds_ex[0, i] = self.nt_ds_ex[0, i + 1]
            #         self.nt_v[i] = self.nt_v[i + 1]
            #
            #     for j in range(3):
            #         self.nt_imu_acce_fin[j, self.nt - 1] = self.imu_acce[j, 0]
            #         self.nt_imu_acce_fin[j] = signal.filtfilt(b, a, self.nt_imu_acce_fin[j])
            #         self.nt_imu_acce_fin[j] = self.nt_imu_acce_fin[j] - np.mean(self.nt_imu_acce_fin[j])
            #         self.nt_imu_acce[j, self.nt - 1] = self.nt_imu_acce_fin[j, self.nt - 1]

            self.nt_ds_ex[0, self.nt - 1] = self.ds
            self.nt_v[self.nt - 1] = abs(self.ds - self.ds_pre) / self.dt
            self.nt_ds_ex = signal.filtfilt(b1, a1, self.nt_ds_ex)

            for i in range(self.nt):
                self.nt_ds[i] = self.nt_ds_ex[0, i]

            # get the original state with the transition function
            if self.m == self.nt + 1:
                for i in range(self.nt):
                    ax = self.nt_imu_acce[0][i]
                    ay = self.nt_imu_acce[1][i]
                    az = self.nt_imu_acce[2][i]
                    x1 = self.update_x(ax, ay, az, self.x_pre)
                    for j in range(6):
                        self.nt_x[j][i] = x1[j]
                        self.x_pre[j] = x1[j]
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

                if self.n == 0:
                    y = (self.x_pre[0, 0] + 3) * (self.x_pre[0, 0] + 3)
                    y = (self.x_pre[1, 0] - 0.7) * (self.x_pre[1, 0] - 0.7) + y
                    y = (self.x_pre[2, 0] - 0.5) * (self.x_pre[2, 0] - 0.5) + y
                    self.n = self.n + 1
                else:
                    y = (self.x_[0, 0] + 3) * (self.x_[0, 0] + 3)
                    y = (self.x_[1, 0] - 0.7) * (self.x_[1, 0] - 0.7) + y
                    y = (self.x_[2, 0] - 0.5) * (self.x_[2, 0] - 0.5) + y
                y = np.sqrt(y)
                if y > 1:
                    kk = (self.nt_ds[self.nt - 1] - y) / y
                else:
                    kk = 0.1
                print(kk)
                # kk = (self.nt_ds[self.nt - 1] - y) / y

                G = np.zeros((6, 6))
                H = np.zeros((6, 3))
                for i in range(3):
                    G[i, i] = np.exp(kk * self.dt)
                    G[i, i + 3] = (1 / kk) * (np.exp(kk * self.dt) - 1)
                    G[i + 3, i + 3] = 1
                    H[i, i] = (1 / (kk * kk)) * (np.exp(kk * self.dt) - 1) - (1 / kk) * self.dt
                    H[i + 3, i] = self.dt
                # print(G)
                # print(H)

                if self.n == 1:
                    for i in range(6):
                        x1[i, 0] = self.x_pre[i, 0]
                    x = np.matrix(G) * x1 + H * np.matrix([self.nt_imu_acce[0][self.nt - 1],
                                                           self.nt_imu_acce[1][self.nt - 1],
                                                           self.nt_imu_acce[2][self.nt - 1]]).T
                    self.n = self.n + 1
                else:
                    for i in range(6):
                        x1[i, 0] = self.x_[i, 0]
                    x = np.matrix(G) * x1 + H * np.matrix([self.nt_imu_acce[0][self.nt - 1],
                                                           self.nt_imu_acce[1][self.nt - 1],
                                                           self.nt_imu_acce[2][self.nt - 1]]).T

                for i in range(6):
                    self.x_[i, 0] = x[i]
                    self.nt_x[i, self.nt - 1] = x[i]

            # print(self.nt_x)
            # print(" ")

            # define the args of the function 'objective'
            args = (80, 100, 0.5, 10)
            # define the args of the function 'cons'
            args1 = (-5.0, 5.0, -5.0, 5.0, -5.0, 5.0, -5.0, 5.0, -5.0, 5.0, -5.0, 5.0)
            cons = self.con(args1)
            if self.m == self.nt + 1:
                x0 = np.asarray((self.vis_pos[0, 14], self.vis_pos[1, 14], self.vis_pos[2, 14], self.vis_pos[3, 14], self.vis_pos[4, 14], self.vis_pos[5, 14]))
            else:
                x0 = np.asarray((self.x_[0], self.x_[1], self.x_[2], self.x_[3], self.x_[4], self.x_[5]))
            res = minimize(self.objective(args), x0, method='Nelder-Mead', constraints=cons)

            print(res.fun)
            print(res.success)
            print(res.x)
            end = time.clock()
            print(end - start)
            for i in range(6):
                self.x[i] = res.x[i]

            ground_x = str(self.mocap_x)
            ground_y = str(self.mocap_y)
            ground_z = str(self.mocap_z)
            ds_real = np.sqrt(self.mocap_x * self.mocap_x + self.mocap_y * self.mocap_y + self.mocap_z * self.mocap_z)
            ds_real = str(ds_real)
            x_cal = str(self.x[0, 0])
            y_cal = str(self.x[1, 0])
            z_cal = str(self.x[2, 0])
            vx_cal = str(self.x[3, 0])
            vy_cal = str(self.x[4, 0])
            vz_cal = str(self.x[5, 0])
            ds = str(self.ds)
            ds_cal = str(self.nt_ds_ex[0, self.nt - 1])

            if abs(self.mocap_x) > 0.14:
                file_handle = open('/home/yuzhu/1.txt', mode='a+')
                file_handle.write(ground_x)
                file_handle.write('    ')
                file_handle.write(ground_y)
                file_handle.write('    ')
                file_handle.write(ground_z)
                file_handle.write('    ')
                file_handle.write(x_cal)
                file_handle.write('    ')
                file_handle.write(y_cal)
                file_handle.write('    ')
                file_handle.write(z_cal)
                file_handle.write('    ')
                file_handle.write(vx_cal)
                file_handle.write('    ')
                file_handle.write(vy_cal)
                file_handle.write('    ')
                file_handle.write(vz_cal)
                file_handle.write('    ')
                file_handle.write(ds)
                file_handle.write('    ')
                file_handle.write(ds_cal)
                file_handle.write('    ')
                file_handle.write('\n')
                file_handle.close()

                imu_x = str(self.x_[0, 0])
                imu_y = str(self.x_[1, 0])
                imu_z = str(self.x_[2, 0])
                imu_vx = str(self.x_[3, 0])
                imu_vy = str(self.x_[4, 0])
                imu_vz = str(self.x_[5, 0])

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
                file_handle.write(imu_vx)
                file_handle.write('    ')
                file_handle.write(imu_vy)
                file_handle.write('    ')
                file_handle.write(imu_vz)
                file_handle.write('\n')
                file_handle.close()

    # optimazition function
    def objective(self, args):

        r1, r2, r3, r4 = args

        err_x = 0.0
        err_d = 0.0
        err_v = 0.0
        err_ori = 0.0

        # there is no update about the optimization value
        if self.m == self.nt + 1:
            for i in range(self.nt):
                for j in range(6):
                    self.nt_opt_x[j, i] = self.vis_pos[j, i]
            self.m = self.m + 1
        else:
            for i in range(self.nt - 1):
                self.nt_opt_x[:, i] = self.nt_opt_x[:, i + 1]
            for i in range(6):
                self.nt_opt_x[i, self.nt - 1] = self.x[i, 0]

        # print(self.nt_opt_x)
        # print(" ")

        for i in range(self.nt - 1):
            for j in range(6):
                err_x = (self.nt_opt_x[j, i + 1] - self.nt_x[j, i]) * (self.nt_opt_x[j, i + 1] - self.nt_x[j, i]) + err_x
            ori_d = (self.nt_opt_x[0, i + 1] + 3) * (self.nt_opt_x[0, i + 1] + 3)
            ori_d = (self.nt_opt_x[1, i + 1] - 0.7) * (self.nt_opt_x[1, i + 1] - 0.7) + ori_d
            ori_d = (self.nt_opt_x[2, i + 1] - 0.5) * (self.nt_opt_x[2, i + 1] - 0.5) + ori_d
            ori_v = 0.0
            for j in range(3):
                ori_v = self.nt_opt_x[j + 3, i + 1] * self.nt_opt_x[j + 3, i + 1] + ori_v
            err_d = (np.sqrt(ori_d) - self.nt_ds[i]) * (np.sqrt(ori_d) - self.nt_ds[i]) + err_d
            err_v = (np.sqrt(ori_v) - self.nt_v[i]) * (np.sqrt(ori_v) - self.nt_v[i]) + err_v
            # err_v = np.exp((self.nt_v[i] + 2) / (np.sqrt(ori_v) + 2)) * (np.sqrt(ori_v) - self.nt_v[i]) * \
            #         (np.sqrt(ori_v) - self.nt_v[i]) + err_v
        #     print(self.nt_v[i])
        #     print(" ")
        #     print(np.sqrt(ori_v))
        print(err_x)
        print(" ")
        print(err_d)
        print(" ")
        print(err_v)
        print(" ")

        v = lambda x: r1 * (err_x + (x[0] - self.nt_x[0, self.nt - 1]) * (x[0] - self.nt_x[0, self.nt - 1]) +
                            (x[1] - self.nt_x[1, self.nt - 1]) * (x[1] - self.nt_x[1, self.nt - 1]) +
                            (x[2] - self.nt_x[2, self.nt - 1]) * (x[2] - self.nt_x[2, self.nt - 1]) +
                            (x[3] - self.nt_x[3, self.nt - 1]) * (x[3] - self.nt_x[3, self.nt - 1]) +
                            (x[4] - self.nt_x[4, self.nt - 1]) * (x[4] - self.nt_x[4, self.nt - 1]) +
                            (x[5] - self.nt_x[5, self.nt - 1]) * (x[5] - self.nt_x[5, self.nt - 1])) + \
                      r2 * (err_d + (np.sqrt((x[0] + 3) * (x[0] + 3) + (x[1] - 0.7) * (x[1] - 0.7) + (x[2] - 0.5) * (x[2] - 0.5)) - self.nt_ds[self.nt - 1]) *
                            (np.sqrt((x[0] + 3) * (x[0] + 3) + (x[1] - 0.7) * (x[1] - 0.7) + (x[2] - 0.5) * (x[2] - 0.5)) - self.nt_ds[self.nt - 1])) + \
                      r3 * (err_v + (np.sqrt(x[3] * x[3] + x[4] * x[4] + x[5] * x[5]) - self.nt_v[self.nt - 1]) *
                            (np.sqrt(x[3] * x[3] + x[4] * x[4] + x[5] * x[5]) - self.nt_v[self.nt - 1]))
        # np.exp(self.nt_v[self.nt - 1] / np.sqrt(x[3] * x[3] + x[4] * x[4] + x[5] * x[5])) *
        return v


    def con(self, args):
        x0min, x0max, x1min, x1max, x2min, x2max, x3min, x3max, x4min, x4max, x5min, x5max = args
        cons = ({'type': 'ineq', 'fun': lambda x: x[0] - x0min},
                {'type': 'ineq', 'fun': lambda x: -x[0] + x0max},
                {'type': 'ineq', 'fun': lambda x: x[1] - x1min},
                {'type': 'ineq', 'fun': lambda x: -x[1] + x1max},
                {'type': 'ineq', 'fun': lambda x: x[2] - x2min},
                {'type': 'ineq', 'fun': lambda x: -x[2] + x2max},
                {'type': 'ineq', 'fun': lambda x: x[3] - x3min},
                {'type': 'ineq', 'fun': lambda x: -x[3] + x3max},
                {'type': 'ineq', 'fun': lambda x: x[4] - x4min},
                {'type': 'ineq', 'fun': lambda x: -x[4] + x4max},
                {'type': 'ineq', 'fun': lambda x: x[5] - x5min},
                {'type': 'ineq', 'fun': lambda x: -x[5] + x5max})
        return cons

    # update the state of x
    def update_x(self, ax, ay, az, x):

        # the state transition matrix
        A = np.eye(6)
        B = np.zeros((6, 3))

        for i in range(3):
            A[i][i + 3] = self.dt
            B[i][i] = self.dt * self.dt * 0.5
            B[i + 3][i] = self.dt
        x = np.matrix(A) * x + B * np.matrix([ax, ay, az]).T
        return x

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
        mhe()
        rospy.spin()

    except KeyboardInterrupt:
        print("shutting down the node")
