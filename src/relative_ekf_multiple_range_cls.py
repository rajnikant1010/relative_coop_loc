#!/usr/bin/env python

import rospy
import rospkg
import time
import random
import numpy as np
import numpy.linalg as np_lin
import scipy.linalg as la
import math
import tf

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from crl_multiple.msg import *
from crl_pairwise.msg import *
from message_filters import Subscriber, ApproximateTimeSynchronizer


class ekf_cls():

    def __init__(self):

        self.sigma_px = rospy.get_param(
            "~sigma_px")  # tilde is for a local variable accessible in that node only, for global variables no tilde required
        self.sigma_py = rospy.get_param("~sigma_py")
        self.sigma_del_psi = rospy.get_param("~sigma_del_psi")
        self.sigma_v = rospy.get_param("~sigma_v")
        self.sigma_w = rospy.get_param("~sigma_w")
        self.num_iter = rospy.get_param("~num_iter")
        self.dt = rospy.get_param("~time_step")
        self.robot0_vel = rospy.get_param("~robot0_vel")
        self.robot1_vel = rospy.get_param("~robot1_vel")
        self.robot2_vel = rospy.get_param("~robot2_vel")
        self.beta10 = rospy.get_param("~beta1")
        self.beta20 = rospy.get_param("~beta2")
        self.CRLFlag = rospy.get_param("~CRLFlag")

        print("Entered Initialize - Multi vehicle EKF", self.sigma_px)

        self.ekf_pb10 = rospy.Publisher('/ekf_bot/rel_est10', rel_pose_est, queue_size=1)
        self.ekf_pb20 = rospy.Publisher('/ekf_bot/rel_est20', rel_pose_est, queue_size=1)

        self.rl_pose_est10 = rel_pose_est()
        self.rl_pose_est20 = rel_pose_est()

        robot0_pose_sub = Subscriber("/ekf_bot/inertial_pose0", inert_pose)
        robot1_pose_sub = Subscriber("/ekf_bot/inertial_pose1", inert_pose)
        robot2_pose_sub = Subscriber("/ekf_bot/inertial_pose2", inert_pose)
        imu0 = Subscriber('/robot0/mobile_base/sensors/imu_data', Imu)
        imu1 = Subscriber('/robot1/mobile_base/sensors/imu_data', Imu)
        imu2 = Subscriber('/robot2/mobile_base/sensors/imu_data', Imu)
        true_rel_pose = Subscriber('/ekf_bot/rel_true_multiple', rel_pose_multiple)
        meas_node10 = Subscriber('/ekf_bot/range_meas10', sensor_rho_pairwise)
        meas_node20 = Subscriber('/ekf_bot/range_meas20', sensor_rho_pairwise)
        meas_node12 = Subscriber('/ekf_bot/range_meas12', sensor_rho_pairwise)

        self.node_no = 1
        self.Phat = np.identity(3)
        self.xhat = np.identity(3)
        self.Qu = np.matrix([[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]])

        ats = ApproximateTimeSynchronizer([robot0_pose_sub, robot1_pose_sub, robot2_pose_sub, imu0, imu1, imu2, true_rel_pose, meas_node10, meas_node20, meas_node12], queue_size=1,
                                          slop=0.1, allow_headerless=False)
        ats.registerCallback(self.ekf_func)

    def pi2pi(self, angle):  # function for angle wrapping

        while ((math.fabs(angle) - np.pi) > 0.001):
            if (angle > np.pi):
                angle = angle - 2 * np.pi
            if (angle < -np.pi):
                angle = angle + 2 * np.pi
            return angle

    # def ekf_func(self, data0, data1, data2, data3, data4, data5, data6, data7, data8, data9):
    def ekf_func(self, data1, data0, data2, data3, data4, data5, data6, data7, data8, data9):

        print("entered callback")

        ### robot 0 pose callback ###

        self.robot0_x = data0.x_true
        self.robot0_y = data0.y_true
        self.robot0_psi = data0.psi_true
        # self.robot0_omega      = data0.w

        ### robot 1 pose callback ###

        self.robot1_x = data1.x_true
        self.robot1_y = data1.y_true
        self.robot1_psi = data1.psi_true
        # self.robot1_omega      = data1.w

        ### robot 2 pose callback ###

        self.robot2_x = data2.x_true
        self.robot2_y = data2.y_true
        self.robot2_psi = data2.psi_true
        # self.robot1_omega      = data1.w

        self.robot0_omega = data3.angular_velocity.z
        self.robot1_omega = data4.angular_velocity.z
        self.robot2_omega = data5.angular_velocity.z

        ### relative pose callback ###

        self.true_px10 = data6.px_true10
        self.true_py10 = data6.py_true10
        self.true_delpsi10 = data6.delpsi_true10
        self.true_px20 = data6.px_true20
        self.true_py20 = data6.py_true20
        self.true_delpsi20 = data6.delpsi_true20

        ### range measurement callback ###

        self.meas_range10 = data7.meas_range
        self.true_range10 = data7.true_range

        self.meas_range20 = data8.meas_range
        self.true_range20 = data8.true_range

        self.meas_range12 = data9.meas_range
        self.true_range12 = data9.true_range

        if (self.node_no == 1):

            print("entered if")

            s_px = np.power(self.sigma_px, 2)
            s_py = np.power(self.sigma_py, 2)
            s_dpsi = np.power(self.sigma_del_psi, 2)
            s_v = np.power(self.sigma_v, 2)
            s_w = np.power(self.sigma_w, 2)

            self.xhat = np.matrix([[self.true_px10], [self.true_py10], [self.true_delpsi10], [self.true_px20], [self.true_py20], [self.true_delpsi20]])
            + np.matrix([[self.sigma_px*random.random()], [self.sigma_py*random.random()], [self.sigma_del_psi*random.random()]])

            self.Phat = np.matrix([[s_px, 0, 0, 0, 0, 0], [0, s_py, 0, 0, 0, 0], [0, 0, s_dpsi, 0, 0, 0], [0, 0, 0, s_px, 0, 0], [0, 0, 0, 0, s_py, 0], [0, 0, 0, 0, 0, s_dpsi]])

            self.Qu = np.matrix([[s_v, 0, 0, 0, 0, 0], [0, s_w, 0, 0, 0, 0], [0, 0, s_v, 0, 0, 0], [0, 0, 0, s_w, 0, 0], [0, 0, 0, 0, s_v, 0], [0, 0, 0, 0, 0, s_w]])

            self.node_no = self.node_no + 1

            self.prev_time = data2.header.stamp.to_nsec()
            self.TimeCRLStart = data2.header.stamp.to_nsec()

        else:

            self.node_no = self.node_no + 1
            print "entered else", self.node_no

            self.dt = (data2.header.stamp.to_nsec() - self.prev_time) * 1e-9
            print("dt ", self.dt)
            self.prev_time = data2.header.stamp.to_nsec()
            self.CurrentTime = (data2.header.stamp.to_nsec() - self.TimeCRLStart) * 1e-9

            PxNoise = np.power(self.sigma_px, 2)
            PyNoise = np.power(self.sigma_py, 2)
            DelPsiNoise = np.power(self.sigma_del_psi, 2)

            for i in xrange(int(self.num_iter)):
                c_psi10 = np.cos(self.xhat.item(2))
                s_psi10 = np.sin(self.xhat.item(2))
                c_psi20 = np.cos(self.xhat.item(5))
                s_psi20 = np.sin(self.xhat.item(5))

                fx = np.matrix([[self.robot0_omega * self.xhat.item(1) + self.robot1_vel * c_psi10 - self.robot0_vel],
                                [-self.robot0_omega * self.xhat.item(0) + self.robot1_vel * s_psi10],
                                [self.robot1_omega - self.robot0_omega],
                                [self.robot0_omega * self.xhat.item(4) + self.robot2_vel * c_psi20 - self.robot0_vel],
                                [-self.robot0_omega * self.xhat.item(3) + self.robot2_vel * s_psi20],
                                [self.robot2_omega - self.robot0_omega]
                                ])
                F = np.matrix([[0, self.robot0_omega, -self.robot1_vel * s_psi10, 0, 0, 0],
                               [-self.robot0_omega, 0, self.robot1_vel * c_psi10, 0, 0, 0],
                               [0, 0, 0, 0, 0, 0],
                               [0, 0, 0, 0, self.robot0_omega, -self.robot2_vel * s_psi20],
                               [0, 0, 0, -self.robot0_omega, 0, self.robot2_vel * c_psi20],
                               [0, 0, 0, 0, 0, 0]
                               ])

                G = np.matrix([[-1, self.xhat.item(1), np.cos(self.xhat.item(2)), 0, 0, 0],
                               [0, -self.xhat.item(0), np.sin(self.xhat.item(2)), 0, 0, 0],
                               [0, -1, 0, 1, 0, 0],
                               [-1, self.xhat.item(4), 0, 0, np.cos(self.xhat.item(5)), 0],
                               [0, -self.xhat.item(3), 0, 0, np.sin(self.xhat.item(5)), 0],
                               [0, -1, 0, 0, 0, 1]
                               ])

                if self.CurrentTime > 20:
                    Q_term = np.dot(G, self.Qu)
                    Q = np.dot(Q_term, np.transpose(G))
                else:
                    Q = np.matrix([[PxNoise, 0, 0, 0, 0, 0],
                                   [0, PyNoise, 0, 0, 0, 0],
                                   [0, 0, DelPsiNoise, 0, 0, 0],
                                   [0, 0, 0, PxNoise, 0, 0],
                                   [0, 0, 0, 0, PyNoise, 0],
                                   [0, 0, 0, 0, 0, DelPsiNoise]
                                 ])

                self.xhat = self.xhat + np.dot(np.divide(self.dt, self.num_iter), fx)
                
                cov_term = np.dot(F, self.Phat) + np.dot(self.Phat, np.transpose(F)) + Q
                self.Phat = self.Phat + np.dot(np.divide(self.dt, self.num_iter), cov_term)

                self.xhat[2] = (self.xhat.item(2) + np.pi) % (2 * np.pi) - np.pi
                self.xhat[5] = (self.xhat.item(5) + np.pi) % (2 * np.pi) - np.pi

            alpha10 = 1 - self.beta10
            alpha20 = 1 - self.beta20

            dx_est10 = self.xhat.item(0)
            dy_est10 = self.xhat.item(1)

            rng_est10 = np.sqrt(np.power(dx_est10, 2) + np.power(dy_est10, 2))

            if self.meas_range10 != -9999:

                C10_1 = np.divide(dx_est10, rng_est10)
                C10_2 = np.divide(dy_est10, rng_est10)

                C_rho_px10 = np.matrix([C10_1, C10_2, 0, 0, 0, 0])
                L10_term1_px = np.dot(self.Phat, np.transpose(C_rho_px10))
                L10_term2_px = np.dot(C_rho_px10, L10_term1_px)
                L10_term4_px = self.sigma_px + L10_term2_px
                L10_term5_px = L10_term4_px.item(0)
                L10_term6_px = L10_term1_px * np.divide(1, L10_term5_px)

                # print "L6", L10_term6_px

                Phat_temp1_px = np.dot(L10_term6_px, C_rho_px10)
                Phat_temp2_px = np.identity(6) - Phat_temp1_px
                Phat_old = self.Phat

                self.Phat = np.dot(Phat_temp2_px, self.Phat)
                self.Phat[2, 2] = np.power(alpha10, 2) * Phat_old.item((2, 2)) + (1 - np.power(alpha10, 2)) * self.Phat.item((2, 2))
                self.Phat[5, 5] = np.power(alpha20, 2) * Phat_old.item((5, 5)) + (1 - np.power(alpha20, 2)) * self.Phat.item((5, 5))

                xhat_old = self.xhat
                meas_diff_px10 = self.meas_range10 - rng_est10
                xhat = self.xhat + np.dot(L10_term6_px, meas_diff_px10)
                xhat[2] = alpha10 * xhat_old.item(2) + (1 - alpha10) * self.xhat.item(2)
                xhat[5] = alpha20 * xhat_old.item(5) + (1 - alpha20) * self.xhat.item(5)

                self.xhat[2] = (self.xhat.item(2) + np.pi) % (2 * np.pi) - np.pi
                self.xhat[5] = (self.xhat.item(5) + np.pi) % (2 * np.pi) - np.pi

            dx = self.xhat.item(0)
            dy = self.xhat.item(1)
            delta_psi = self.xhat.item(2)

            # if self.true_delpsi10 != -9999 and self.CRLFlag == 0:
            #
            #     delta_psi_10 = self.xhat.item(2)
            #     hr1_d10 = 1
            #     Hr_d10 = np.matrix([0, 0, hr1_d10, 0, 0, 0])
            #
            #     l1_d10 = self.sigma_del_psi + np.dot(Hr_d10, np.dot(self.Phat, np.transpose(Hr_d10)))
            #     l1_sclr_d10 = l1_d10.item(0)
            #     l1_inv_d10 = 1 / l1_sclr_d10
            #     L_d10 = l1_inv_d10 * np.dot(self.Phat, np.transpose(Hr_d10))
            #
            #     Phat_old = self.Phat
            #     pTemp_d10 = np.identity(6) - np.dot(L_d10, Hr_d10)
            #     self.Phat = np.dot(pTemp_d10, self.Phat)
            #     self.Phat[2, 2] = np.power(alpha10, 2) * Phat_old.item((2, 2)) + (1 - np.power(alpha10, 2)) * self.Phat.item((2, 2))
            #
            #     self.Phat[5, 5] = np.power(alpha20, 2) * Phat_old.item((5, 5)) + (1 - np.power(alpha20, 2)) * self.Phat.item((5, 5))
            #
            #     mag_sensor_fake = self.true_delpsi10
            #     meas_diff_d10 = mag_sensor_fake - delta_psi_10
            #
            #     meas_diff_d10 = (meas_diff_d10 + np.pi) % (2 * np.pi) - np.pi
            #
            #     meas_diff_d10_wr = meas_diff_d10
            #
            #     xhat_old = self.xhat
            #
            #     self.xhat = self.xhat + np.dot(L_d10, meas_diff_d10_wr)
            #     self.xhat[2] = alpha10 * xhat_old.item(2) + (1 - alpha10) * self.xhat.item(2)
            #     self.xhat[5] = alpha20 * xhat_old.item(5) + (1 - alpha20) * self.xhat.item(5)
            #
            #     self.xhat[2] = (self.xhat.item(2) + np.pi) % (2 * np.pi) - np.pi
            #     self.xhat[5] = (self.xhat.item(5) + np.pi) % (2 * np.pi) - np.pi

            dx_est20 = self.xhat.item(3)
            dy_est20 = self.xhat.item(4)

            rng_est20 = np.sqrt(np.power(dx_est20, 2) + np.power(dy_est20, 2))

            if self.meas_range20 != -9999:

                C20_4 = np.divide(dx_est20, rng_est20)
                C20_5 = np.divide(dy_est20, rng_est20)

                C_rho_px20 = np.matrix([0, 0, 0, C20_4, C20_5, 0])
                L20_term1_px = np.dot(self.Phat, np.transpose(C_rho_px20))
                L20_term2_px = np.dot(C_rho_px20, L20_term1_px)
                L20_term4_px = self.sigma_px + L20_term2_px
                L20_term5_px = L20_term4_px.item(0)
                L20_term6_px = L20_term1_px * np.divide(1, L20_term5_px)

                # print "L6", L20_term6_px

                Phat_temp1_px = np.dot(L20_term6_px, C_rho_px20)
                Phat_temp2_px = np.identity(6) - Phat_temp1_px
                Phat_old = self.Phat

                self.Phat = np.dot(Phat_temp2_px, self.Phat)
                self.Phat[2, 2] = np.power(alpha10, 2) * Phat_old.item((2, 2)) + (1 - np.power(alpha10, 2)) * self.Phat.item((2, 2))

                self.Phat[5, 5] = np.power(alpha20, 2) * Phat_old.item((5, 5)) + (1 - np.power(alpha20, 2)) * self.Phat.item((5, 5))

                xhat_old = self.xhat
                meas_diff_px20 = self.meas_range20 - rng_est20
                xhat = self.xhat + np.dot(L20_term6_px, meas_diff_px20)
                xhat[2] = alpha10 * xhat_old.item(2) + (1 - alpha10) * self.xhat.item(2)
                xhat[5] = alpha20 * xhat_old.item(5) + (1 - alpha20) * self.xhat.item(2)

                self.xhat[2] = (self.xhat.item(2) + np.pi) % (2 * np.pi) - np.pi
                self.xhat[5] = (self.xhat.item(5) + np.pi) % (2 * np.pi) - np.pi

                dx = self.xhat.item(3)
                dy = self.xhat.item(4)
                delta_psi = self.xhat.item(5)

                # if self.true_delpsi20 != -9999 and self.CRLFlag == 0:
                #
                #     delta_psi_20 = self.xhat.item(5)
                #     hr1_d20 = 1
                #     Hr_d20 = np.matrix([0, 0, 0, 0, 0, hr1_d20])
                #
                #     l1_d20 = self.sigma_del_psi + np.dot(Hr_d20, np.dot(self.Phat, np.transpose(Hr_d20)))
                #     l1_sclr_d20 = l1_d20.item(0)
                #     l1_inv_d20 = 1 / l1_sclr_d20
                #     L_d20 = l1_inv_d20 * np.dot(self.Phat, np.transpose(Hr_d20))
                #
                #     Phat_old = self.Phat
                #     pTemp_d20 = np.identity(6) - np.dot(L_d20, Hr_d20)
                #     self.Phat = np.dot(pTemp_d20, self.Phat)
                #     self.Phat[2, 2] = np.power(alpha10, 2) * Phat_old.item((2, 2)) + (1 - np.power(alpha10, 2)) * self.Phat.item((2, 2))
                #     self.Phat[5, 5] = np.power(alpha20, 2) * Phat_old.item((5, 5)) + (1 - np.power(alpha20, 2)) * self.Phat.item((5, 5))
                #
                #     mag_sensor_fake = self.true_delpsi20
                #     meas_diff_d20 = mag_sensor_fake - delta_psi_20
                #
                #     meas_diff_d20 = (meas_diff_d20 + np.pi) % (2 * np.pi) - np.pi
                #
                #     meas_diff_d20_wr = meas_diff_d20
                #
                #     xhat_old = self.xhat
                #
                #     self.xhat = self.xhat + np.dot(L_d20, meas_diff_d20_wr)
                #     self.xhat[2] = alpha10 * xhat_old.item(2) + (1 - alpha10) * self.xhat.item(2)
                #     self.xhat[5] = alpha20 * xhat_old.item(5) + (1 - alpha20) * self.xhat.item(5)
                #
                #     self.xhat[2] = (self.xhat.item(2) + np.pi) % (2 * np.pi) - np.pi
                #     self.xhat[5] = (self.xhat.item(5) + np.pi) % (2 * np.pi) - np.pi

                dx_est12 = self.xhat.item(0) - self.xhat.item(3)
                dy_est12 = self.xhat.item(1) - self.xhat.item(4)

                rng_est12 = np.sqrt(np.power(dx_est12, 2) + np.power(dy_est12, 2))

                self.CRLTime = (data2.header.stamp.to_nsec() - self.TimeCRLStart) * 1e-9
                print "CRL Time = ", self.CRLTime

                if self.meas_range12 != -9999 and self.CRLTime > 20 and self.CRLFlag == 1:
                    print "entered CRL"

                    C12_x = np.divide(dx_est12, rng_est12)
                    C12_y = np.divide(dy_est12, rng_est12)

                    C_rho_px12 = np.matrix([C12_x, C12_y, 0, -C12_x, -C12_y, 0])
                    L12_term1_px = np.dot(self.Phat, np.transpose(C_rho_px12))
                    L12_term2_px = np.dot(C_rho_px12, L12_term1_px)
                    L12_term4_px = self.sigma_px + L12_term2_px
                    L12_term5_px = L12_term4_px.item(0)
                    L12_term6_px = L12_term1_px * np.divide(1, L12_term5_px)

                    # print "L6", L20_term6_px

                    Phat_temp1_px = np.dot(L12_term6_px, C_rho_px12)
                    Phat_temp2_px = np.identity(6) - Phat_temp1_px
                    Phat_old = self.Phat

                    self.Phat = np.dot(Phat_temp2_px, self.Phat)
                    self.Phat[2, 2] = np.power(alpha10, 2) * Phat_old.item((2, 2)) + (1 - np.power(alpha10, 2)) * self.Phat.item((2, 2))

                    self.Phat[5, 5] = np.power(alpha20, 2) * Phat_old.item((5, 5)) + (1 - np.power(alpha20, 2)) * self.Phat.item((5, 5))

                    xhat_old = self.xhat
                    meas_diff_px12 = self.meas_range12 - rng_est12
                    xhat = self.xhat + np.dot(L12_term6_px, meas_diff_px12)
                    xhat[2] = alpha10 * xhat_old.item(2) + (1 - alpha10) * self.xhat.item(2)
                    xhat[5] = alpha20 * xhat_old.item(5) + (1 - alpha20) * self.xhat.item(5)

                    self.xhat[2] = (self.xhat.item(2) + np.pi) % (2 * np.pi) - np.pi
                    self.xhat[5] = (self.xhat.item(5) + np.pi) % (2 * np.pi) - np.pi

            self.rl_pose_est10.header = data0.header

            self.rl_pose_est10.px_est = self.xhat.item(0)
            self.rl_pose_est10.py_est = self.xhat.item(1)
            self.rl_pose_est10.delpsi_est = self.xhat.item(2)

            self.rl_pose_est10.cov_px = self.Phat.item((0, 0))
            self.rl_pose_est10.cov_py = self.Phat.item((1, 1))
            self.rl_pose_est10.cov_delpsi = self.Phat.item((2, 2))

            self.rl_pose_est20.header = data0.header

            self.rl_pose_est20.px_est = self.xhat.item(3)
            self.rl_pose_est20.py_est = self.xhat.item(4)
            self.rl_pose_est20.delpsi_est = self.xhat.item(5)

            self.rl_pose_est20.cov_px = self.Phat.item((3, 3))
            self.rl_pose_est20.cov_py = self.Phat.item((4, 4))
            self.rl_pose_est20.cov_delpsi = self.Phat.item((5, 5))

            self.ekf_pb10.publish(self.rl_pose_est10)
            self.ekf_pb20.publish(self.rl_pose_est20)

        # r = rospy.Rate(1.0)
    # r.sleep()


if __name__ == '__main__':
    rospy.init_node("ekf_relative_range", anonymous=False)
    try:
        verifyObj = ekf_cls()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
