#!/usr/bin/env python

import rospy
import rospkg
import random
import numpy as np
import scipy.linalg as la
import math
import tf

from math import *
from crl_multiple.msg import *
from crl_pairwise.msg import *
from message_filters import ApproximateTimeSynchronizer, Subscriber


class true_rel_pose_pub():

    def __init__(self):

        # print "Entered Initialize - true relative pose"

        self.true_pose_pb = rospy.Publisher('/ekf_bot/rel_true_multiple', rel_pose_multiple, queue_size=1)

        robot0_pose_sub = Subscriber("/ekf_bot/inertial_pose0", inert_pose)
        robot1_pose_sub = Subscriber("/ekf_bot/inertial_pose1", inert_pose)
        robot2_pose_sub = Subscriber("/ekf_bot/inertial_pose2", inert_pose)

        self.rl_pose_true = rel_pose_multiple()

        ats = ApproximateTimeSynchronizer([robot0_pose_sub, robot1_pose_sub, robot2_pose_sub], queue_size=1000,
                                          slop=0.001,
                                          allow_headerless=False)

        ats.registerCallback(self.true_pose_multiple_publish)

        self.node_no = 1

    # self.true_pose_publish()

    # def true_pose_multiple_publish(self, data0, data1, data2):
    def true_pose_multiple_publish(self, data1, data0, data2):

        # print ("Entered callback - true relative pose", self.node_no)

        dx10 = data1.x_true - data0.x_true
        dy10 = data1.y_true - data0.y_true
        delpsi10 = data1.psi_true - data0.psi_true

        dx20 = data2.x_true - data0.x_true
        dy20 = data2.y_true - data0.y_true
        delpsi20 = data2.psi_true - data0.psi_true

        c_psi0 = np.cos(data0.psi_true)
        s_psi0 = np.sin(data0.psi_true)

        delpsi10 = (delpsi10 + np.pi) % (2 * np.pi) - np.pi
        delpsi20 = (delpsi20 + np.pi) % (2 * np.pi) - np.pi

        Rot_mat0 = np.matrix([[c_psi0, s_psi0, 0], [-s_psi0, c_psi0, 0], [0, 0, 1]])

        diff_mat10 = np.matrix([[dx10], [dy10], [delpsi10]])
        diff_mat20 = np.matrix([[dx20], [dy20], [delpsi20]])

        relpose_true10 = np.dot(Rot_mat0, diff_mat10)
        relpose_true20 = np.dot(Rot_mat0, diff_mat20)

        self.rl_pose_true.header = data0.header
        self.rl_pose_true.px_true10 = relpose_true10.item(0)
        self.rl_pose_true.py_true10 = relpose_true10.item(1)
        self.rl_pose_true.delpsi_true10 = relpose_true10.item(2)
        self.rl_pose_true.px_true20 = relpose_true20.item(0)
        self.rl_pose_true.py_true20 = relpose_true20.item(1)
        self.rl_pose_true.delpsi_true20 = relpose_true20.item(2)

        self.true_pose_pb.publish(self.rl_pose_true)

        r = rospy.Rate(1.0)
        r.sleep()

        # self.true_pose_pb.publish(self.rl_pose_true)
        self.node_no = self.node_no + 1

    # r.sleep()


if __name__ == '__main__':
    rospy.init_node("true_state_publisher", anonymous=False)
    try:
        verifyObj = true_rel_pose_pub()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
