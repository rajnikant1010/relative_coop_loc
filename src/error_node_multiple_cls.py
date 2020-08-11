#!/usr/bin/env python

import rospy
# import rospkg
import time
import random
import numpy as np
import scipy.linalg as la
import math

from math import *
from crl_pairwise.msg import *
from crl_multiple.msg import *
from message_filters import ApproximateTimeSynchronizer, Subscriber


class error_cls():

    def __init__(self):

        self.pub_err10 = rospy.Publisher('/ekf_bot/errors_rel10', error_st, queue_size=1)
        self.pub_err20 = rospy.Publisher('/ekf_bot/errors_rel20', error_st, queue_size=1)

        sub_true = Subscriber('/ekf_bot/rel_true_multiple', rel_pose_multiple)
        sub_est10 = Subscriber('/ekf_bot/rel_est10', rel_pose_est)
        sub_est20 = Subscriber('/ekf_bot/rel_est20', rel_pose_est)

        self.error_rel10 = error_st()
        self.error_rel20 = error_st()

        ats = ApproximateTimeSynchronizer([sub_true, sub_est10, sub_est20], queue_size=1, slop=0.1, allow_headerless=False)
        ats.registerCallback(self.err_calc)

    def pi2pi(self, angle):  # function for angle wrapping

        while ((math.fabs(angle) - np.pi) > 0.001):
            if (angle > np.pi):
                angle = angle - 2 * np.pi
            if (angle < -np.pi):
                angle = angle + 2 * np.pi
        return angle

    def err_calc(self, data1, data2, data3):

        print "in callback"

        print "px10 err = ", data1.px_true10 - data2.px_est
        print "py10 err = ", data1.py_true10 - data2.py_est
        print "delpsi10 err = ", data1.delpsi_true10 - data2.delpsi_est

        #self.error_rel10.header = data1.header
        self.error_rel10.err_px = data1.px_true10 - data2.px_est
        self.error_rel10.err_py = data1.py_true10 - data2.py_est
        self.error_rel10.err_delpsi = data1.delpsi_true10 - data2.delpsi_est
        self.error_rel10.err_delpsi = (self.error_rel10.err_delpsi + np.pi) % (2 * np.pi) - np.pi

        self.error_rel10.cov_px_p = 3 * np.power(data2.cov_px, 0.5)
        self.error_rel10.cov_px_n = -3 * np.power(data2.cov_px, 0.5)

        self.error_rel10.cov_py_p = 3 * np.power(data2.cov_py, 0.5)
        self.error_rel10.cov_py_n = -3 * np.power(data2.cov_py, 0.5)

        self.error_rel10.cov_delpsi_p = 3 * np.power(data2.cov_delpsi, 0.5)
        self.error_rel10.cov_delpsi_n = -3 * np.power(data2.cov_delpsi, 0.5)

        #self.error_rel20.header = data1.header
        self.error_rel20.err_px = data1.px_true20 - data3.px_est
        self.error_rel20.err_py = data1.py_true20 - data3.py_est
        self.error_rel20.err_delpsi = data1.delpsi_true20 - data3.delpsi_est
        self.error_rel20.err_delpsi = (self.error_rel20.err_delpsi + np.pi) % (2 * np.pi) - np.pi

        self.error_rel20.cov_px_p = 3 * np.power(data3.cov_px, 0.5)
        self.error_rel20.cov_px_n = -3 * np.power(data3.cov_px, 0.5)

        self.error_rel20.cov_py_p = 3 * np.power(data3.cov_py, 0.5)
        self.error_rel20.cov_py_n = -3 * np.power(data3.cov_py, 0.5)

        self.error_rel20.cov_delpsi_p = 3 * np.power(data3.cov_delpsi, 0.5)
        self.error_rel20.cov_delpsi_n = -3 * np.power(data3.cov_delpsi, 0.5)

        self.pub_err10.publish(self.error_rel10)
        self.pub_err20.publish(self.error_rel20)


if __name__ == '__main__':
    rospy.init_node("error_calc", anonymous=False)
    try:
        verifyObj = error_cls()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
