#!/usr/bin/env python

import rospy
#import rospkg
import random
import numpy as np
import scipy.linalg as la
import math
import tf

from math import *
from crl_pairwise.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber


class range_meas_pub():

    def __init__(self):

        print "Entered Initialize - range measure multiple"

        self.sigma_rho = rospy.get_param("~sigma_rho")

        self.rng_pub10 = rospy.Publisher('/ekf_bot/range_meas10', sensor_rho_pairwise, queue_size=1)
        self.rng_pub20 = rospy.Publisher('/ekf_bot/range_meas20', sensor_rho_pairwise, queue_size=1)
        self.rng_pub12 = rospy.Publisher('/ekf_bot/range_meas12', sensor_rho_pairwise, queue_size=1)

        robot0_pose_sub = Subscriber("/ekf_bot/inertial_pose0", inert_pose)
        robot1_pose_sub = Subscriber("/ekf_bot/inertial_pose1", inert_pose)
        robot2_pose_sub = Subscriber("/ekf_bot/inertial_pose2", inert_pose)

        self.rng10 = sensor_rho_pairwise()
        self.rng20 = sensor_rho_pairwise()
        self.rng12 = sensor_rho_pairwise()

        ats = ApproximateTimeSynchronizer([robot0_pose_sub, robot1_pose_sub, robot2_pose_sub], queue_size=1000, slop=0.1,
                                          allow_headerless=False)

        ats.registerCallback(self.rng_pairwise)

        self.node_no = 1

    # self.true_pose_publish()

    # def rng_pairwise(self, data0, data1, data2):
    def rng_pairwise(self, data1, data0, data2):

        # print ("Entered callback - true relative pose", self.node_no)

        dx10 = data1.x_true - data0.x_true
        dy10 = data1.y_true - data0.y_true

        # dx20 = data2.x_true - data0.x_true
        # dy20 = data2.y_true - data0.y_true

        # dx12 = data1.x_true - data2.x_true
        # dy12 = data1.y_true - data2.y_true

        dx20 = data2.x_true - data1.x_true
        dy20 = data2.y_true - data1.y_true

        dx12 = data0.x_true - data2.x_true
        dy12 = data0.y_true - data2.y_true

        s_rho = np.power(self.sigma_rho, 2)

        rng_true10 = np.sqrt(np.power(dx10, 2) + np.power(dy10, 2))
        rng_meas10 = rng_true10 + s_rho

        rng_true20 = np.sqrt(np.power(dx20, 2) + np.power(dy20, 2))
        rng_meas20 = rng_true20 + s_rho

        rng_true12 = np.sqrt(np.power(dx12, 2) + np.power(dy12, 2))
        rng_meas12 = rng_true12 + s_rho

        self.rng10.header = data1.header
        self.rng10.meas_range = rng_meas10
        self.rng10.true_range = rng_true10

        self.rng20.header = data1.header
        self.rng20.meas_range = rng_meas20
        self.rng20.true_range = rng_true20

        self.rng12.header = data1.header
        self.rng12.meas_range = rng_meas12
        self.rng12.true_range = rng_true12

        self.rng_pub10.publish(self.rng10)
        self.rng_pub20.publish(self.rng20)
        self.rng_pub12.publish(self.rng12)

        r = rospy.Rate(1.0)
        r.sleep()

        # self.true_pose_pb.publish(self.rl_pose_true)
        self.node_no = self.node_no + 1

    # r.sleep()


if __name__ == '__main__':
    rospy.init_node("true_state_publisher", anonymous=False)
    try:
        verifyObj = range_meas_pub()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()

