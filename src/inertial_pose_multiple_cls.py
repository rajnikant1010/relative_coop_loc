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
from std_msgs.msg import *
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber


class true_rel_pose_pub():

    def __init__(self):

        # print "Entered Initialize - true relative pose"

        self.true_pose_in0 = rospy.Publisher('/ekf_bot/inertial_pose0', inert_pose, queue_size=1)
        self.true_pose_in1 = rospy.Publisher('/ekf_bot/inertial_pose1', inert_pose, queue_size=1)
        self.true_pose_in2 = rospy.Publisher('/ekf_bot/inertial_pose2', inert_pose, queue_size=1)

        robot0_pose_sub = Subscriber("/robot0/odom_truth", Odometry)
        robot1_pose_sub = Subscriber("/robot1/odom_truth", Odometry)
        robot2_pose_sub = Subscriber("/robot2/odom_truth", Odometry)
        imu0 = Subscriber('/robot0/mobile_base/sensors/imu_data', Imu)
        imu1 = Subscriber('/robot1/mobile_base/sensors/imu_data', Imu)
        imu2 = Subscriber('/robot2/mobile_base/sensors/imu_data', Imu)

        self.inert_pose0 = inert_pose()
        self.inert_pose1 = inert_pose()
        self.inert_pose2 = inert_pose()

        ats = ApproximateTimeSynchronizer([robot0_pose_sub, robot1_pose_sub, robot2_pose_sub, imu0, imu1, imu2], queue_size=1, slop=0.1,
                                          allow_headerless=False)
        ats.registerCallback(self.true_inert_publish)

        self.node_no = 1

    ## wrapping function ##

    def pi2pi(self, angle):  # function for angle wrapping

        while ((math.fabs(angle) - np.pi) > 0.001):
            if (angle > np.pi):
                angle = angle - 2 * np.pi
            if (angle < -np.pi):
                angle = angle + 2 * np.pi
            return angle

    # self.true_pose_publish()

    # def true_inert_publish(self, data0, data1, data2, data3, data4, data5):
    def true_inert_publish(self, data1, data0, data2, data3, data4, data5):

        # print "entered callback", self.node_no

        x0 = np.divide(data0.pose.pose.position.x, 1)
        x1 = np.divide(data1.pose.pose.position.x, 1)
        x2 = np.divide(data2.pose.pose.position.x, 1)
        y0 = np.divide(data0.pose.pose.position.y, 1)
        y1 = np.divide(data1.pose.pose.position.y, 1)
        y2 = np.divide(data2.pose.pose.position.y, 1)

        xq0 = data0.pose.pose.orientation.x
        yq0 = data0.pose.pose.orientation.y
        zq0 = data0.pose.pose.orientation.z
        wq0 = data0.pose.pose.orientation.w

        orientation0 = (xq0, yq0, zq0, wq0)
        euler_ang0 = tf.transformations.euler_from_quaternion(orientation0)
        phi0 = euler_ang0[0]
        theta0 = euler_ang0[1]
        psi0 = euler_ang0[2]

        xq1 = data1.pose.pose.orientation.x
        yq1 = data1.pose.pose.orientation.y
        zq1 = data1.pose.pose.orientation.z
        wq1 = data1.pose.pose.orientation.w

        orientation1 = (xq1, yq1, zq1, wq1)
        euler_ang1 = tf.transformations.euler_from_quaternion(orientation1)
        phi1 = euler_ang1[0]
        theta1 = euler_ang1[1]
        psi1 = euler_ang1[2]

        xq2 = data2.pose.pose.orientation.x
        yq2 = data2.pose.pose.orientation.y
        zq2 = data2.pose.pose.orientation.z
        wq2 = data2.pose.pose.orientation.w

        orientation2 = (xq2, yq2, zq2, wq2)
        euler_ang2 = tf.transformations.euler_from_quaternion(orientation2)
        phi2 = euler_ang2[0]
        theta2 = euler_ang2[1]
        psi2 = euler_ang2[2]

        self.inert_pose0.header = data0.header
        self.inert_pose0.x_true = x0
        self.inert_pose0.y_true = y0
        self.inert_pose0.psi_true = psi0
        self.inert_pose0.w = data3.angular_velocity.z

        self.inert_pose1.header = data0.header
        self.inert_pose1.x_true = x1
        self.inert_pose1.y_true = y1
        self.inert_pose1.psi_true = psi1
        self.inert_pose1.w = data4.angular_velocity.z

        self.inert_pose2.header = data0.header
        self.inert_pose2.x_true = x2
        self.inert_pose2.y_true = y2
        self.inert_pose2.psi_true = psi2
        self.inert_pose2.w = data5.angular_velocity.z

        self.true_pose_in0.publish(self.inert_pose0)
        self.true_pose_in1.publish(self.inert_pose1)
        self.true_pose_in2.publish(self.inert_pose2)

        self.node_no = self.node_no + 1
        r = rospy.Rate(1.0)
        r.sleep()


if __name__ == '__main__':
    rospy.init_node("true_state_publisher_multiple", anonymous=False)
    try:
        verifyObj = true_rel_pose_pub()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()

