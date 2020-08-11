#!/usr/bin/env python
# license removed for brevity

## 


import rospy
import rospkg
import time
import numpy as np
import scipy.linalg as la
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from math import *
from geometry_msgs.msg import Twist
from waypoint_manager.msg import bot_states
from relative_localization.msg import *

#### Time initialization ####

start_time = 0
print "start_time", start_time
t_prev     = start_time


## Global Variable initialization ##
omega_1    = 0.0
velocity_1 = 0.05
omega_2    = 0.0
velocity_2 = 0.1
omega_3    = 0.0
velocity_3 = 0.05

traj_flag = 2


def omega_gen():

    global omega_2, omega_1, omega_3
    global start_time

    # current_time = rospy.Time.from_sec(time.time())
    # current_time = current_time.to_sec()
    current_time = rospy.get_time()
    # print "current_time=", current_time
    dt           = (current_time - start_time)  
    print "dt=", dt

    if (traj_flag == 1): # same direction circle
       if (dt>=0 and dt<20):
            omega_2 = 0.1
       elif (dt>=20 and dt<40):
            omega_2 = 0.1
       elif (dt>=40 and dt<60):
            omega_2 = 0.1
       elif (dt>=60 and dt<80):
            omega_2 = 0.1
       elif (dt>=80):
            omega_2 = 0.1

    elif (traj_flag == 2): # S shaped trajectroy
        if (dt>=0 and dt<20):
            omega_1 = 0.01
            omega_2 = 0.05
            omega_3 = -0.01
        elif (dt>=20 and dt<40):
            omega_1 = -0.01
            omega_2 = -0.05
            omega_3 = 0.01
        elif (dt>=40 and dt<60):
            omega_1 = 0.01
            omega_2 = 0.05
            omega_3 = -0.01
        elif (dt>=60 and dt<80):
            omega_1 = -0.01
            omega_2 = -0.05
            omega_3 = 0.01
        elif (dt>=80 and dt<100):
            omega_1 = 0.01
            omega_2 = 0.05
            omega_3 = -0.01
        if (dt>=100 and dt<150):
            omega_1 = -0.01
            omega_2 = -0.05
            omega_3 = 0.01
        elif (dt>=150 and dt<200):
            omega_1 = 0.01
            omega_2 = 0.05
            omega_3 = -0.01
        elif (dt>=200 and dt<250):
            omega_1 = -0.01
            omega_2 = -0.05
            omega_3 = 0.01
        elif (dt>=250):
            omega_1 = 0.01
            omega_2 = 0.05
            omega_3 = -0.01
        

    elif (traj_flag == 3): # spline with low turn rate
        if (dt>=0 and dt<20):
            omega_2 = 0.0
        elif (dt>=20 and dt<40):
            omega_2 = -0.01
        elif (dt>=40 and dt<60):
            omega_2 = 0.01
        elif (dt>=60 and dt<80):
            omega_2 = -0.01
        elif (dt>=80):
            omega_2 = 0.0

    elif (traj_flag == 4): # alternate circular trajectory
       if (dt>=0 and dt<20):
            omega_2 = 0.1
       elif (dt>=20 and dt<40):
            omega_2 = 0.1
       elif (dt>=40 and dt<60):
            omega_2 = 0.1
       elif (dt>=60 and dt<80):
            omega_2 = 0.1
       elif (dt>=80 and dt<100):
            omega_2 = 0.1
       elif (dt>=100 and dt<120):
            omega_2 = 0.1
       elif (dt>=120 and dt<140):
            omega_2 = 0.1
       elif (dt>=140 and dt<160):
            omega_2 = 0.1
       elif (dt>=160 and dt<180):
            omega_2 = 0.1
       elif(dt>=180):
            omega_2 = 0.1

    elif (traj_flag == 5): # new traj - Kevin's case
        if (dt>=0 and dt<5):
            omega_2 = -0.05
        elif (dt>=5 and dt<10):
            omega_2 = 0.1
        elif (dt>=10 and dt<15):
            omega_2 = 0.1
        elif (dt>=15 and dt<20):
            omega_2 = -0.05
        elif (dt>=20 and dt<25):
            omega_2 = -0.05
        elif (dt>=35):
            omega_2 = 0.01

    return omega_1, omega_2, omega_3

def velocity_gen():

    global velocity
    global start_time

    current_time = rospy.get_time()
    dt           = current_time - start_time  

    if (dt>=0 and dt<20):
        velocity_2 = 0.05
    elif (dt>=20 and dt<40):
        velocity_2 = 0.05
    elif (dt>=40 and dt<60):
        velocity_2 = 0.05
    elif (dt>=60 and dt<80):
        velocity_2 = 0.05
    elif (dt>=80):
        velocity_2 = 0.05

    return  velocity_2

def callback1(data1):
   
    global traj_flag
    
    traj_flag = data1.flag1

def main():
    
    rospy.init_node('Trajectory_BOT1', anonymous=True)
    r = rospy.Rate(50)
    # start_time = rospy.Time.from_sec(time.time())
    # start_time = start_time.to_sec()
    start_time = rospy.get_time()

    ## setup publisher ##

    pub1 = rospy.Publisher('/robot0/mobile_base/commands/velocity', Twist, queue_size=1)
    pub2 = rospy.Publisher('/robot1/mobile_base/commands/velocity', Twist, queue_size=1)
    pub3 = rospy.Publisher('/robot2/mobile_base/commands/velocity', Twist, queue_size=1)

    ## setup subscriber ##

    sub1 = rospy.Subscriber('/flag', flags, callback1)

    while not rospy.is_shutdown():

        omega_1, omega_2, omega_3 = omega_gen()
        #velocity_2 = velocity_gen()

        cmd1 = Twist()
        cmd1.linear.x  = velocity_1
        cmd1.linear.y  = 0
        cmd1.linear.z  = 0
        cmd1.angular.x = 0
        cmd1.angular.y = 0
        cmd1.angular.z = omega_1

        cmd2 = Twist()
        cmd2.linear.x  = velocity_2
        cmd2.linear.y  = 0
        cmd2.linear.z  = 0
        cmd2.angular.x = 0
        cmd2.angular.y = 0
        cmd2.angular.z = omega_2

        cmd3 = Twist()
        cmd3.linear.x  = velocity_3
        cmd3.linear.y  = 0
        cmd3.linear.z  = 0
        cmd3.angular.x = 0
        cmd3.angular.y = 0
        cmd3.angular.z = omega_3

        pub1.publish(cmd1)
        pub2.publish(cmd2)
        pub3.publish(cmd3)

        r.sleep()

    rospy.loginfo('Trajectory BOT1 node has shutdown')
    rospy.signal_shutdown()

if __name__ == '__main__':
    main()




