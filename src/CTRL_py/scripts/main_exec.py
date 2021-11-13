#!/usr/bin/env python

import os
import argparse
import copy
import time
import rospy
import rospkg
import numpy as np
import yaml
import sys
from math import pi
from Ur3Interface import *

# Position for UR3 not blocking the camera
go_away = [270*PI/180.0, -90*PI/180.0, 90*PI/180.0, -90*PI/180.0, -90*PI/180.0, 135*PI/180.0]

# Store world coordinates of sanitizer and the target place
xw_yw_S = []
xw_yw_M = []
# 20Hz
SPIN_RATE = 20

# UR3 home location
home = [0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0]

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0.0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False

image_shape_define = False


"""
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def input_callback(msg):

    global digital_in_0
    digital_in_0 = msg.DIGIN
    digital_in_0 = digital_in_0 & 1 # Only look at least significant bit, meaning index 0
    global suction_state

    suction_state = msg.DIGIN


"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True


"""
Function to control the suction cup on/off
"""
def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            #rospy.loginfo("Goal is reached!")
            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


"""
Move robot arm from one position to another
"""
def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
            #rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg_gg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


def move_block(pub_cmd, loop_rate, start_xw_yw_zw, target_xw_yw_zw,end_xw_yw_zw, vel, accel):

    """
    start_xw_yw_zw: where to pick up a block in global coordinates
    target_xw_yw_zw: where to put the block at a given place in global coordinates
    end_xw_yw_zw: where to place a block in global coordinates

    hint: you will use lab_invk(), gripper(), move_arm() functions to
    pick and place a block

    """
    # ========================= Student's code starts here =========================

    # global variable1
    # global variable2
    angle_start = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2], 0)
    angle_target = lab_invk(target_xw_yw_zw[0], target_xw_yw_zw[1],target_xw_yw_zw[2] , 0)
    angle_end = lab_invk(end_xw_yw_zw[0], end_xw_yw_zw[1], end_xw_yw_zw[2], 0)
    angle_start_upper = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2]+0.1, 0)
    angle_end_upper = lab_invk(end_xw_yw_zw[0], end_xw_yw_zw[1], end_xw_yw_zw[2]+0.1, 0)

    vel = 4.0
    accel = 4.0
    #Define locations from left to right as 1, 2, 3. Define height from low to high as 1 2 3
    move_arm(pub_cmd, loop_rate, angle_start_upper, vel, accel)
    move_arm(pub_cmd, loop_rate, angle_start, vel, accel)
    gripper(pub_cmd, loop_rate, suction_on)
    time.sleep(1.0)
    if (suction_state == False):
        gripper(pub_cmd, loop_rate, suction_off)
        move_arm(pub_cmd, loop_rate, go_away, vel, accel)
        rospy.loginfo("Block Missing, Mission Skipped!")
    else:
        move_arm(pub_cmd, loop_rate, angle_start_upper, vel, accel)
        move_arm(pub_cmd, loop_rate, angle_target, vel, accel)
        time.sleep(2.0)
        move_arm(pub_cmd, loop_rate, angle_end_upper, vel, accel)
        move_arm(pub_cmd, loop_rate, angle_end, vel, accel)
        gripper(pub_cmd, loop_rate, suction_off)
        move_arm(pub_cmd, loop_rate, angle_end_upper, vel, accel)
    time.sleep(1.0)

    error = 0

    # ========================= Student's code ends here ===========================

    return error


"""
Program run from here
"""
def main():

    global go_away
    global xw_yw_S
    global xw_yw_M
    # global variable1
    # global variable2

    # Initialize ROS node
    #rospy.init_node('lab5node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position & ur3/gripper_input and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)
    sub_input = rospy.Subscriber('ur3/gripper_input', gripper_input, input_callback)

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    # Initialize the rate to publish to ur3/command
    loop_rate = rospy.Rate(SPIN_RATE)

    vel = 4.0
    accel = 4.0
    move_arm(pub_command, loop_rate, go_away, vel, accel)

    ic = ImageConverter(SPIN_RATE)
    time.sleep(5)

    
    """
    Initial and final postion should be xw_yw_S, medimum place should be xw_yw_M which should be detected by the camera.
    """
    move_block(pub_command, loop_rate, xw_yw_S, xw_yw_M,xw_yw_S, 4.0, 4.0)
    move_arm(pub_command, loop_rate, go_away, vel, accel)
    rospy.loginfo("Task Completed!")
    print("Use Ctrl+C to exit program")
    rospy.spin()


#Q13 = [120 * pi / 180.0, -56 * pi / 180.0, 124 * pi / 180.0, -158 * pi / 180.0, -90 * pi / 180.0, 0 * pi / 180.0]


#def main():

    #rospy.init_node('CTRL_node')
    #ur3IF = UR3Interface()
    #rospy.loginfo('hello, world')
    #loop_rate = rospy.Rate(20)
    #ur3IF.set_angle(Q13, 4.0, 4.0)


if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
