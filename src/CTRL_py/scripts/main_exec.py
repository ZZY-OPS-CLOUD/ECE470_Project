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
from ur3interface import *

Q13 = [120 * pi / 180.0, -56 * pi / 180.0, 124 * pi / 180.0, -158 * pi / 180.0, -90 * pi / 180.0, 0 * pi / 180.0]

def main():
    rospy.init_node('CTRL_node')
    ur3IF = UR3Interface()
    rospy.loginfo('hello, world')
    loop_rate = rospy.Rate(20)
    ur3IF.set_angle(Q13, 4.0, 4.0)


if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
