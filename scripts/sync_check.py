#!/usr/bin/env python3

"""
This script is an adaptation from :
https://github.com/danielsnider/follow_waypoints
"""

import threading
import rospy
import actionlib
from smach import State, StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, PointStamped, PoseStamped, Point, Quaternion
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, String
from tf import TransformListener
import tf
import math
import rospkg
import csv
import time
import os
from datetime import timedelta, datetime
from geometry_msgs.msg import PoseStamped
import dynamic_reconfigure.client
import numpy as np
from nav_msgs.msg import Path
import random

# smach.set_loggers(rospy.logdebug, rospy.logwarn, rospy.logdebug, rospy.logerr)

# Waypoints container
waypoints       = []
auxilary_data   = []
mission_report  = []

route_dict      = {}
route_sequence  = []
start_dict      = {}    
goal_dict       = {}

path_names_list = ['xy', 'yz', 'zx'] #['de', 'ef', 'fd'] #['ab', 'ba', 'ac', 'cb']

velocity_default = 1.0

# INDICES OF AUXILLARY DATA
AUX_WAYPOINT_INDEX  = 0
AUX_VELOCITY        = 1



def wait_for_start_journey():
    """thread worker function"""
    try:
        data_from_start_journey = rospy.wait_for_message('start_journey', String)

    except rospy.ROSInterruptException:
        return 'killed'

    rospy.loginfo('Received path READY start_journey')

    wait_time_at_goal = 3.0
    time.sleep(wait_time_at_goal)

    base_time = 1.0

    for i in range(10):
        rospy.loginfo("Tick with value " + str(i))
        variable_time = random.uniform(.7, 2.3)

        time.sleep(variable_time)





def main():
    rospy.init_node('sync_check')




if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        rospy.logwarn("Node shutdown")