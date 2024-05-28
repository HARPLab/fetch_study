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

from sound_play.libsoundplay import SoundClient

# smach.set_loggers(rospy.logdebug, rospy.logwarn, rospy.logdebug, rospy.logerr)

# Waypoints container
waypoints       = []
auxilary_data   = []
mission_report  = []
mission_report_short  = []

route_dict      = {}
route_sequence  = []
start_dict      = {}    
goal_dict       = {}

path_names_list = ['CF-early', 'FE-early', 'EF-early', 'FD-early', 'DE-early', 'EA-early', 'AC-early-obs', 'CD-early-obs', 'DA-early', 'AE-early', 'EC-early', 'CE-early', 'ED-early', 'DA-early', 'AF-early', 'FB-early', 'BF-early', 'FA-early', 'AB-early', 'BC-early', 'CD-early', 'DF-early', 'FC-early', 'CA-early', 'AC-early', 'CB-early', 'BD-early', 'DB-early', 'BE-early', 'EB-early', 'BA-early', 'AD-early', 'DC-early', 'BF-late', 'FE-late', 'EA-late', 'AC-late', 'CD-late', 'DF-late', 'FA-late', 'AD-late', 'DC-late', 'CA-late-obs', 'AF-late-obs', 'FC-late', 'CB-late', 'BD-late', 'DB-late', 'BC-late', 'CE-late', 'EB-late', 'BE-late', 'EF-late', 'FD-late', 'DA-late', 'AF-late', 'FB-late', 'BA-late', 'AE-late', 'EC-late', 'CA-late', 'AB-late', 'BD-late', 'DE-late', 'ED-late', 'DC-late', 'CF-late', 'AC-even', 'CF-even', 'FC-even', 'CD-even', 'DE-even', 'EF-even', 'FB-even', 'BE-even', 'ED-even', 'DF-even', 'FA-even', 'AC-even-obs', 'CA-even-obs', 'AD-even', 'DB-even', 'BA-even', 'AE-even', 'EC-even', 'CB-even', 'BF-even', 'FA-even-obs', 'AC-even-obs', 'CF-even', 'FE-even', 'EA-even', 'AB-even', 'BC-even', 'CA-even', 'AF-even', 'FD-even', 'DA-even', 'AE-even', 'EB-even', 'BD-even', 'DC-even', 'CE-even']  #['DB-null', 'BE-null', 'ED-null', 'DA-null', 'AF-null', 'FE-null', 'EB-null', 'BF-null', 'FB-null', 'BA-null', 'AB-null', 'BD-null', 'DE-null', 'EC-null', 'CD-null', 'DF-null', 'FC-null', 'CF-null', 'FA-null', 'AC-null', 'CA-null', 'AE-null', 'EF-null', 'FD-null', 'DC-null', 'CB-null', 'BC-null', 'CE-null', 'EA-null', 'AD-null', 'FE-toptwo', 'EF-toptwo', 'FA-toptwo', 'AC-toptwo', 'CA-toptwo', 'AB-toptwo', 'BD-toptwo', 'DC-toptwo', 'CF-toptwo', 'FB-toptwo', 'BC-toptwo', 'CE-toptwo', 'EC-toptwo', 'CB-toptwo', 'BE-toptwo', 'EA-toptwo', 'AF-toptwo', 'FD-toptwo', 'DA-toptwo', 'AE-toptwo', 'EB-toptwo', 'BA-toptwo', 'AD-toptwo', 'DE-toptwo', 'ED-toptwo', 'DF-toptwo', 'FC-toptwo', 'CD-toptwo', 'DB-toptwo', 'BF-toptwo'] #['xy', 'yz', 'zx'] #['de', 'ef', 'fd'] #['ab', 'ba', 'ac', 'cb']

velocity_default = 1.0

# INDICES OF AUXILLARY DATA
AUX_WAYPOINT_INDEX  = 0
AUX_VELOCITY        = 1

waypoint_pub        = None

FLAG_PP_MODE        = True

# change Pose to the correct frame
def changePose(waypoint, target_frame):
    if waypoint.header.frame_id == target_frame:
        # already in correct frame
        return waypoint
    if not hasattr(changePose, 'listener'):
        changePose.listener = tf.TransformListener()
    tmp = PoseStamped()
    tmp.header.frame_id = waypoint.header.frame_id
    tmp.pose = waypoint.pose.pose
    try:
        changePose.listener.waitForTransform(
            target_frame, tmp.header.frame_id, rospy.Time(0), rospy.Duration(3))
        pose = changePose.listener.transformPose(target_frame, tmp)
        ret = PoseWithCovarianceStamped()
        ret.header.frame_id = target_frame
        ret.pose.pose = pose.pose
        return ret
    except:
        rospy.loginfo("CAN'T TRANSFORM POSE TO {} FRAME".format(target_frame))
        exit()

def get_exp_sequence(exp_option):
    # exp_option = "data: "1""
    
    exp_option = exp_option.data
    # exp_option.replace("data: ", "")
    # exp_option.strip('"')

    print("EXP_OPTION:~" + str(exp_option) + "~")
    # return path_names_list

    # if exp_option == "1":
    #     return ['AC_OBS-late', 'CD_OBS-late', 'DF-early', 'FE-even', 'ED-null', 'DA-even', 'AF_OBS-early', 'FD_OBS-late', 'DA-even', 'AD-even', 'DC-null', 'CE-even', 'EA-null', 'AC-late', 'CB-even', 'BD-null', 'DF-even', 'FD_OBS-early', 'DA-late', 'AE-late', 'EC-null', 'CF-late', 'FE-late', 'EF-null', 'FA_OBS-early', 'AF_OBS-even', 'FA-null', 'AB-late', 'BA-null', 'AC-early', 'CA_OBS-early', 'AC-even', 'CF-early', 'FE-early', 'EC-null', 'CE-early', 'EA-null', 'AB-early', 'BA-null', 'AF_OBS-late', 'FC-early', 'CA_OBS-even', 'AC-even', 'CD_OBS-even', 'DF-late', 'FD_OBS-even']
    # elif exp_option == "2":
    #     return ['AC_OBS-early', 'CD_OBS-early', 'DF-even', 'FE-early', 'ED-null', 'DA-early', 'AF_OBS-even', 'FD_OBS-early', 'DA-early', 'AD-early', 'DC-null', 'CE-late', 'EA-null', 'AC-even', 'CB-early', 'BD-null', 'DF-late', 'FD_OBS-even', 'DA-even', 'AE-early', 'EC-null', 'CF-even', 'FE-even', 'EF-null', 'FA_OBS-even', 'AF_OBS-late', 'FA-null', 'AB-even', 'BA-null', 'AC-late', 'CA_OBS-even', 'AC-early', 'CF-late', 'FE-late', 'EC-null', 'CE-even', 'EA-null', 'AB-late', 'BA-null', 'AF_OBS-early', 'FC-late', 'CA_OBS-late', 'AC-early', 'CD_OBS-late', 'DF-early', 'FD_OBS-late']
    # elif exp_option == "3":
    #     return ['AC_OBS-even', 'CD_OBS-even', 'DF-late', 'FE-late', 'ED-null', 'DA-late', 'AF_OBS-late', 'FD_OBS-even', 'DA-late', 'AD-late', 'DC-null', 'CE-early', 'EA-null', 'AC-early', 'CB-late', 'BD-null', 'DF-early', 'FD_OBS-late', 'DA-early', 'AE-even', 'EC-null', 'CF-early', 'FE-early', 'EF-null', 'FA_OBS-late', 'AF_OBS-early', 'FA-null', 'AB-early', 'BA-null', 'AC-even', 'CA_OBS-late', 'AC-late', 'CF-even', 'FE-even', 'EC-null', 'CE-late', 'EA-null', 'AB-even', 'BA-null', 'AF_OBS-even', 'FC-even', 'CA_OBS-early', 'AC-late', 'CD_OBS-early', 'DF-even', 'FD_OBS-early']
    # elif exp_option == "4":
    #     return ['AC-even', 'CD_OBS-even', 'DA-late', 'AB-late', 'BC-null', 'CE-early', 'EA-null', 'AD-early', 'DC_OBS-early', 'CA_OBS-late', 'AD-early', 'DE-early', 'ED-null', 'DF_OBS-late', 'FD-late', 'DF-early', 'FA_OBS-late', 'AF_OBS-even', 'FD_OBS-early', 'DA-early', 'AB-early', 'BA-null', 'AC_OBS-early', 'CA-late', 'AE-even', 'ED-null', 'DE-even', 'EA-null', 'AD-even', 'DE-late', 'EC-null', 'CA_OBS-even', 'AF_OBS-late', 'FD-even', 'DA-even', 'AD-late', 'DF_OBS-even', 'FA_OBS-early', 'AE-late', 'EC-null', 'CA-early', 'AB-even']
    # elif exp_option == "5":
    #     return ['AC-late', 'CD_OBS-late', 'DA-early', 'AB-even', 'BC-null', 'CE-late', 'EA-null', 'AD-late', 'DC_OBS-late', 'CA_OBS-even', 'AD-late', 'DE-even', 'ED-null', 'DF_OBS-even', 'FD-even', 'DF-late', 'FA_OBS-early', 'AF_OBS-early', 'FD_OBS-late', 'DA-even', 'AB-late', 'BA-null', 'AC_OBS-late', 'CA-early', 'AE-early', 'ED-null', 'DE-late', 'EA-null', 'AD-early', 'DE-early', 'EC-null', 'CA_OBS-early', 'AF_OBS-even', 'FD-early', 'DA-late', 'AD-even', 'DF_OBS-early', 'FA_OBS-even', 'AE-even', 'EC-null', 'CA-even', 'AB-early']
    # elif exp_option == "6":
    #     return ['AC-early', 'CD_OBS-early', 'DA-even', 'AB-early', 'BC-null', 'CE-even', 'EA-null', 'AD-even', 'DC_OBS-even', 'CA_OBS-early', 'AD-even', 'DE-late', 'ED-null', 'DF_OBS-early', 'FD-early', 'DF-even', 'FA_OBS-even', 'AF_OBS-late', 'FD_OBS-even', 'DA-late', 'AB-even', 'BA-null', 'AC_OBS-even', 'CA-even', 'AE-late', 'ED-null', 'DE-early', 'EA-null', 'AD-late', 'DE-even', 'EC-null', 'CA_OBS-late', 'AF_OBS-early', 'FD-late', 'DA-early', 'AD-early', 'DF_OBS-late', 'FA_OBS-late', 'AE-early', 'EC-null', 'CA-late', 'AB-late']
    # elif exp_option == "7":
    #     return ['AC-late', 'CA_OBS-early', 'AE-early', 'EF-null', 'FD-even', 'DE-early', 'EA-null', 'AF_OBS-early', 'FA_OBS-late', 'AB-late', 'BF-null', 'FC-even', 'CF-late', 'FD_OBS-late', 'DF-late', 'FD_OBS-early', 'DC_OBS-even', 'CF-early', 'FE-even', 'EA-null', 'AE-even', 'EF-null', 'FC-late', 'CB-early', 'BC-null', 'CA-early', 'AC_OBS-even', 'CD_OBS-early', 'DA-early', 'AC_OBS-late', 'CE-late', 'EA-null', 'AB-even', 'BA-null', 'AC-even', 'CF-even', 'FA_OBS-even', 'AF_OBS-late', 'FD-early', 'DE-late', 'EF-null', 'FD_OBS-even']
    # elif exp_option == "8":
    #     return ['AC-early', 'CA_OBS-even', 'AE-late', 'EF-null', 'FD-early', 'DE-late', 'EA-null', 'AF_OBS-even', 'FA_OBS-even', 'AB-early', 'BF-null', 'FC-early', 'CF-early', 'FD_OBS-even', 'DF-even', 'FD_OBS-late', 'DC_OBS-late', 'CF-even', 'FE-early', 'EA-null', 'AE-early', 'EF-null', 'FC-even', 'CB-even', 'BC-null', 'CA-even', 'AC_OBS-late', 'CD_OBS-late', 'DA-late', 'AC_OBS-early', 'CE-even', 'EA-null', 'AB-late', 'BA-null', 'AC-late', 'CF-late', 'FA_OBS-early', 'AF_OBS-early', 'FD-late', 'DE-even', 'EF-null', 'FD_OBS-early']
    # elif exp_option == "9":
    #     return ['AC-even', 'CA_OBS-late', 'AE-even', 'EF-null', 'FD-late', 'DE-even', 'EA-null', 'AF_OBS-late', 'FA_OBS-early', 'AB-even', 'BF-null', 'FC-late', 'CF-even', 'FD_OBS-early', 'DF-early', 'FD_OBS-even', 'DC_OBS-early', 'CF-late', 'FE-late', 'EA-null', 'AE-late', 'EF-null', 'FC-early', 'CB-late', 'BC-null', 'CA-late', 'AC_OBS-early', 'CD_OBS-even', 'DA-even', 'AC_OBS-even', 'CE-early', 'EA-null', 'AB-early', 'BA-null', 'AC-early', 'CF-early', 'FA_OBS-late', 'AF_OBS-even', 'FD-even', 'DE-early', 'EF-null', 'FD_OBS-late']
    # elif exp_option == "10":
    #     return ['CF-late', 'FE-late', 'EF-null', 'FD_OBS-even', 'DA-late', 'AC_OBS-early', 'CA-late', 'AF_OBS-late', 'FD-late', 'DA-late', 'AB-early', 'BC-null', 'CE-even', 'EC-null', 'CD_OBS-even', 'DF-early', 'FA_OBS-late', 'AD-early', 'DA-early', 'AC-early', 'CB-even', 'BA-null', 'AC_OBS-even', 'CE-late', 'ED-null', 'DF_OBS-late', 'FE-early', 'EA-null', 'AF_OBS-early', 'FD_OBS-early', 'DF-even', 'FC-even', 'CF-even', 'FA_OBS-early', 'AF_OBS-even', 'FE-even', 'EA-null', 'AC_OBS-late', 'CA-even', 'AE-early', 'EA-null', 'AB-late']
    # elif exp_option == "11":
    #     return ['CF-even', 'FE-even', 'EF-null', 'FD_OBS-early', 'DA-even', 'AC_OBS-even', 'CA-early', 'AF_OBS-even', 'FD-even', 'DA-even', 'AB-late', 'BC-null', 'CE-early', 'EC-null', 'CD_OBS-early', 'DF-late', 'FA_OBS-even', 'AD-late', 'DA-late', 'AC-even', 'CB-early', 'BA-null', 'AC_OBS-late', 'CE-even', 'ED-null', 'DF_OBS-even', 'FE-late', 'EA-null', 'AF_OBS-late', 'FD_OBS-late', 'DF-early', 'FC-early', 'CF-early', 'FA_OBS-late', 'AF_OBS-early', 'FE-early', 'EA-null', 'AC_OBS-early', 'CA-late', 'AE-late', 'EA-null', 'AB-even']
    # elif exp_option == "12":
    #     return ['CF-early', 'FE-early', 'EF-null', 'FD_OBS-late', 'DA-early', 'AC_OBS-late', 'CA-even', 'AF_OBS-early', 'FD-early', 'DA-early', 'AB-even', 'BC-null', 'CE-late', 'EC-null', 'CD_OBS-late', 'DF-even', 'FA_OBS-early', 'AD-even', 'DA-even', 'AC-late', 'CB-late', 'BA-null', 'AC_OBS-early', 'CE-early', 'ED-null', 'DF_OBS-early', 'FE-even', 'EA-null', 'AF_OBS-even', 'FD_OBS-even', 'DF-late', 'FC-late', 'CF-late', 'FA_OBS-even', 'AF_OBS-late', 'FE-late', 'EA-null', 'AC_OBS-even', 'CA-early', 'AE-even', 'EA-null', 'AB-early']

    # elif exp_option == "13":
    #     return ['BD-late', 'DB-even', 'BE-early', 'EC-even', 'CE-early', 'ED-even', 'DC_OBS-even', 'CA_OBS-early', 'AB-early', 'BA-even', 'AC-even', 'CD_OBS-late', 'DF_OBS-even', 'FD-late', 'DE-even', 'EC-even', 'CD-early', 'DC-early', 'CF-late', 'FC-even', 'CE-even', 'EA-late', 'AF-even', 'FA-even', 'AF_OBS-late', 'FD-early', 'DA-late', 'AC_OBS-even', 'CD_OBS-early', 'DE-late', 'EF-late', 'FB-late', 'BC-late', 'CA-late', 'AD-early', 'DF_OBS-late', 'FB-late', 'BF-early', 'FE-late', 'EB-even', 'BC-late', 'CB-even', 'BE-late', 'ED-early', 'DE-early', 'EA-early', 'AB-late', 'BA-early', 'AE-late', 'EA-early', 'AD-even', 'DA-early', 'AC_OBS-late', 'CD_OBS-even', 'DC_OBS-early', 'CA-early', 'AF-late', 'FD-even', 'DF_OBS-early', 'FA-late', 'AD-even', 'DB-early', 'BD-even']
    # elif exp_option == "14":
    #     return ['BD-even', 'DB-late', 'BE-late', 'EC-late', 'CE-even', 'ED-early', 'DC_OBS-late', 'CA_OBS-late', 'AB-even', 'BA-early', 'AC-early', 'CD_OBS-even', 'DF_OBS-early', 'FD-early', 'DE-late', 'EC-late', 'CD-late', 'DC-even', 'CF-early', 'FC-early', 'CE-late', 'EA-early', 'AF-late', 'FA-early', 'AF_OBS-early', 'FD-even', 'DA-even', 'AC_OBS-early', 'CD_OBS-late', 'DE-early', 'EF-even', 'FB-early', 'BC-even', 'CA-even', 'AD-even', 'DF_OBS-even', 'FB-early', 'BF-late', 'FE-early', 'EB-early', 'BC-even', 'CB-late', 'BE-even', 'ED-late', 'DE-even', 'EA-even', 'AB-early', 'BA-late', 'AE-early', 'EA-even', 'AD-late', 'DA-late', 'AC_OBS-even', 'CD_OBS-early', 'DC_OBS-even', 'CA-late', 'AF-early', 'FD-late', 'DF_OBS-late', 'FA-even', 'AD-late', 'DB-even', 'BD-early']
    # elif exp_option == "15":
    #     return ['BD-early', 'DB-early', 'BE-even', 'EC-early', 'CE-late', 'ED-late', 'DC_OBS-early', 'CA_OBS-even', 'AB-late', 'BA-late', 'AC-late', 'CD_OBS-early', 'DF_OBS-late', 'FD-even', 'DE-early', 'EC-early', 'CD-even', 'DC-late', 'CF-even', 'FC-late', 'CE-early', 'EA-even', 'AF-early', 'FA-late', 'AF_OBS-even', 'FD-late', 'DA-early', 'AC_OBS-late', 'CD_OBS-even', 'DE-even', 'EF-early', 'FB-even', 'BC-early', 'CA-early', 'AD-late', 'DF_OBS-early', 'FB-even', 'BF-even', 'FE-even', 'EB-late', 'BC-early', 'CB-early', 'BE-early', 'ED-even', 'DE-late', 'EA-late', 'AB-even', 'BA-even', 'AE-even', 'EA-late', 'AD-early', 'DA-even', 'AC_OBS-early', 'CD_OBS-late', 'DC_OBS-late', 'CA-even', 'AF-even', 'FD-early', 'DF_OBS-even', 'FA-early', 'AD-early', 'DB-late', 'BD-late']
    # elif exp_option == "16":
    #     return ['BC-early', 'CE-early', 'ED-late', 'DC-even', 'CB-even', 'BD-early', 'DE-early', 'EC-even', 'CA_OBS-even', 'AF_OBS-late', 'FA-late', 'AD-even', 'DF-even', 'FB-late', 'BE-even', 'EF-late', 'FD_OBS-early', 'DA-early', 'AC-early', 'CD_OBS-even', 'DF_OBS-even', 'FD-late', 'DC-late', 'CA_OBS-late', 'AD-late', 'DE-even', 'EC-late', 'CD-early', 'DA-even', 'AC-even', 'CE-even', 'EF-early', 'FA_OBS-late', 'AB-late', 'BE-late', 'ED-early', 'DB-early', 'BF-even', 'FD-late', 'DC_OBS-early', 'CB-late', 'BC-even', 'CD_OBS-early', 'DA-late', 'AB-early', 'BE-early', 'EF-even', 'FD-early', 'DE-late', 'EA-early', 'AD-early', 'DB-even', 'BF-late', 'FA-even', 'AC_OBS-early', 'CE-late', 'EB-early', 'BC-late', 'CA-late', 'AF-early', 'FD_OBS-late', 'DC_OBS-even']
    # elif exp_option == "17":
    #     return ['BC-late', 'CE-late', 'ED-even', 'DC-late', 'CB-early', 'BD-late', 'DE-even', 'EC-late', 'CA_OBS-early', 'AF_OBS-early', 'FA-early', 'AD-late', 'DF-early', 'FB-even', 'BE-early', 'EF-even', 'FD_OBS-late', 'DA-even', 'AC-late', 'CD_OBS-early', 'DF_OBS-early', 'FD-even', 'DC-early', 'CA_OBS-even', 'AD-early', 'DE-late', 'EC-early', 'CD-even', 'DA-late', 'AC-early', 'CE-early', 'EF-late', 'FA_OBS-even', 'AB-even', 'BE-even', 'ED-late', 'DB-late', 'BF-early', 'FD-even', 'DC_OBS-even', 'CB-even', 'BC-early', 'CD_OBS-late', 'DA-early', 'AB-late', 'BE-late', 'EF-early', 'FD-late', 'DE-early', 'EA-even', 'AD-even', 'DB-early', 'BF-even', 'FA-late', 'AC_OBS-late', 'CE-even', 'EB-late', 'BC-even', 'CA-even', 'AF-even', 'FD_OBS-even', 'DC_OBS-late']
    # elif exp_option == "18":
    #     return ['BC-even', 'CE-even', 'ED-early', 'DC-early', 'CB-late', 'BD-even', 'DE-late', 'EC-early', 'CA_OBS-late', 'AF_OBS-even', 'FA-even', 'AD-early', 'DF-late', 'FB-early', 'BE-late', 'EF-early', 'FD_OBS-even', 'DA-late', 'AC-even', 'CD_OBS-late', 'DF_OBS-late', 'FD-early', 'DC-even', 'CA_OBS-early', 'AD-even', 'DE-early', 'EC-even', 'CD-late', 'DA-early', 'AC-late', 'CE-late', 'EF-even', 'FA_OBS-early', 'AB-early', 'BE-early', 'ED-even', 'DB-even', 'BF-late', 'FD-early', 'DC_OBS-late', 'CB-early', 'BC-late', 'CD_OBS-even', 'DA-even', 'AB-even', 'BE-even', 'EF-late', 'FD-even', 'DE-even', 'EA-late', 'AD-late', 'DB-late', 'BF-early', 'FA-early', 'AC_OBS-even', 'CE-early', 'EB-even', 'BC-early', 'CA-early', 'AF-late', 'FD_OBS-early', 'DC_OBS-early']

    # elif exp_option == "19":
    #     return ['CE-even', 'EC-late', 'CB-early', 'BC-early', 'CD-even', 'DF-late', 'FB-early', 'BF-even', 'FA_OBS-late', 'AF_OBS-late', 'FC-late', 'CA_OBS-even', 'AF-even', 'FD_OBS-even', 'DE-early', 'ED-early', 'DB-early', 'BE-even', 'EA-late', 'AD-early', 'DC-even', 'CA-late', 'AF-late', 'FD-early', 'DC_OBS-early', 'CD_OBS-early', 'DA-early', 'AC_OBS-late', 'CA-early', 'AB-even', 'BE-late', 'ED-even', 'DB-even', 'BC-even', 'CF-even', 'FD_OBS-late', 'DE-even', 'EA-early', 'AE-late', 'EB-late', 'BF-late', 'FA-late', 'AF-early', 'FC-even', 'CE-early', 'EC-even', 'CD_OBS-even', 'DB-late', 'BA-late', 'AD-late', 'DE-late', 'ED-late', 'DF_OBS-early', 'FA-early', 'AC-even', 'CA_OBS-early', 'AF_OBS-even', 'FD-even', 'DE-late', 'EB-early', 'BD-early', 'DC-early', 'CB-late']
    # elif exp_option == "20":
    #     return ['CE-late', 'EC-even', 'CB-even', 'BC-late', 'CD-late', 'DF-even', 'FB-late', 'BF-early', 'FA_OBS-even', 'AF_OBS-even', 'FC-even', 'CA_OBS-late', 'AF-early', 'FD_OBS-early', 'DE-late', 'ED-even', 'DB-late', 'BE-early', 'EA-even', 'AD-even', 'DC-early', 'CA-early', 'AF-even', 'FD-late', 'DC_OBS-late', 'CD_OBS-late', 'DA-late', 'AC_OBS-early', 'CA-even', 'AB-late', 'BE-even', 'ED-late', 'DB-early', 'BC-early', 'CF-late', 'FD_OBS-even', 'DE-early', 'EA-late', 'AE-early', 'EB-even', 'BF-even', 'FA-early', 'AF-late', 'FC-early', 'CE-even', 'EC-early', 'CD_OBS-early', 'DB-even', 'BA-even', 'AD-early', 'DE-even', 'ED-early', 'DF_OBS-late', 'FA-even', 'AC-late', 'CA_OBS-even', 'AF_OBS-early', 'FD-early', 'DE-even', 'EB-late', 'BD-late', 'DC-late', 'CB-early']
    # elif exp_option == "21":
    #     return ['CE-early', 'EC-early', 'CB-late', 'BC-even', 'CD-early', 'DF-early', 'FB-even', 'BF-late', 'FA_OBS-early', 'AF_OBS-early', 'FC-early', 'CA_OBS-early', 'AF-late', 'FD_OBS-late', 'DE-even', 'ED-late', 'DB-even', 'BE-late', 'EA-early', 'AD-late', 'DC-late', 'CA-even', 'AF-early', 'FD-even', 'DC_OBS-even', 'CD_OBS-even', 'DA-even', 'AC_OBS-even', 'CA-late', 'AB-early', 'BE-early', 'ED-early', 'DB-late', 'BC-late', 'CF-early', 'FD_OBS-early', 'DE-late', 'EA-even', 'AE-even', 'EB-early', 'BF-early', 'FA-even', 'AF-even', 'FC-late', 'CE-late', 'EC-late', 'CD_OBS-late', 'DB-early', 'BA-early', 'AD-even', 'DE-early', 'ED-even', 'DF_OBS-even', 'FA-late', 'AC-early', 'CA_OBS-late', 'AF_OBS-late', 'FD-late', 'DE-early', 'EB-even', 'BD-even', 'DC-even', 'CB-even']
    # elif exp_option == "22":
    #     return ['DF-late', 'FA_OBS-even', 'AD-even', 'DB-even', 'BC-even', 'CA-early', 'AC_OBS-even', 'CE-early', 'EF-early', 'FA-late', 'AF_OBS-even', 'FC-even', 'CB-even', 'BE-early', 'EC-early', 'CD-late', 'DC-early', 'CB-even', 'BD-even', 'DE-even', 'EF-early', 'FD_OBS-even', 'DB-late', 'BA-late', 'AE-even', 'EF-even', 'FD-early', 'DE-late', 'EA-even', 'AF-even', 'FD_OBS-late', 'DA-late', 'AC-even', 'CF-late', 'FA_OBS-late', 'AC_OBS-late', 'CD-early', 'DC_OBS-late', 'CB-late', 'BE-even', 'EB-even', 'BD-late', 'DC_OBS-early', 'CD_OBS-early', 'DA-early', 'AC_OBS-early', 'CB-early', 'BC-early', 'CF-early', 'FD_OBS-early', 'DC-late', 'CE-late', 'ED-late', 'DB-early', 'BF-early', 'FE-early', 'EB-late', 'BF-early', 'FA-even', 'AC-late', 'CE-late', 'EC-late', 'CD-even', 'DF-even']
    # elif exp_option == "23":
    #     return ['DF-early', 'FA_OBS-early', 'AD-early', 'DB-late', 'BC-early', 'CA-late', 'AC_OBS-early', 'CE-even', 'EF-even', 'FA-early', 'AF_OBS-early', 'FC-early', 'CB-late', 'BE-late', 'EC-even', 'CD-early', 'DC-even', 'CB-late', 'BD-late', 'DE-early', 'EF-even', 'FD_OBS-early', 'DB-early', 'BA-even', 'AE-late', 'EF-late', 'FD-even', 'DE-even', 'EA-late', 'AF-late', 'FD_OBS-even', 'DA-even', 'AC-early', 'CF-even', 'FA_OBS-even', 'AC_OBS-even', 'CD-even', 'DC_OBS-even', 'CB-early', 'BE-early', 'EB-early', 'BD-early', 'DC_OBS-late', 'CD_OBS-late', 'DA-late', 'AC_OBS-late', 'CB-even', 'BC-late', 'CF-late', 'FD_OBS-late', 'DC-early', 'CE-early', 'ED-early', 'DB-even', 'BF-even', 'FE-late', 'EB-even', 'BF-even', 'FA-late', 'AC-even', 'CE-early', 'EC-early', 'CD-late', 'DF-late']
    # elif exp_option == "24":
    #     return ['DF-even', 'FA_OBS-late', 'AD-late', 'DB-early', 'BC-late', 'CA-even', 'AC_OBS-late', 'CE-late', 'EF-late', 'FA-even', 'AF_OBS-late', 'FC-late', 'CB-early', 'BE-even', 'EC-late', 'CD-even', 'DC-late', 'CB-early', 'BD-early', 'DE-late', 'EF-late', 'FD_OBS-late', 'DB-even', 'BA-early', 'AE-early', 'EF-early', 'FD-late', 'DE-early', 'EA-early', 'AF-early', 'FD_OBS-early', 'DA-early', 'AC-late', 'CF-early', 'FA_OBS-early', 'AC_OBS-early', 'CD-late', 'DC_OBS-early', 'CB-even', 'BE-late', 'EB-late', 'BD-even', 'DC_OBS-even', 'CD_OBS-even', 'DA-even', 'AC_OBS-even', 'CB-late', 'BC-even', 'CF-even', 'FD_OBS-even', 'DC-even', 'CE-even', 'ED-even', 'DB-late', 'BF-late', 'FE-even', 'EB-early', 'BF-late', 'FA-early', 'AC-early', 'CE-even', 'EC-even', 'CD-early', 'DF-early']

    # if exp_option == 'a1':
    #     return ['AE-early', 'EC-early', 'CD_OBS-late', 'DC-early', 'CA-late', 'AC_OBS-early', 'CD-early', 'DA-early', 'AB-early', 'BE-late', 'EF-late', 'FD_OBS-early', 'DB-early', 'BF-late', 'FD-early', 'DC_OBS-early', 'CF-late', 'FE-late', 'EB-late', 'BA-late', 'AF_OBS-late', 'FA_OBS-early', 'AC_OBS-late', 'CF-early', 'FA-late', 'AC-early', 'CE-late', 'EA-late', 'AF-late', 'FE-early', 'EF-early', 'FD_OBS-late', 'DA-late', 'AB-late', 'BC-early', 'CD-late', 'DB-late', 'BD-early', 'DF-late', 'FE-early', 'EB-early']
    # elif exp_option == 'a2':
    #     return ['AE-late', 'EC-late', 'CD_OBS-early', 'DC-late', 'CA-early', 'AC_OBS-late', 'CD-late', 'DA-late', 'AB-late', 'BE-early', 'EF-early', 'FD_OBS-late', 'DB-late', 'BF-early', 'FD-late', 'DC_OBS-late', 'CF-early', 'FE-early', 'EB-early', 'BA-early', 'AF_OBS-early', 'FA_OBS-late', 'AC_OBS-early', 'CF-late', 'FA-early', 'AC-late', 'CE-early', 'EA-early', 'AF-early', 'FE-late', 'EF-late', 'FD_OBS-early', 'DA-early', 'AB-early', 'BC-late', 'CD-early', 'DB-early', 'BD-late', 'DF-early', 'FE-late', 'EB-late']
    
    # elif exp_option == 'b1':
    #     return ['AE-early', 'EB-late', 'BC-late', 'CD_OBS-early', 'DF_OBS-late', 'FC-early', 'CF-late', 'FD-late', 'DB-late', 'BF-late', 'FE-early', 'ED-early', 'DC_OBS-late', 'CA-early', 'AB-late', 'BA-late', 'AC_OBS-late', 'CD-early', 'DC-early', 'CE-early', 'EC-early', 'CF-early', 'FE-late', 'EA-late', 'AE-late', 'EB-early', 'BA-early', 'AF-late', 'FA-late', 'AC-late', 'CA_OBS-early', 'AB-early', 'BF-early', 'FD_OBS-early', 'DA-late', 'AF_OBS-early', 'FA_OBS-late', 'AE-late', 'ED-late', 'DB-early', 'BF-early', 'FD-early']

    # elif exp_option == 'b2':
    #     return ['AE-late', 'EB-early', 'BC-early', 'CD_OBS-late', 'DF_OBS-early', 'FC-late', 'CF-early', 'FD-early', 'DB-early', 'BF-early', 'FE-late', 'ED-late', 'DC_OBS-early', 'CA-late', 'AB-early', 'BA-early', 'AC_OBS-early', 'CD-late', 'DC-late', 'CE-late', 'EC-late', 'CF-late', 'FE-early', 'EA-early', 'AE-early', 'EB-late', 'BA-late', 'AF-early', 'FA-early', 'AC-early', 'CA_OBS-late', 'AB-late', 'BF-late', 'FD_OBS-late', 'DA-early', 'AF_OBS-late', 'FA_OBS-early', 'AE-early', 'ED-early', 'DB-late', 'BF-late', 'FD-late']
    
    # elif exp_option == 'c1':
    #     return ['DC-late', 'CE-late', 'ED-late', 'DB-late', 'BA-late', 'AC-late', 'CB-early', 'BD-early', 'DC_OBS-late', 'CD-early', 'DF-late', 'FD_OBS-early', 'DA-late', 'AD-late', 'DE-late', 'EC-late', 'CD_OBS-late', 'DA-late', 'AC_OBS-early', 'CB-early', 'BE-early', 'ED-early', 'DF-early', 'FB-early', 'BE-late', 'EA-early', 'AF_OBS-early', 'FC-early', 'CE-early', 'EC-early', 'CD_OBS-early', 'DF_OBS-late', 'FE-early', 'EB-late', 'BA-early', 'AF-early', 'FA-late', 'AC_OBS-late', 'CA-early', 'AD-early', 'DA-early', 'AB-late', 'BF-late']

    # elif exp_option == 'c2':
    #     return ['DC-early', 'CE-early', 'ED-early', 'DB-early', 'BA-early', 'AC-early', 'CB-late', 'BD-late', 'DC_OBS-early', 'CD-late', 'DF-early', 'FD_OBS-late', 'DA-early', 'AD-early', 'DE-early', 'EC-early', 'CD_OBS-early', 'DA-early', 'AC_OBS-late', 'CB-late', 'BE-late', 'ED-late', 'DF-late', 'FB-late', 'BE-early', 'EA-late', 'AF_OBS-late', 'FC-late', 'CE-late', 'EC-late', 'CD_OBS-late', 'DF_OBS-early', 'FE-late', 'EB-early', 'BA-late', 'AF-late', 'FA-early', 'AC_OBS-early', 'CA-late', 'AD-late', 'DA-late', 'AB-early', 'BF-early']

    # elif exp_option == 'd1':
    #     return ['DA-early', 'AC_OBS-late', 'CD_OBS-late', 'DE-early', 'ED-late', 'DF_OBS-late', 'FA-late', 'AD-late', 'DC-late', 'CA-early', 'AB-early', 'BA-late', 'AE-late', 'EA-early', 'AF_OBS-late', 'FD-early', 'DB-late', 'BE-early', 'EB-early', 'BD-early', 'DC_OBS-early', 'CF-early', 'FA_OBS-early', 'AC_OBS-early', 'CB-late', 'BA-early', 'AE-early', 'EA-late', 'AC-late', 'CD-early', 'DB-early', 'BD-late', 'DC-early', 'CD-early', 'DA-late', 'AD-early', 'DF-late', 'FE-late', 'ED-early', 'DF_OBS-early', 'FE-late', 'EB-late']

    # elif exp_option == 'd2':
    #     return ['DA-late', 'AC_OBS-early', 'CD_OBS-early', 'DE-late', 'ED-early', 'DF_OBS-early', 'FA-early', 'AD-early', 'DC-early', 'CA-late', 'AB-late', 'BA-early', 'AE-early', 'EA-late', 'AF_OBS-early', 'FD-late', 'DB-early', 'BE-late', 'EB-late', 'BD-late', 'DC_OBS-late', 'CF-late', 'FA_OBS-late', 'AC_OBS-late', 'CB-early', 'BA-late', 'AE-late', 'EA-early', 'AC-early', 'CD-late', 'DB-late', 'BD-early', 'DC-late', 'CD-late', 'DA-early', 'AD-late', 'DF-early', 'FE-early', 'ED-late', 'DF_OBS-late', 'FE-early', 'EB-early']

    # elif exp_option == 'e1':
    #     return ['DA-late', 'AE-early', 'EA-late', 'AB-early', 'BD-late', 'DF-early', 'FA-late', 'AF-early', 'FE-early', 'EB-early', 'BA-late', 'AC_OBS-early', 'CF-late', 'FD_OBS-late', 'DB-early', 'BD-late', 'DC_OBS-early', 'CA-early', 'AF-early', 'FA_OBS-early', 'AE-early', 'EF-early', 'FA-early', 'AC_OBS-late', 'CE-late', 'EC-early', 'CF-early', 'FD-late', 'DE-late', 'ED-late', 'DA-early', 'AB-late', 'BE-late', 'EC-early', 'CA-late', 'AF_OBS-late', 'FA_OBS-late', 'AF-late', 'FB-late', 'BC-early', 'CB-late', 'BF-early', 'FD_OBS-early']

    # elif exp_option == 'e2':
    #     return ['DA-early', 'AE-late', 'EA-early', 'AB-late', 'BD-early', 'DF-late', 'FA-early', 'AF-late', 'FE-late', 'EB-late', 'BA-early', 'AC_OBS-late', 'CF-early', 'FD_OBS-early', 'DB-late', 'BD-early', 'DC_OBS-late', 'CA-late', 'AF-late', 'FA_OBS-late', 'AE-late', 'EF-late', 'FA-late', 'AC_OBS-early', 'CE-early', 'EC-late', 'CF-late', 'FD-early', 'DE-early', 'ED-early', 'DA-late', 'AB-early', 'BE-early', 'EC-late', 'CA-early', 'AF_OBS-early', 'FA_OBS-early', 'AF-early', 'FB-early', 'BC-late', 'CB-early', 'BF-late', 'FD_OBS-late']

    # elif exp_option == 'f1':
    #     return ['BE-early', 'ED-early', 'DA-late', 'AF-early', 'FD-early', 'DB-early', 'BD-early', 'DE-early', 'EC-late', 'CA-early', 'AC_OBS-early', 'CF-early', 'FA_OBS-early', 'AF_OBS-late', 'FA-early', 'AB-late', 'BC-early', 'CE-late', 'ED-early', 'DF_OBS-late', 'FE-late', 'EF-late', 'FA-late', 'AF_OBS-early', 'FC-early', 'CA_OBS-late', 'AF-late', 'FA_OBS-late', 'AE-early', 'EA-early', 'AD-late', 'DB-late', 'BC-late', 'CB-early', 'BF-late', 'FD-late', 'DF_OBS-early', 'FA-late', 'AC-late', 'CE-early', 'EB-late']

    # elif exp_option == 'f2':
    #     return ['BE-late', 'ED-late', 'DA-early', 'AF-late', 'FD-late', 'DB-late', 'BD-late', 'DE-late', 'EC-early', 'CA-late', 'AC_OBS-late', 'CF-late', 'FA_OBS-late', 'AF_OBS-early', 'FA-late', 'AB-early', 'BC-late', 'CE-early', 'ED-late', 'DF_OBS-early', 'FE-early', 'EF-early', 'FA-early', 'AF_OBS-late', 'FC-late', 'CA_OBS-early', 'AF-early', 'FA_OBS-early', 'AE-late', 'EA-late', 'AD-early', 'DB-early', 'BC-early', 'CB-late', 'BF-early', 'FD-early', 'DF_OBS-late', 'FA-early', 'AC-early', 'CE-late', 'EB-early']

    # elif exp_option == 'g1':
    #     return ['BD-early', 'DA-early', 'AB-late', 'BC-early', 'CF-early', 'FA-early', 'AF-early', 'FE-early', 'EA-late', 'AF_OBS-early', 'FD-early', 'DB-early', 'BE-early', 'EF-late', 'FA_OBS-early', 'AC_OBS-late', 'CA-late', 'AE-early', 'ED-late', 'DF_OBS-late', 'FD_OBS-early', 'DC-late', 'CB-early', 'BE-late', 'EC-early', 'CD-late', 'DC_OBS-late', 'CD_OBS-late', 'DA-late', 'AE-late', 'EF-early', 'FD-late', 'DB-late', 'BC-late', 'CA-early', 'AC_OBS-early', 'CF-late', 'FE-late', 'EB-late', 'BD-late']

    # elif exp_option == 'g2':
    #     return ['BD-late', 'DA-late', 'AB-early', 'BC-late', 'CF-late', 'FA-late', 'AF-late', 'FE-late', 'EA-early', 'AF_OBS-late', 'FD-late', 'DB-late', 'BE-late', 'EF-early', 'FA_OBS-late', 'AC_OBS-early', 'CA-early', 'AE-late', 'ED-early', 'DF_OBS-early', 'FD_OBS-late', 'DC-early', 'CB-late', 'BE-early', 'EC-late', 'CD-early', 'DC_OBS-early', 'CD_OBS-early', 'DA-early', 'AE-early', 'EF-late', 'FD-early', 'DB-early', 'BC-early', 'CA-late', 'AC_OBS-late', 'CF-early', 'FE-early', 'EB-early', 'BD-early']

    # elif exp_option == "test":
    #     return ["AF-even", "FE-early", "EA-late", "AC-even", "CB-early", "BD-even", "DE-even", "EA-even"]


    # ### POST MAY 1
    # a1 = ['FB-late', 'BA-early', 'AD-late', 'DC-late', 'CD-early', 'DA-late', 'AB-early', 'BF-late', 'FA_OBS-late', 'AC_OBS-early', 'CE-early', 'EB-early', 'BD-late', 'DE-early', 'ED-late', 'DC_OBS-late', 'CA-early', 'AD-late', 'DF_OBS-late', 'FD-early', 'DE-early', 'EA-late']
    # a2 = ['DF-early', 'FA-early', 'AF_OBS-late', 'FC-late', 'CE-early', 'EB-early', 'BC-early', 'CF-late', 'FD_OBS-late', 'DE-early', 'EA-late', 'AF-late', 'FB-late', 'BF-late', 'FA_OBS-late', 'AC_OBS-early', 'CA-early', 'AB-early', 'BE-early', 'ED-late']

    # b1 = ['CA-early', 'AD-late', 'DA-late', 'AE-early', 'EB-early', 'BD-early', 'DF_OBS-late', 'FA_OBS-late', 'AF_OBS-late', 'FB-late', 'BC-late', 'CB-late', 'BF-early', 'FD-late', 'DE-early', 'EC-early', 'CA_OBS-late', 'AF-early', 'FA-late', 'AE-early', 'EF-late']
    # b2 = ['DF-late', 'FB-late', 'BA-late', 'AF-early', 'FA_OBS-late', 'AD-late', 'DC_OBS-late', 'CA-early', 'AE-early', 'ED-late', 'DA-late', 'AB-late', 'BE-early', 'EC-early', 'CA_OBS-late', 'AF-early', 'FD_OBS-late', 'DE-early', 'EC-early', 'CD-late', 'DB-late', 'BD-early']

    # c1 = ['AF-late', 'FA_OBS-early', 'AC-late', 'CD-late', 'DB-early', 'BE-late', 'EF-early', 'FE-late', 'EA-early', 'AE-late', 'EB-late', 'BA-early', 'AD-early', 'DA-early', 'AF_OBS-early', 'FD-late', 'DF_OBS-early', 'FB-early', 'BF-late', 'FA-late', 'AC_OBS-early', 'CB-early']
    # c2 = ['FC-early', 'CD-late', 'DF_OBS-early', 'FD-late', 'DB-early', 'BD-late', 'DC-late', 'CB-early', 'BE-late', 'EF-early', 'FA_OBS-early', 'AC-late', 'CE-late', 'EC-early', 'CF-early', 'FE-late', 'EC-early', 'CA_OBS-early', 'AF_OBS-early', 'FB-early', 'BC-early']

    # d1 = ['AE-early', 'ED-late', 'DA-early', 'AC-late', 'CD_OBS-late', 'DF_OBS-late', 'FD-late', 'DE-late', 'EC-late', 'CB-early', 'BE-early', 'ED-late', 'DC_OBS-early', 'CD-early', 'DC-early', 'CA_OBS-late', 'AD-early', 'DB-early', 'BC-early', 'CB-early', 'BD-early']
    # d2 = ['BF-early', 'FB-early', 'BC-early', 'CF-early', 'FE-late', 'EB-early', 'BA-early', 'AF-early', 'FA-early', 'AC_OBS-late', 'CD_OBS-late', 'DF-late', 'FC-early', 'CB-early', 'BE-early', 'ED-late', 'DC_OBS-early', 'CA-late', 'AE-early', 'EA-late', 'AD-early', 'DF_OBS-late']

    # e1 = ['DE-late', 'EB-late', 'BF-early', 'FB-late', 'BC-early', 'CE-late', 'EA-late', 'AD-late', 'DF_OBS-early', 'FC-late', 'CD-late', 'DC_OBS-early', 'CA_OBS-early', 'AC-early', 'CB-late', 'BC-early', 'CD_OBS-early', 'DF-early', 'FE-late', 'EF-early', 'FA-late', 'AF-late']
    # e2 = ['AB-late', 'BA-early', 'AC-early', 'CE-late', 'EA-late', 'AC_OBS-early', 'CD_OBS-early', 'DF_OBS-early', 'FD-early', 'DE-late', 'EF-early', 'FB-late', 'BF-early', 'FC-late', 'CF-late', 'FA-late', 'AF_OBS-early', 'FE-late', 'EB-late', 'BA-early', 'AF-late']

    # f1 = ['AE-late', 'EF-late', 'FA_OBS-early', 'AD-late', 'DC_OBS-late', 'CA-early', 'AC_OBS-late', 'CD-late', 'DC-late', 'CB-early', 'BE-late', 'EA-early', 'AB-early', 'BC-late', 'CF-late', 'FD-early', 'DA-early', 'AD-late', 'DE-late', 'EB-late', 'BF-late', 'FB-early', 'BD-late', 'DF_OBS-early']
    # f2 = ['FE-late', 'EC-early', 'CA-early', 'AE-late', 'ED-late', 'DF_OBS-early', 'FA_OBS-early', 'AF-late', 'FA-late', 'AF_OBS-late', 'FD-early', 'DB-early', 'BF-late', 'FC-early', 'CA_OBS-late', 'AD-late', 'DE-late', 'EB-late', 'BA-late', 'AB-early']

    p0a = ['CB-null', 'BA-early', 'AB-late', 'BE-late', 'ED-late', 'DC-early', 'CD-early', 'DA-early', 'AD-early', 'DF-early', 'FB-late', 'BD-early', 'DF_OBS-early', 'FE-late', 'EC-late', 'CA_OBS-early', 'AC-early', 'CE-late', 'ED-late', 'DE-late']

    p0b = ['CB-null', 'BA-late', 'AB-early', 'BE-early', 'ED-early', 'DC-late', 'CD-late', 'DA-late', 'AD-late', 'DF-late', 'FB-early', 'BD-late', 'DF_OBS-late', 'FE-early', 'EC-early', 'CA_OBS-late', 'AC-late', 'CE-early', 'ED-early', 'DE-early']

    p1a = ['AD-null', 'DB-late', 'BE-late', 'EF-late', 'FD-early', 'DE-late', 'EA-late', 'AD-early', 'DA-early', 'AC_OBS-early', 'CD-early', 'DF_OBS-early', 'FC-early', 'CA-early', 'AE-late', 'EC-late', 'CB-late', 'BC-early', 'CB-late', 'BD-early', 'DC-early', 'CD-early', 'DE-late', 'EA-late']

    p1b = ['AD-null', 'DB-early', 'BE-early', 'EF-early', 'FD-late', 'DE-early', 'EA-early', 'AD-late', 'DA-late', 'AC_OBS-late', 'CD-late', 'DF_OBS-late', 'FC-late', 'CA-late', 'AE-early', 'EC-early', 'CB-early', 'BC-late', 'CB-early', 'BD-late', 'DC-late', 'CD-late', 'DE-early', 'EA-early']

    p2a = ['FB-null', 'BC-early', 'CD-early', 'DF-early', 'FD_OBS-early', 'DA-early', 'AD-early', 'DE-late', 'ED-late', 'DC-early', 'CA_OBS-early', 'AB-late', 'BD-early', 'DB-late', 'BE-late', 'EC-late', 'CE-late', 'EC-late', 'CA-early'] # 

    p2b = ['FB-null', 'BC-late', 'CD-late', 'DF-late', 'FD_OBS-late', 'DA-late', 'AD-late', 'DE-early', 'ED-early', 'DC-late', 'CA_OBS-late', 'AB-early', 'BD-late', 'DB-early', 'BE-early', 'EC-early', 'CE-early', 'EC-early', 'CA-late']

    p3a = ['BA-null', 'AF-early', 'FD-early', 'DB-late', 'BA-early', 'AD-early', 'DA-early', 'AB-late', 'BF-early', 'FD_OBS-early', 'DE-late', 'EB-late', 'BE-late', 'EC-late', 'CE-late', 'ED-late', 'DA-early', 'AC-early', 'CD-early', 'DA-early', 'AC_OBS-early']

    p3b = ['BA-null', 'AF-late', 'FD-late', 'DB-early', 'BA-late', 'AD-late', 'DA-late', 'AB-early', 'BF-late', 'FD_OBS-late', 'DE-early', 'EB-early', 'BE-early', 'EC-early', 'CE-early', 'ED-early', 'DA-late', 'AC-late', 'CD-late', 'DA-late', 'AC_OBS-late']



    p4a = ['AD-null', 'DF_OBS-early', 'FC-late', 'CF-early', 'FC-early', 'CA_OBS-late', 'AF-early', 'FB-early', 'BA-late', 'AE-late', 'ED-late', 'DF-late', 'FB-early', 'BE-late'] #'AC-late', 'CB-early', 'BF-late', 'FE-late', 'EC-early', 'CA_OBS-late', 'AF-early', 'FB-early', 'BA-late', 'AE-late', 'ED-late', 'DF-late', 'FB-early', 'BE-late']

    p4b = ['AD-null', 'DF_OBS-late', 'FC-early', 'CF-late', 'FA-late', 'AC-early', 'CB-late', 'BF-early', 'FE-early', 'EC-late', 'CA_OBS-early', 'AF-late', 'FB-late', 'BA-early', 'AE-early', 'ED-early', 'DF-early', 'FB-late', 'BE-early']

    p5a = ['EF-null', 'FD-late', 'DF_OBS-early', 'FE-late', 'EF-late', 'FD-late', 'DC-early', 'CA_OBS-late', 'AB-early', 'BF-late', 'FA-early', 'AE-late', 'EB-late', 'BD-late', 'DB-early']

    p5b = ['EF-null', 'FC-early', 'CE-early', 'EC-late', 'CB-late', 'BC-early', 'CA-early', 'AD-late', 'DF_OBS-late', 'FE-early', 'EF-early', 'FD-early', 'DC-late', 'CA_OBS-early', 'AB-late', 'BF-early', 'FA-late', 'AE-early', 'EB-early', 'BD-early', 'DB-late']

    p6a = ['FC-null', 'CB-early', 'BA-late', 'AE-late', 'EC-early', 'CE-late', 'ED-late', 'DA-late', 'AD-early', 'DB-early', 'BF-late', 'FD-late', 'DC-early', 'CA_OBS-late', 'AD-early', 'DB-early', 'BA-late', 'AC-late', 'CE-late', 'ED-late', 'DE-late', 'EA-early', 'AE-late', 'EB-late', 'BD-late', 'DA-late', 'AF-early', 'FA-early', 'AD-early', 'DF_OBS-early']

    p6b = ['FC-null', 'CB-late', 'BA-early', 'AE-early', 'EC-late', 'CE-early', 'ED-early', 'DA-early', 'AD-late', 'DB-late', 'BF-early', 'FD-early', 'DC-late', 'CA_OBS-early', 'AD-late', 'DB-late', 'BA-early', 'AC-early', 'CE-early', 'ED-early', 'DE-early', 'EA-late', 'AE-early', 'EB-early', 'BD-early', 'DA-early', 'AF-late', 'FA-late', 'AD-late', 'DF_OBS-late']

    p7a = ['BD-null', 'DE-late', 'EB-late', 'BD-late', 'DF_OBS-early', 'FA-early', 'AC-late', 'CB-early', 'BC-late', 'CF-early', 'FD-late', 'DA-late', 'AC_OBS-late', 'CE-late', 'EA-early', 'AF-early', 'FB-early', 'BE-late', 'ED-late', 'DE-late', 'EB-late', 'BD-late']

    p7b = ['BD-null', 'DE-early', 'EB-early', 'BD-early', 'DF_OBS-late', 'FA-late', 'AC-early', 'CB-late', 'BC-early', 'CF-late', 'FD-early', 'DA-early', 'AC_OBS-early', 'CE-early', 'EA-late', 'AF-late', 'FB-late', 'BE-early', 'ED-early', 'DE-early', 'EB-early', 'BD-early']



    if exp_option == "p0a":
        return p0a
    elif exp_option == "p0b":
        return p0b
    elif exp_option == "p1a":
        return p1a
    elif exp_option == "p1b":
        return p1b
    elif exp_option == "p2a":
        return p2a
    elif exp_option == "p2b":
        return p2b
    elif exp_option == "p3a":
        return p3a
    elif exp_option == "p3b":
        return p3b

    elif exp_option == "p4a":
        return p4a
    elif exp_option == "p4b":
        return p4b
    elif exp_option == "p5a":
        return p5a
    elif exp_option == "p5b":
        return p5b
    elif exp_option == "p6a":
        return p6a
    elif exp_option == "p6b":
        return p6b
    elif exp_option == "p7a":
        return p7a
    elif exp_option == "p7b":
        return p7b




    elif exp_option == "test1":
        return ["AF-null", "FE-null", "EA-null", "AC-null", "CB-null", "BD-null", "DE-null", "EA-null"]

    elif exp_option == "test2":
        return ["BA-null", "AC-null", "CF-null", "FB-null", "BE-null", "ED-null", "DF-null"]

    elif exp_option == "test3":
        return ["CA-null", "AE-null", "EF-null", "FA-null", "AD-null", "DB-null", "BC-null"]


    return ["AB-null", "BC-null"]

    # #### AC EARLY OBS IS A PROBLEM
    # return ['CF-early', 'FE-early', 'EF-early', 'FD-early', 'DE-early', 'EA-early', 'AC-early-obs', 'CD-early-obs', 'DA-early', 'AE-early', 'EC-early', 'CE-early', 'ED-early', 'DA-early', 'AF-early', 'FB-early', 'BF-early', 'FA-early', 'AB-early', 'BC-early', 'CD-early', 'DF-early', 'FC-early', 'CA-early', 'AC-early', 'CB-early', 'BD-early', 'DB-early', 'BE-early', 'EB-early', 'BA-early', 'AD-early', 'DC-early', 'BF-late', 'FE-late', 'EA-late', 'AC-late', 'CD-late', 'DF-late', 'FA-late', 'AD-late', 'DC-late', 'CA-late-obs', 'AF-late-obs', 'FC-late', 'CB-late', 'BD-late', 'DB-late', 'BC-late', 'CE-late', 'EB-late', 'BE-late', 'EF-late', 'FD-late', 'DA-late', 'AF-late', 'FB-late', 'BA-late', 'AE-late', 'EC-late', 'CA-late', 'AB-late', 'BD-late', 'DE-late', 'ED-late', 'DC-late', 'CF-late', 'AC-even', 'CF-even', 'FC-even', 'CD-even', 'DE-even', 'EF-even', 'FB-even', 'BE-even', 'ED-even', 'DF-even', 'FA-even', 'AC-even-obs', 'CA-even-obs', 'AD-even', 'DB-even', 'BA-even', 'AE-even', 'EC-even', 'CB-even', 'BF-even', 'FA-even-obs', 'AC-even-obs', 'CF-even', 'FE-even', 'EA-even', 'AB-even', 'BC-even', 'CA-even', 'AF-even', 'FD-even', 'DA-even', 'AE-even', 'EB-even', 'BD-even', 'DC-even', 'CE-even']

    # # return ['AB-toptwo', 'BA-toptwo', 'AE-toptwo', 'EF-toptwo', 'FD-toptwo', 'DF-toptwo', 'FB-toptwo', 'BE-toptwo', 'ED-toptwo', 'DA-toptwo', 'AF-toptwo', 'FA-toptwo', 'AD-toptwo', 'DB-toptwo', 'BC-toptwo', 'CF-toptwo', 'FC-toptwo', 'CD-toptwo', 'DE-toptwo', 'EB-toptwo', 'BF-toptwo', 'FE-toptwo', 'EA-toptwo', 'AC-toptwo', 'CB-toptwo', 'BD-toptwo', 'DC-toptwo', 'CE-toptwo', 'EC-toptwo', 'CA-toptwo']

    # # return ['CF-null', 'FE-null', 'EF-null', 'FB-null', 'BF-null', 'FA-null', 'AC-null', 'CA-null', 'AD-null', 'DC-null', 'CE-null', 'EB-null', 'BE-null', 'EC-null', 'CB-null', 'BC-null', 'CD-null', 'DB-null', 'BA-null', 'AE-null', 'ED-null', 'DF-null', 'FD-null', 'DE-null', 'EA-null', 'AB-null', 'BD-null', 'DA-null', 'AF-null', 'FC-null']

def initialize_waypoints():
    print("Get waypoints ==> ie detailed trajectories")
    output_folder_default = os.path.join(rospkg.RosPack().get_path('fetch_study'), 'waypoints/')
    output_folder = rospy.get_param('~output_folder', output_folder_default)

    dir_list = os.listdir(output_folder_default)
    dir_list = [f for f in dir_list if 'csv' in f]

    for file_name in dir_list:
        waypoints_path = output_folder_default + file_name
        path_name = file_name.replace('.csv','')

        waypoints_info, start, goal      = import_waypoints(path_name, waypoints_path)
        
        key = path_name

        # KEY = (name, first, last)
        route_dict[key]     = waypoints_info
        start_dict[key]     = start
        goal_dict[key]      = goal



def import_waypoints(path_name, waypoints_path):
    # Read the waypoints_file
    waypoints = []
    waypoints_file = open(waypoints_path)
    for line in waypoints_file.readlines():
        waypoints.append([float(i) for i in line.split(',')])
    waypoints_file.close()

    # Generate the waypoints_info
    waypoints_info = Path()
    waypoints_info.header.frame_id = 'map'
    waypoints_info.header.stamp = rospy.Time.now()

    for idx in range(len(waypoints)):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position = Point(waypoints[idx][0], waypoints[idx][1], waypoints[idx][2])
        pose.pose.orientation = Quaternion(waypoints[idx][3], waypoints[idx][4], waypoints[idx][5], waypoints[idx][6])
        
        waypoints_info.poses.append(pose)
    
    # print('\tFetched ' + str(len(waypoints)) + ' waypoints from ' + str(waypoints_path))

    # if waypoints == []:
    #     rospy.signal_shutdown('No waypoint to draw... Shutdown')

    start   = tuple(waypoints[0])
    goal    = tuple(waypoints[-1])

    key             = path_name

    return waypoints_info, start, goal

class FollowRoute(State):
    def callback(msg):
        # rospy.loginfo("Received at goal message!")
        # rospy.loginfo("Timestamp: " + str(msg.header.stamp))
        # rospy.loginfo("frame_id: " + str(msg.header.frame_id))
        
        # Copying for simplicity
        position    = msg.pose.position
        quat        = msg.pose.orientation
        # rospy.loginfo("Point Position: [ %f, %f, %f ]"%(position.x, position.y, position.z))
        # rospy.loginfo("Quat Orientation: [ %f, %f, %f, %f]"%(quat.x, quat.y, quat.z, quat.w))

        # Also print Roll, Pitch, Yaw
        # euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        # rospy.loginfo("Euler Angles: %s"%str(euler))  

        mini_report = ["IDK", "GOAL_MSG", str(rospy.Time.now()), -1, -1, (position, quat)]
        mission_report_short.append(mini_report)

    def __init__(self, waypub, waypretty, sound_client):
        State.__init__(self, outcomes=['success'], input_keys=['waypoints'])

        self.frame_id = rospy.get_param('~goal_frame_id', 'map')
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')
        self.base_frame_id = rospy.get_param('~base_frame_id', 'base_link') # Change to base 
        self.duration = rospy.get_param('~wait_duration', 0.0)
        # Get a move_base action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Connecting to move_base...')
        self.client.wait_for_server()

        self.waypoint_pub            = waypub
        self.waypoint_pub_rviz       = waypretty
        self.sound_client            = sound_client 

        self.sound_success      = sound_client.waveSound('/home/tbd-fetch/fetch_ws/src/fetch_study/sounds/mission-success-41211.wav')
        self.sound_start        = sound_client.waveSound('/home/tbd-fetch/fetch_ws/src/fetch_study/sounds/car-engine-starting-43705.wav')

        rospy.loginfo('Connected to move_base.')
        rospy.loginfo('Starting a tf listener.')
        self.tf = TransformListener()
        self.listener = tf.TransformListener()
        self.distance_tolerance = 0.0 #.25 #rospy.get_param('waypoint_distance_tolerance', 0.0)

        # rospy.Subscriber("/move_base/goal", PoseStamped, goal_callback)

        # print("Setting up dynamic speed server")
        # self.update_client = dynamic_reconfigure.client.Client('pure_pursuit')
        # # self.update_client.wait_for_server()
        # self.update_client = dynamic_reconfigure.client.Client("move_base/TrajectoryPlannerROS", timeout=4, config_callback=None)
        # print("Found it")


    def distance_between_waypts(self, p1, p2):
        dist = np.sqrt((p1.pose.pose.position.x - p2.pose.pose.position.x)**2 + (p1.pose.pose.position.y - p2.pose.pose.position.y)**2)

        return dist

    def execute(self, userdata):
        global megapoints, mission_report, mission_report_short, waypoint_pub

        goal_a_ramp  = [1.0, -.65]
        goal_b_ramp  = [3.0, -.65]
        goal_c_ramp  = [5.0, -.65]

        goal_d_ramp  = [1.0, -3.35]
        goal_e_ramp  = [3.0, -3.35]
        goal_f_ramp  = [5.0, -3.35]

        goal_a_ramp_lite  = [1.0, -.7]
        goal_b_ramp_lite  = [3.0, -.7]
        goal_c_ramp_lite  = [5.0, -.7]

        goal_d_ramp_lite  = [1.0, -3.3]
        goal_e_ramp_lite  = [3.0, -3.3]
        goal_f_ramp_lite  = [5.0, -3.3]


        goal_a  = [1.0, -1.0]
        goal_b  = [3.0, -1.0] 
        goal_c  = [5.0, -1.0]

        goal_d  = [1.0, -3.0]
        goal_e  = [3.0, -3.0]
        goal_f  = [5.0, -3.0]


        # Execute waypoints each in sequence
        # prev_waypoint = waypoints[0]

        mega_target_counter = 0
        for megatarget, aux_data in zip(megapoints, auxilary_data):
            mega_target_counter += 1
            tic = time.perf_counter()

            megapoint, megaroute_name = megatarget
            goal                = goal_dict[megaroute_name]
            start               = start_dict[megaroute_name]
            gx, gy              = goal[0], goal[1]
            sx, sy              = start[0], start[1]
            path_to_broadcast   = route_dict[megaroute_name]
            # TODO verify this is good for first and last paths

            end = goal

            # Break if preempted
            if not megapoints:
                rospy.loginfo('The waypoint queue has been reset.')
                break

            print("OVERALL TARGET UPDATE! " + str(aux_data[AUX_WAYPOINT_INDEX]))
            print("Start")
            print(start)
            print("Goal")
            print(goal)

            ## Option for speed control
            # self.update_client.update_configuration({"linear_velocity": .1})
            # r.sleep()

            ### Set up goal checkpoints
            start_goal = MoveBaseGoal()
            start_goal.target_pose.header.frame_id = self.frame_id

            start_goal.target_pose.pose.position     = Point(start[0], start[1], start[2])
            start_goal.target_pose.pose.orientation  = Quaternion(start[3], start[4], start[5], start[6])

            end_target  = end
            end_ramp    = end
            if [end[0], end[1]] == goal_a_ramp:
                end_target  = goal_a
                end_ramp    = goal_a_ramp

            elif [end[0], end[1]] == goal_b_ramp:
                end_target  = goal_b
                end_ramp    = goal_b_ramp

            elif [end[0], end[1]] == goal_c_ramp:
                end_target  = goal_c
                end_ramp    = goal_c_ramp

            elif [end[0], end[1]] == goal_d_ramp:
                end_target  = goal_d
                end_ramp    = goal_d_ramp
            
            elif [end[0], end[1]] == goal_e_ramp:
                end_target  = goal_e
                end_ramp    = goal_e_ramp
            
            elif [end[0], end[1]] == goal_f_ramp:
                end_target  = goal_f
                end_ramp    = goal_f_ramp

            rospy.loginfo("Real end target")
            rospy.loginfo(end_target)

            end_goal = MoveBaseGoal()
            end_goal.target_pose.header.frame_id = self.frame_id

            end_goal.target_pose.pose.position     = Point(end[0], end[1], end[2])
            end_goal.target_pose.pose.orientation  = Quaternion(end[3], end[4], end[5], end[6])


            self.already_aligned_with_start_pose = False
            def start_callback_done(state, result):
                # print("Action server is done. State: %s, result: %s" % (str(state), str(result)))
                self.already_aligned_with_start_pose = True
                # print("Is primed to move on from start")

                mini_report = [str(aux_data[AUX_WAYPOINT_INDEX]), "START", str(rospy.Time.now()), 0, mega_target_counter]
                mission_report_short.append(mini_report)
                print("~~START~~")

            def end_callback_done(state, result):
                # print("Action server is done. State: %s, result: %s" % (str(state), str(result)))
                # self.already_aligned_with_start_pose = True
                # self.has_reached_endgoal = True
                print("Done with this path")
                # NOT ACTUALLY IN USE
                self.ready_for_next_chunk = True

            ##### Prep BEFORE we start the path
            mini_report = [str(aux_data[AUX_WAYPOINT_INDEX]), "PREP", str(rospy.Time.now()), 0, mega_target_counter]
            mission_report_short.append(mini_report)

            self.client.send_goal(start_goal, done_cb=start_callback_done)
            rospy.loginfo('Executing move_base goal to START position (x,y) with velocity: %s, %s, %s' % (sx, sy, -1))

            counter = 0

            self.has_reached_endgoal = False
            self.has_broadcast_curve = False
            while not self.has_reached_endgoal and not rospy.is_shutdown():
                counter += 1

                if self.already_aligned_with_start_pose and not self.has_reached_endgoal:
                    if not self.has_broadcast_curve:
                        self.waypoint_pub.publish(path_to_broadcast)

                        toc = time.perf_counter()
                        time_elapsed = toc - tic
                        mini_report = [str(aux_data[AUX_WAYPOINT_INDEX]), "LAUNCH", str(rospy.Time.now()), time_elapsed, mega_target_counter]
                        mission_report_short.append(mini_report)

                        # Mark the path's start when we start broadcasting the direction
                        
                        rospy.loginfo('Executing move_base goal to END position (x,y) of # waypoints: %s, %s, %s' % (gx, gy, len(path_to_broadcast.poses)))

                        ### If we were to do it this way
                        # self.client.send_goal(end_goal, done_cb=end_callback_done)
                        # rospy.loginfo('Executing move_base goal to END position (x,y) with velocity: %s, %s, %s' % (gx, gy, -1))

                        self.has_broadcast_curve = True


                    now = rospy.Time.now()
                    self.listener.waitForTransform('map', 'base_link', now, rospy.Duration(.6))
                    trans, rot = self.listener.lookupTransform('map', 'base_link', now)
                    distance_to_goal = math.sqrt(
                        pow(end_target[0] - trans[0], 2) + pow(end_target[1] - trans[1],
                                                                               2))

                    lidar_offset = .3
                    if (distance_to_goal - lidar_offset) <= self.distance_tolerance:
                        self.has_reached_endgoal = True
                        self.sound_success.play()
                        rospy.loginfo("ATTEMPT TO PLAY SOUND")
                    else:
                        time.sleep(self.duration)

                    if counter % 500 == 0 and distance_to_goal < .6:
                        print("Robot "  + str(distance_to_goal) + " from goal.")

                    # rospy.loginfo(str("base_link: ") + str(rospy.Time.now()) + ", " + str(trans) + ", " + str(rot), logger_name="location_logger")

                  
                    ### LOG PROGRESS ON PATH
                    toc = time.perf_counter()
                    step_time_elapsed = toc - tic
                    step_time_elapsed = str(step_time_elapsed)

                    report = [trans[0], trans[1], step_time_elapsed, str(rospy.Time.now()), rot]
                    mission_report.append(report)

                # time.sleep(self.duration)

            print("Huzzah! Exiting this loop, because we reached the goal!")
            # mission_report.append("REACHED " + str(megapoint))
            toc = time.perf_counter()
            time_elapsed = toc - tic

            mini_report = [str(aux_data[AUX_WAYPOINT_INDEX]), "END", str(rospy.Time.now()), time_elapsed, mega_target_counter]
            mission_report_short.append(mini_report)
            print("~~END~~")
            
            print(f"Leg " + str(aux_data[AUX_WAYPOINT_INDEX]) + " took " + str(time_elapsed) + " seconds")


            def parking_callback_done(state, result):
                # print("Action server is done. State: %s, result: %s" % (str(state), str(result)))
                # self.already_aligned_with_start_pose = True
                # self.has_reached_endgoal = True
                # print("Done with this path")

                ### Once parked
                toc = time.perf_counter()
                time_elapsed = toc - tic

                mini_report = [str(aux_data[AUX_WAYPOINT_INDEX]), "PARKED", str(rospy.Time.now()), time_elapsed, mega_target_counter]
                mission_report_short.append(mini_report)
                self.is_parked = True
                print("~~PARKED~~")

                
            self.is_parked = False
            end_ramp_goal = MoveBaseGoal()
            end_ramp_goal.target_pose.header.frame_id = self.frame_id

            end_ramp_goal.target_pose.pose.position     = Point(end_ramp[0], end_ramp[1], end[2])
            end_ramp_goal.target_pose.pose.orientation  = Quaternion(end[3], end[4], end[5], end[6])

            rospy.loginfo('Executing move_base goal to PARKED position (x,y) with velocity: %s, %s, %s' % (str(end_ramp[0]), str(end_ramp[1]), str(end[2])))
            # rospy.loginfo('at angle: %s, %s, %s, %s' % str(end[3]), str(end[4]), str(end[5]), str(end[6]))

            self.client.send_goal(end_ramp_goal, done_cb=parking_callback_done)

            while not self.is_parked and not rospy.is_shutdown():
                
                pass


            # print("Next loop?")

            ######## HALT THE ROBOT AT END OF PATH
            # blank_path = Path()
            # blank_path.header.frame_id = 'map'
            # blank_path.header.stamp = rospy.Time.now()
            # self.waypoint_pub.publish(blank_path)

            wait_time_at_goal = 0.25
            time.sleep(wait_time_at_goal)

            # toc = time.perf_counter()
            # time_elapsed = toc - tic
            # time_elapsed = str(time_elapsed)
            # # mission_report.append("REACHED " + str(megapoint))


        return 'success'


def convert_PoseWithCovArray_to_PoseArray(waypoints):
    """Used to publish waypoints as pose array so that you can see them in rviz, etc."""
    poses = PoseArray()
    poses.header.frame_id = rospy.get_param('~goal_frame_id', 'map')
    poses.poses = [pose.pose.pose for pose in waypoints]
    return poses


class GetRoute(State):
    def __init__(self, waypub, sound_client):
        State.__init__(self, outcomes=['success', 'killed'], input_keys=['waypoints'], output_keys=['waypoints'])
        # Parameters
        self.goal_frame_id = rospy.get_param('~goal_frame_id', 'map')

        self.waypoint_pub = waypub

        self.sound_client            = sound_client 

        self.sound_success      = sound_client.waveSound('/home/tbd-fetch/fetch_ws/src/fetch_study/sounds/mission-success-41211.wav')
        self.sound_start        = sound_client.waveSound('/home/tbd-fetch/fetch_ws/src/fetch_study/sounds/car-engine-starting-43705.wav')

        # Subscribe to pose message to get new waypoints
        # self.addpose_topic = rospy.get_param('~addpose_topic', '/initialpose') // removed since not adding waypoints
        # Create publisher to publish waypoints as pose array so that you can see them in rviz, etc.
        # self.posearray_topic = rospy.get_param('~posearray_topic', '/megapoints')
        # self.poseArray_publisher = rospy.Publisher(self.posearray_topic, PoseArray, queue_size=1)

        # Path for saving and retrieving the pose.csv file
        output_folder_default = os.path.join(rospkg.RosPack().get_path('fetch_study'), 'saved_path')
        output_folder = rospy.get_param('~output_folder', output_folder_default)
        if not os.path.isdir(output_folder):
            os.makedirs(output_folder)

        input_file_name = rospy.get_param('~input_filename', 'waypoints.csv')
        self.input_file_path = os.path.join(output_folder, input_file_name)

        output_file_name = rospy.get_param('~output_filename', 'waypoints.csv')
        self.output_file_path = os.path.join(output_folder, output_file_name)

        # Start thread to listen for reset messages to clear the waypoint queue
        def wait_for_path_reset():
            """thread worker function"""
            global megapoints
            while not rospy.is_shutdown():
                try:
                    data = rospy.wait_for_message('/path_reset', Empty)
                except rospy.ROSInterruptException:
                    break
                rospy.loginfo('Received path RESET message')
                self.initialize_path_queue()
                rospy.sleep(3)  # Wait 3 seconds because `rostopic echo` latches
                # for three seconds and wait_for_message() in a
                # loop will see it again.

        reset_thread = threading.Thread(target=wait_for_path_reset)
        reset_thread.start()

    def initialize_path_queue(self):
        global megapoints
        megapoints = []  # the waypoint queue
        # publish empty waypoint queue as pose array so that you can see them the change in rviz, etc.
        # self.poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(megapoints))

    def execute(self, userdata):
        global megapoints, route_dict, route_sequence
        self.initialize_path_queue()
        self.path_ready = False

        # Start thread to listen for when the path is ready (this function will end then)
        # Also will save the clicked path to pose.csv file
        def wait_for_path_ready():
            """thread worker function"""
            try:
                data = rospy.wait_for_message('/path_ready', Empty)
            except rospy.ROSInterruptException:
                return
            rospy.loginfo('Received path READY message')
            self.path_ready = True
            with open(self.output_file_path, 'w') as file:
                for current_pose in waypoints:
                    file.write(str(current_pose.pose.pose.position.x) + ',' + str(
                        current_pose.pose.pose.position.y) + ',' + str(current_pose.pose.pose.position.z) + ',' + str(
                        current_pose.pose.pose.orientation.x) + ',' + str(
                        current_pose.pose.pose.orientation.y) + ',' + str(
                        current_pose.pose.pose.orientation.z) + ',' + str(current_pose.pose.pose.orientation.w) + ',' + "1.0" + '\n')
                rospy.loginfo('poses written to ' + self.output_file_path)

        ready_thread = threading.Thread(target=wait_for_path_ready)
        ready_thread.start()

        initialize_waypoints()
        print("Imported all waypoint options")

        self.start_journey_bool = False

        # Start thread to listen start_jorney 
        # for loading the saved poses from saved_path/poses.csv
        def wait_for_start_journey():
            """thread worker function"""
            try:
                data_from_start_journey = rospy.wait_for_message('start_journey', String)

            except rospy.ROSInterruptException:
                return 'killed'
            rospy.loginfo('Received path READY start_journey')

            route_sequence  = get_exp_sequence(data_from_start_journey)

            first_there = False
            for route_key in route_sequence:
                waypoints_info  = route_dict[route_key]
                start           = start_dict[route_key]
                goal            = goal_dict[route_key]

                # if first_there == False:
                #     megapoints.append((start, route_key))
                #     first_there = True

                megapoints.append((goal, route_key))
                auxilary_data.append([route_key, waypoints_info, start, goal])



            # ##### Import CSV document
            # with open(self.input_file_path, 'r') as file:
            #     reader = csv.reader(file, delimiter=',')
            #     row_id = 0
            #     for row in reader:
            #         # print(row)
            #         current_pose = PoseWithCovarianceStamped()
            #         current_pose.pose.pose.position.x = float(row[0])
            #         current_pose.pose.pose.position.y = float(row[1])
            #         current_pose.pose.pose.position.z = float(row[2])
            #         current_pose.pose.pose.orientation.x = float(row[3])
            #         current_pose.pose.pose.orientation.y = float(row[4])
            #         current_pose.pose.pose.orientation.z = float(row[5])
            #         current_pose.pose.pose.orientation.w = float(row[6])
            #         waypoints.append(current_pose)

            #         # Adjust the speed on the fly
            #         if len(row) > 7:
            #             velocity = float(row[7])
            #             aux_velocity = velocity
            #         else:
            #             aux_velocity = velocity_default

            #         aux_data = [row_id, aux_velocity]
            #         row_id += 1

            #         auxilary_data.append(aux_data)


            #         # TODO also add stop-time-on-arrival

            #     self.poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))

            self.start_journey_bool = True
            self.sound_start.play()
            rospy.loginfo("ATTEMPT TO PLAY SOUND")

        start_journey_thread = threading.Thread(target=wait_for_start_journey)
        start_journey_thread.start()

        # topic = self.addpose_topic
        # rospy.loginfo("Waiting to recieve waypoints via Pose msg on topic %s" % topic)
        # rospy.loginfo("To start following waypoints: 'rostopic pub /path_ready std_msgs/Empty -1'")
        # rospy.loginfo("OR")
        rospy.loginfo("To start following saved waypoints: 'rostopic pub /start_journey std_msgs/String \"data: -1\"")

        # Wait for published waypoints or saved path  loaded
        while not self.path_ready and not self.start_journey_bool and not rospy.is_shutdown():
            try:
                pass
            #     pose = rospy.wait_for_message(topic, PoseWithCovarianceStamped, timeout=1)
            #     rospy.loginfo("Received new waypoint")
            #     waypoints.append(changePose(pose, self.goal_frame_id))
            #     # publish waypoint queue as pose array so that you can see them in rviz, etc.
            #     self.poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))

            except rospy.ROSInterruptException:
                rospy.logwarn("Shutting down")
                return 'killed'

            except rospy.ROSException as e:
                rospy.logwarn_throttle(5, "Ros exception: {}".format(e))

        # Path is ready! return success and move on to the next state (FOLLOW_PATH)
        return 'success'


class RouteComplete(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        global mission_report, mission_report_short

        rospy.loginfo('###############################')
        rospy.loginfo('##### REACHED FINISH GATE #####')
        rospy.loginfo('###############################')

        now_id = datetime.now().strftime("%Y_%m_%d-%I_%M_%S_%p")

        output_folder_default = os.path.join(rospkg.RosPack().get_path('fetch_study'), 'saved_path/reports')
        output_folder = rospy.get_param('~output_folder', output_folder_default)

        output_file_path_minireport = os.path.join(output_folder, now_id + "-mini_report.csv")
        with open(output_file_path_minireport, 'w') as file:
            file.write("point, status, time, time_elapsed, iteration_number, aux\n")
            for report in mission_report_short:
                try:
                    if len(report) == 5:
                        file.write(str(report[0]) + ',' + str(report[1]) + ',' + str(report[2]) + ',' + str(report[3]) + ',' + str(report[4]) + '\n')
                    elif len(report) == 6:
                        file.write(str(report[0]) + ',' + str(report[1]) + ',' + str(report[2]) + ',' + str(report[3]) + ',' + str(report[4]) + ',' + str(report[5]) + '\n')
                except:
                    file.write(str(report) + "\n")

        
        print("mission report as length " + str(len(mission_report)))
        output_file_path_report = os.path.join(output_folder, now_id + "-mission_report.csv")
        with open(output_file_path_report, 'w') as file:
            file.write("X, Y, time_to_reach, time\n")
            for report in mission_report:
                if ',' in report and len(report) == 4:
                    file.write(str(report[0]) + ',' + str(report[1]) + ',' + str(report[2]) + ',' + str(report[3]) + ',' + str(report[4]) + '\n')
                else:
                    file.write(str(report) + "\n")
            
        rospy.loginfo('Mission report filed to ' + output_file_path_report)
        exit()
        
        return 'success'


def main():
    rospy.init_node('publish_path')
    # publish_path
    waypoint_pub            = rospy.Publisher('/path_for_tracking_pid', Path, queue_size=1)
    waypoint_pub_vis        = rospy.Publisher('/local_path', Path, queue_size=1)

    sound_client = SoundClient()

    sm = StateMachine(outcomes=['success'])
    with sm:
        StateMachine.add('GET_PATH', GetRoute(waypoint_pub, sound_client),
                         transitions={'success': 'FOLLOW_PATH', 'killed': 'success'},
                         remapping={'waypoints': 'waypoints'})
        StateMachine.add('FOLLOW_PATH', FollowRoute(waypoint_pub, waypoint_pub_vis, sound_client),
                         transitions={'success': 'PATH_COMPLETE'},
                         remapping={'waypoints': 'waypoints'})
        StateMachine.add('PATH_COMPLETE', RouteComplete(),
                         transitions={'success': 'GET_PATH'})

    sm.execute()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        rospy.logwarn("Node shutdown")