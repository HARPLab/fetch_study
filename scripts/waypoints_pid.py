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

lidar_offset = .25

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

    if exp_option == "1":
        return ['AC_OBS-late', 'CD_OBS-late', 'DF-early', 'FE-even', 'ED-null', 'DA-even', 'AF_OBS-early', 'FD_OBS-late', 'DA-even', 'AD-even', 'DC-null', 'CE-even', 'EA-null', 'AC-late', 'CB-even', 'BD-null', 'DF-even', 'FD_OBS-early', 'DA-late', 'AE-late', 'EC-null', 'CF-late', 'FE-late', 'EF-null', 'FA_OBS-early', 'AF_OBS-even', 'FA-null', 'AB-late', 'BA-null', 'AC-early', 'CA_OBS-early', 'AC-even', 'CF-early', 'FE-early', 'EC-null', 'CE-early', 'EA-null', 'AB-early', 'BA-null', 'AF_OBS-late', 'FC-early', 'CA_OBS-even', 'AC-even', 'CD_OBS-even', 'DF-late', 'FD_OBS-even']
    elif exp_option == "2":
        return ['AC_OBS-early', 'CD_OBS-early', 'DF-even', 'FE-early', 'ED-null', 'DA-early', 'AF_OBS-even', 'FD_OBS-early', 'DA-early', 'AD-early', 'DC-null', 'CE-late', 'EA-null', 'AC-even', 'CB-early', 'BD-null', 'DF-late', 'FD_OBS-even', 'DA-even', 'AE-early', 'EC-null', 'CF-even', 'FE-even', 'EF-null', 'FA_OBS-even', 'AF_OBS-late', 'FA-null', 'AB-even', 'BA-null', 'AC-late', 'CA_OBS-even', 'AC-early', 'CF-late', 'FE-late', 'EC-null', 'CE-even', 'EA-null', 'AB-late', 'BA-null', 'AF_OBS-early', 'FC-late', 'CA_OBS-late', 'AC-early', 'CD_OBS-late', 'DF-early', 'FD_OBS-late']
    elif exp_option == "3":
        return ['AC_OBS-even', 'CD_OBS-even', 'DF-late', 'FE-late', 'ED-null', 'DA-late', 'AF_OBS-late', 'FD_OBS-even', 'DA-late', 'AD-late', 'DC-null', 'CE-early', 'EA-null', 'AC-early', 'CB-late', 'BD-null', 'DF-early', 'FD_OBS-late', 'DA-early', 'AE-even', 'EC-null', 'CF-early', 'FE-early', 'EF-null', 'FA_OBS-late', 'AF_OBS-early', 'FA-null', 'AB-early', 'BA-null', 'AC-even', 'CA_OBS-late', 'AC-late', 'CF-even', 'FE-even', 'EC-null', 'CE-late', 'EA-null', 'AB-even', 'BA-null', 'AF_OBS-even', 'FC-even', 'CA_OBS-early', 'AC-late', 'CD_OBS-early', 'DF-even', 'FD_OBS-early']
    elif exp_option == "4":
        return ['AC-even', 'CD_OBS-even', 'DA-late', 'AB-late', 'BC-null', 'CE-early', 'EA-null', 'AD-early', 'DC_OBS-early', 'CA_OBS-late', 'AD-early', 'DE-early', 'ED-null', 'DF_OBS-late', 'FD-late', 'DF-early', 'FA_OBS-late', 'AF_OBS-even', 'FD_OBS-early', 'DA-early', 'AB-early', 'BA-null', 'AC_OBS-early', 'CA-late', 'AE-even', 'ED-null', 'DE-even', 'EA-null', 'AD-even', 'DE-late', 'EC-null', 'CA_OBS-even', 'AF_OBS-late', 'FD-even', 'DA-even', 'AD-late', 'DF_OBS-even', 'FA_OBS-early', 'AE-late', 'EC-null', 'CA-early', 'AB-even']
    elif exp_option == "5":
        return ['AC-late', 'CD_OBS-late', 'DA-early', 'AB-even', 'BC-null', 'CE-late', 'EA-null', 'AD-late', 'DC_OBS-late', 'CA_OBS-even', 'AD-late', 'DE-even', 'ED-null', 'DF_OBS-even', 'FD-even', 'DF-late', 'FA_OBS-early', 'AF_OBS-early', 'FD_OBS-late', 'DA-even', 'AB-late', 'BA-null', 'AC_OBS-late', 'CA-early', 'AE-early', 'ED-null', 'DE-late', 'EA-null', 'AD-early', 'DE-early', 'EC-null', 'CA_OBS-early', 'AF_OBS-even', 'FD-early', 'DA-late', 'AD-even', 'DF_OBS-early', 'FA_OBS-even', 'AE-even', 'EC-null', 'CA-even', 'AB-early']
    elif exp_option == "6":
        return ['AC-early', 'CD_OBS-early', 'DA-even', 'AB-early', 'BC-null', 'CE-even', 'EA-null', 'AD-even', 'DC_OBS-even', 'CA_OBS-early', 'AD-even', 'DE-late', 'ED-null', 'DF_OBS-early', 'FD-early', 'DF-even', 'FA_OBS-even', 'AF_OBS-late', 'FD_OBS-even', 'DA-late', 'AB-even', 'BA-null', 'AC_OBS-even', 'CA-even', 'AE-late', 'ED-null', 'DE-early', 'EA-null', 'AD-late', 'DE-even', 'EC-null', 'CA_OBS-late', 'AF_OBS-early', 'FD-late', 'DA-early', 'AD-early', 'DF_OBS-late', 'FA_OBS-late', 'AE-early', 'EC-null', 'CA-late', 'AB-late']
    elif exp_option == "7":
        return ['AC-late', 'CA_OBS-early', 'AE-early', 'EF-null', 'FD-even', 'DE-early', 'EA-null', 'AF_OBS-early', 'FA_OBS-late', 'AB-late', 'BF-null', 'FC-even', 'CF-late', 'FD_OBS-late', 'DF-late', 'FD_OBS-early', 'DC_OBS-even', 'CF-early', 'FE-even', 'EA-null', 'AE-even', 'EF-null', 'FC-late', 'CB-early', 'BC-null', 'CA-early', 'AC_OBS-even', 'CD_OBS-early', 'DA-early', 'AC_OBS-late', 'CE-late', 'EA-null', 'AB-even', 'BA-null', 'AC-even', 'CF-even', 'FA_OBS-even', 'AF_OBS-late', 'FD-early', 'DE-late', 'EF-null', 'FD_OBS-even']
    elif exp_option == "8":
        return ['AC-early', 'CA_OBS-even', 'AE-late', 'EF-null', 'FD-early', 'DE-late', 'EA-null', 'AF_OBS-even', 'FA_OBS-even', 'AB-early', 'BF-null', 'FC-early', 'CF-early', 'FD_OBS-even', 'DF-even', 'FD_OBS-late', 'DC_OBS-late', 'CF-even', 'FE-early', 'EA-null', 'AE-early', 'EF-null', 'FC-even', 'CB-even', 'BC-null', 'CA-even', 'AC_OBS-late', 'CD_OBS-late', 'DA-late', 'AC_OBS-early', 'CE-even', 'EA-null', 'AB-late', 'BA-null', 'AC-late', 'CF-late', 'FA_OBS-early', 'AF_OBS-early', 'FD-late', 'DE-even', 'EF-null', 'FD_OBS-early']
    elif exp_option == "9":
        return ['AC-even', 'CA_OBS-late', 'AE-even', 'EF-null', 'FD-late', 'DE-even', 'EA-null', 'AF_OBS-late', 'FA_OBS-early', 'AB-even', 'BF-null', 'FC-late', 'CF-even', 'FD_OBS-early', 'DF-early', 'FD_OBS-even', 'DC_OBS-early', 'CF-late', 'FE-late', 'EA-null', 'AE-late', 'EF-null', 'FC-early', 'CB-late', 'BC-null', 'CA-late', 'AC_OBS-early', 'CD_OBS-even', 'DA-even', 'AC_OBS-even', 'CE-early', 'EA-null', 'AB-early', 'BA-null', 'AC-early', 'CF-early', 'FA_OBS-late', 'AF_OBS-even', 'FD-even', 'DE-early', 'EF-null', 'FD_OBS-late']
    elif exp_option == "10":
        return ['CF-late', 'FE-late', 'EF-null', 'FD_OBS-even', 'DA-late', 'AC_OBS-early', 'CA-late', 'AF_OBS-late', 'FD-late', 'DA-late', 'AB-early', 'BC-null', 'CE-even', 'EC-null', 'CD_OBS-even', 'DF-early', 'FA_OBS-late', 'AD-early', 'DA-early', 'AC-early', 'CB-even', 'BA-null', 'AC_OBS-even', 'CE-late', 'ED-null', 'DF_OBS-late', 'FE-early', 'EA-null', 'AF_OBS-early', 'FD_OBS-early', 'DF-even', 'FC-even', 'CF-even', 'FA_OBS-early', 'AF_OBS-even', 'FE-even', 'EA-null', 'AC_OBS-late', 'CA-even', 'AE-early', 'EA-null', 'AB-late']
    elif exp_option == "11":
        return ['CF-even', 'FE-even', 'EF-null', 'FD_OBS-early', 'DA-even', 'AC_OBS-even', 'CA-early', 'AF_OBS-even', 'FD-even', 'DA-even', 'AB-late', 'BC-null', 'CE-early', 'EC-null', 'CD_OBS-early', 'DF-late', 'FA_OBS-even', 'AD-late', 'DA-late', 'AC-even', 'CB-early', 'BA-null', 'AC_OBS-late', 'CE-even', 'ED-null', 'DF_OBS-even', 'FE-late', 'EA-null', 'AF_OBS-late', 'FD_OBS-late', 'DF-early', 'FC-early', 'CF-early', 'FA_OBS-late', 'AF_OBS-early', 'FE-early', 'EA-null', 'AC_OBS-early', 'CA-late', 'AE-late', 'EA-null', 'AB-even']
    elif exp_option == "12":
        return ['CF-early', 'FE-early', 'EF-null', 'FD_OBS-late', 'DA-early', 'AC_OBS-late', 'CA-even', 'AF_OBS-early', 'FD-early', 'DA-early', 'AB-even', 'BC-null', 'CE-late', 'EC-null', 'CD_OBS-late', 'DF-even', 'FA_OBS-early', 'AD-even', 'DA-even', 'AC-late', 'CB-late', 'BA-null', 'AC_OBS-early', 'CE-early', 'ED-null', 'DF_OBS-early', 'FE-even', 'EA-null', 'AF_OBS-even', 'FD_OBS-even', 'DF-late', 'FC-late', 'CF-late', 'FA_OBS-even', 'AF_OBS-late', 'FE-late', 'EA-null', 'AC_OBS-even', 'CA-early', 'AE-even', 'EA-null', 'AB-early']

    elif exp_option == "test":
        return ["AF-null", "FE-null", "EA-null", "AC-null", "CB-null", "BD-null", "DE-null", "EA-null"]

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

    for idx in range(len(waypoints)-1):
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
    def __init__(self, waypub, waypretty):
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

        rospy.loginfo('Connected to move_base.')
        rospy.loginfo('Starting a tf listener.')
        self.tf = TransformListener()
        self.listener = tf.TransformListener()
        self.distance_tolerance = 0.0 #.25 #rospy.get_param('waypoint_distance_tolerance', 0.0)

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

        # Execute waypoints each in sequence
        # prev_waypoint = waypoints[0]

        for megatarget, aux_data in zip(megapoints, auxilary_data):
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

            print("OVERALL TARGET UPDATE!")
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


            end_goal = MoveBaseGoal()
            end_goal.target_pose.header.frame_id = self.frame_id

            end_goal.target_pose.pose.position     = Point(end[0], end[1], end[2])
            end_goal.target_pose.pose.orientation  = Quaternion(end[3], end[4], end[5], end[6])


            self.already_aligned_with_start_pose = False
            def start_callback_done(state, result):
                # print("Action server is done. State: %s, result: %s" % (str(state), str(result)))
                self.already_aligned_with_start_pose = True
                print("Is primed to move on from start")

            def end_callback_done(state, result):
                # print("Action server is done. State: %s, result: %s" % (str(state), str(result)))
                # self.already_aligned_with_start_pose = True
                # self.has_reached_endgoal = True
                print("Done with this path")

            self.client.send_goal(start_goal, done_cb=start_callback_done)

            mini_report = [str(aux_data[AUX_WAYPOINT_INDEX]), "PREP", str(rospy.Time.now()), 0]
            mission_report_short.append(mini_report)
            
            rospy.loginfo('Executing move_base goal to START position (x,y) with velocity: %s, %s, %s' % (sx, sy, -1))

            counter = 0

            self.has_reached_endgoal = False
            self.has_broadcast_curve = False
            while not self.has_reached_endgoal and not rospy.is_shutdown():
                counter += 1

                if False and "OLD SCHOOL JUST GOAL MODE":
                    # rospy.loginfo("To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")
                    # self.client.send_goal(goal)
                    print("Wrong slot, not publishing")
                    pass
                else: #### NEW MORE ELABORATE METHOD
                    if counter % 10 == 0:
                        # print("publishing path")
                        # print("pass")
                        pass


                if self.already_aligned_with_start_pose and not self.has_reached_endgoal:
                    if not self.has_broadcast_curve:
                        self.waypoint_pub.publish(path_to_broadcast)

                        # Mark the path's start when we start broadcasting the direction
                        mini_report = [str(aux_data[AUX_WAYPOINT_INDEX]), "START", str(rospy.Time.now()), 0]
                        mission_report_short.append(mini_report)

                        rospy.loginfo('Executing move_base goal to END position (x,y) of # waypoints: %s, %s, %s' % (gx, gy, len(path_to_broadcast.poses)))
                        # self.client.send_goal(end_goal, done_cb=end_callback_done)
                        # rospy.loginfo('Executing move_base goal to END position (x,y) with velocity: %s, %s, %s' % (gx, gy, -1))

                        self.has_broadcast_curve = True
                    # else:
                    #     blank_path = Path()
                    #     self.waypoint_pub.publish(blank_path)


                    now = rospy.Time.now()

                    self.listener.waitForTransform('map', 'base_link', now, rospy.Duration(.6))
                    trans, rot = self.listener.lookupTransform('map', 'base_link', now)
                    distance_to_goal = math.sqrt(
                        pow(end_goal.target_pose.pose.position.x - trans[0], 2) + pow(end_goal.target_pose.pose.position.y - trans[1],
                                                                               2))

                    if (distance_to_goal - lidar_offset) <= self.distance_tolerance:
                        self.has_reached_endgoal = True

                    # if counter % 500 == 0:
                        print("Robot "  + str(distance_to_goal) + " from goal.")

                    # rospy.loginfo(str("base_link: ") + str(rospy.Time.now()) + ", " + str(trans) + ", " + str(rot), logger_name="location_logger")

                  
                    ### LOG PROGRESS ON PATH
                    toc = time.perf_counter()
                    step_time_elapsed = toc - tic
                    step_time_elapsed = str(step_time_elapsed)

                    report = [trans[0], trans[1], step_time_elapsed, str(rospy.Time.now())]
                    mission_report.append(report)

                time.sleep(self.duration)

            print("Huzzah! Exiting this loop, because we reached the goal!")

            mission_report.append("REACHED " + str(megapoint))
            toc = time.perf_counter()
            time_elapsed = toc - tic
            time_elapsed = str(time_elapsed)

            mini_report = [str(aux_data[AUX_WAYPOINT_INDEX]), "END", str(rospy.Time.now()), time_elapsed]
            mission_report_short.append(mini_report)
            
            print(f"Leg " + str(aux_data[AUX_WAYPOINT_INDEX]) + " took " + time_elapsed + " seconds")

            ######## HALT THE ROBOT AT END OF PATH
            blank_path = Path()
            blank_path.header.frame_id = 'map'
            blank_path.header.stamp = rospy.Time.now()
            self.waypoint_pub.publish(blank_path)

            wait_time_at_goal = 0.0
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
    def __init__(self, waypub):
        State.__init__(self, outcomes=['success', 'killed'], input_keys=['waypoints'], output_keys=['waypoints'])
        # Parameters
        self.goal_frame_id = rospy.get_param('~goal_frame_id', 'map')

        self.waypoint_pub = waypub

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
            file.write("point, status, time\n")
            for report in mission_report_short:
                    file.write(str(report) + "\n")

        output_file_path_report = os.path.join(output_folder, now_id + "-mission_report.csv")
        with open(output_file_path_report, 'w') as file:
            file.write("X, Y, time_to_reach\n")
            for report in mission_report:
                if ',' in report:
                    file.write(str(report[0]) + ',' + str(report[1]) + ',' + str(report[2]) + ',' + str(report[3]) + '\n')
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

    sm = StateMachine(outcomes=['success'])
    with sm:
        StateMachine.add('GET_PATH', GetRoute(waypoint_pub),
                         transitions={'success': 'FOLLOW_PATH', 'killed': 'success'},
                         remapping={'waypoints': 'waypoints'})
        StateMachine.add('FOLLOW_PATH', FollowRoute(waypoint_pub, waypoint_pub_vis),
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