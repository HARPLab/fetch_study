#!/usr/bin/env python3

# Copyright (c) 2015, Fetch Robotics Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Fetch Robotics Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL FETCH ROBOTICS INC. BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Author: Michael Ferguson

import copy
import actionlib
import rospy

from math import sin, cos
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

robot_plan = []

# Move base using navigation stack
class MoveBaseClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()


    def goto_pt(self, point, frame="map"):
        x, y, theta = point

        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        # TODO wait for things to work
        self.client.send_goal(move_goal)
        self.client.wait_for_result()

    def goto(self, x, y, theta, frame="map"):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        # TODO wait for things to work
        self.client.send_goal(move_goal)
        self.client.wait_for_result()

    def travel_path(self, path):
        for pt in path:
            self.goto_pt(path)

# Point the head using controller
class PointHeadClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()

    def look_at(self, x, y, z, frame, duration=1.0):
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(goal)
        self.client.wait_for_result()


def setup_robot_plan():
    path = []
    path.append((3.886, -4.470, 0.021))
    path.append((4.488, -3.901, 0.896))
    path.append((4.826, -3.392, 0.720))
    path.append((4.532, -3.132, 2.407))
    path.append((4.171, -3.151, 3.130))
    path.append((3.786, -3.170, -3.114))
    path.append((3.860, -3.682, -1.309))
    path.append((3.814, -4.423, -1.612))

    robot_plan.append(path)
    robot_plan.append(("STOP", 5))
    robot_plan.append(path)

    return robot_plan

def run_robot_plan(robot_plan, move_base):
    for stage in robot_plan:
        if len(stage) == 1:
            if stage[0] == "STOP":
                stop_time = stage[1]
                rospy.sleep(stop_time)
            else:
                path = stage
                move_base.travel_path(path)

def file_mission_report(self):
    now_id = datetime.now().strftime("%Y_%m_%d-%I_%M_%S_%p")

    output_folder_default = os.path.join(rospkg.RosPack().get_path('fetch_study'), 'saved_path/reports')
    output_folder = rospy.get_param('~output_folder', output_folder_default)
    output_file_path_report = os.path.join(output_folder, now_id + "-mission_report.csv")
    with open(output_file_path_report, 'w') as file:
        file.write("X, Y, time_to_reach\n")
        for report in mission_report:
            file.write(str(report[0]) + ',' + str(report[1]) + ',' + str(report[2]) + '\n')
        
    rospy.loginfo('Mission report filed to ' + output_file_path_report)



if __name__ == "__main__":
    # Create a node
    rospy.init_node("demo")

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    # Setup clients
    move_base = MoveBaseClient()
    # head_action = PointHeadClient()

    # Move the base to be in front of the table
    # Demonstrates the use of the navigation stack
    rospy.loginfo("Moving to table...")
 
    # import 
    input_file_name = rospy.get_param('~input_filename', 'waypoints.csv')
    self.input_file_path = os.path.join(output_folder, input_file_name)

    output_file_name = rospy.get_param('~output_filename', 'waypoints.csv')
    self.output_file_path = os.path.join(output_folder, output_file_name)

    robot_plan = setup_robot_plan()

    mission_report = run_robot_plan(robot_plan, move_base)
    file_mission_report(mission_report)













