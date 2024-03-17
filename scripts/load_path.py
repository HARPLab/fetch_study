#!/usr/bin/env python3
# coding:utf-8

import rospy
import os
import rospkg
import tf
from tf import TransformListener
import numpy as np
import threading
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Empty, String

AT_GOAL_DISTANCE = .05


class PathManager():
    waypoints_dict = {}
    path_ready = False

    def __init__(self):
        #### Set up the node
        rospy.init_node('waypoints_loader')
        self.waypoint_pub = rospy.Publisher('/waypoints', Path, queue_size=1)
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')
        self.base_frame_id = rospy.get_param('~base_frame_id', 'base_link')

        self.rate = rospy.Rate(10) # Rate is in hz

        rospy.loginfo('Starting a tf listener.')
        self.tf         = TransformListener()
        self.listener   = tf.TransformListener()

        self.waypoints_dict = self.get_waypoints()
        self.path_ready     = True

        print("Setting up points now")

        self.start_journey_bool = False
        # Start thread to listen start_jorney 
        # for loading the saved poses from saved_path/poses.csv
        def wait_for_start_journey():
            """thread worker function"""
            try:
                data_from_start_journey = rospy.wait_for_message('start_journey', Empty)
            except rospy.ROSInterruptException:
                return 'killed'
            rospy.loginfo('Received path READY start_journey')
            self.start_journey_bool = True


        start_journey_thread = threading.Thread(target=wait_for_start_journey)
        start_journey_thread.start()
        rospy.loginfo("To start following saved waypoints: 'rostopic pub /start_journey std_msgs/Empty -1'")

        # Wait for published waypoints or saved path  loaded
        while not self.path_ready and not self.start_journey_bool:
            key, path = self.determine_next_path(self.waypoints_dict)
            self.broadcast_single_path(key, path)


    def get_waypoints(self):
        output_folder_default = os.path.join(rospkg.RosPack().get_path('fetch_study'), 'waypoints/')
        output_folder = rospy.get_param('~output_folder', output_folder_default)

        path_dict = {}

        for path_name in ['waypoints']:
            waypoints_path = output_folder_default + path_name + ".csv"
        
            key, waypoints_info      = self.load_waypoints(path_name, waypoints_path)
            
            # KEY = (name, first, last)
            path_dict[key]  = waypoints_info

        return path_dict


    def load_waypoints(self, path_name, waypoints_path):
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
        
        rospy.loginfo('Fetched ' + str(len(waypoints)) + ' waypoints from ' + str(waypoints_path))

        if waypoints == []:
            rospy.signal_shutdown('No waypoint to draw... Shutdown')

        start   = tuple(waypoints[0])
        goal    = tuple(waypoints[-1])

        key             = (path_name, start, goal)

        return key, waypoints_info

    def determine_next_path(self, waypoints_dict):
        # Get a path that starts where we are
        # Move along that path
        
        now = rospy.Time.now()
        self.listener.waitForTransform(self.odom_frame_id, self.base_frame_id, now, rospy.Duration(4))
        trans, rot = self.listener.lookupTransform(self.odom_frame_id, self.base_frame_id, now)
        
        target_key = None
        for key in waypoints_dict.keys():
            pname, pstart, pgoal = key

            X_INDEX, Y_INDEX = 0, 1
            distance = np.sqrt(
                pow(pstart[X_INDEX] - trans[0], 2) 
                    + pow(pstart[Y_INDEX] - trans[1], 2))

            print("Robot "  + str(distance) + " from path start.")
            # distance_left = distance

            if distance < .1:
                target_key = key

        return key, waypoints_dict[key]


    def broadcast_single_path(self, key, path_to_broadcast):
        pname, pstart, pgoal = key

        self.broadcast_on   = True
        at_goal             = False
        try:
            while not rospy.is_shutdown() and not at_goal and self.broadcast_on:
                now = rospy.Time.now()
            
                self.listener.waitForTransform(self.odom_frame_id, self.base_frame_id, now, rospy.Duration(1))
                trans, rot = self.listener.lookupTransform(self.odom_frame_id, self.base_frame_id, now)
 
                distance = np.sqrt(
                    pow(pgoal[0] - trans[0], 2) 
                        + pow(pgoal[1] - trans[1], 2))

                print("Robot "  + str(distance) + " from goal.")
                
                if distance < AT_GOAL_DISTANCE:
                    at_goal = True
                    print("Reached goal!")

                # Then continue on the path
                self.waypoint_pub.publish(path_to_broadcast)
                self.rate.sleep()


        except rospy.ROSInterruptException:
            rospy.logerr('Get KeyBoardInterrupt... Shutdown')

        print("Successsfully completed path, moving onto the next")



if __name__ == "__main__":
    pm = PathManager()
