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

smach.set_loggers(rospy.logdebug, rospy.logwarn, rospy.logdebug, rospy.logerr)

# Waypoints container
waypoints       = []
auxilary_data   = []
mission_report  = []

route_dict      = {}
route_sequence  = []
start_dict      = {}    
goal_dict       = {}


velocity_default = 1.0

# INDICES OF AUXILLARY DATA
AUX_WAYPOINT_INDEX  = 0
AUX_VELOCITY        = 1

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
    if exp_option == 0:
        return ['ba', 'ab']
    else:
        return ['ab', 'ba', 'ac', 'cb']

def initialize_waypoints():
    print("Get waypoints ==> ie detailed trajectories")
    output_folder_default = os.path.join(rospkg.RosPack().get_path('fetch_study'), 'waypoints/')
    output_folder = rospy.get_param('~output_folder', output_folder_default)

    for path_name in ['ab', 'ba']:
        waypoints_path = output_folder_default + path_name + ".csv"
    
        key, waypoints_info, start, goal      = import_waypoints(path_name, waypoints_path)
        
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
    
    rospy.loginfo('Fetched ' + str(len(waypoints)) + ' waypoints from ' + str(waypoints_path))

    # if waypoints == []:
    #     rospy.signal_shutdown('No waypoint to draw... Shutdown')

    start   = tuple(waypoints[0])
    goal    = tuple(waypoints[-1])

    key             = path_name

    return key, waypoints_info, start, goal


class FollowRoute(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'], input_keys=['waypoints'])
        self.frame_id = rospy.get_param('~goal_frame_id', 'map')
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')
        self.base_frame_id = rospy.get_param('~base_frame_id', 'base_footprint')
        self.duration = rospy.get_param('~wait_duration', 0.0)
        # Get a move_base action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Connecting to move_base...')
        self.client.wait_for_server()

        rospy.loginfo('Connected to move_base.')
        rospy.loginfo('Starting a tf listener.')
        self.tf = TransformListener()
        self.listener = tf.TransformListener()
        self.distance_tolerance = rospy.get_param('waypoint_distance_tolerance', 0.0)

        self.waypoint_pub = rospy.Publisher('/waypoints', Path, queue_size=1)

        # print("Setting up dynamic speed server")
        # self.update_client = dynamic_reconfigure.client.Client('pure_pursuit')
        # # self.update_client.wait_for_server()
        # self.update_client = dynamic_reconfigure.client.Client("move_base/TrajectoryPlannerROS", timeout=4, config_callback=None)
        # print("Found it")


    def distance_between_waypts(self, p1, p2):
        dist = np.sqrt((p1.pose.pose.position.x - p2.pose.pose.position.x)**2 + (p1.pose.pose.position.y - p2.pose.pose.position.y)**2)

        return dist

    def execute(self, userdata):
        global megapoints, mission_report

        # Execute waypoints each in sequence
        # prev_waypoint = waypoints[0]

        for megatarget, aux_data in zip(megapoints, auxilary_data):
            tic = time.perf_counter()

            megapoint, megaroute_name = megatarget
            px, py              = goal_dict[megaroute_name]
            path_to_broadcast   = route_dict[megaroute_name]
            # TODO verify this is good for first and last paths

            # Break if preempted
            if not megapoints:
                rospy.loginfo('The waypoint queue has been reset.')
                break

            # self.update_client.update_configuration({"max_vel_x": aux_data[AUX_VELOCITY]})
            # r.sleep()

            # Otherwise publish next waypoint as goal
            # goal = MoveBaseGoal()
            # goal.target_pose.header.frame_id = self.frame_id
            # goal.target_pose.pose.position = waypoint.pose.pose.position
            # goal.target_pose.pose.orientation = waypoint.pose.pose.orientation
            rospy.loginfo('Executing move_base goal to position (x,y) with velocity: %s, %s, %s' %
                          (px, py, -1))
            
            if False or "OLD SCHOOL JUST GOAL MODE":
                # rospy.loginfo("To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")
                self.client.send_goal(goal)
            else: #### NEW MORE ELABORATE METHOD
                self.waypoint_pub.publish(path_to_broadcast)


            # waypoint_gap_dist = self.distance_between_waypts(prev_megapoint, megapoint)

            if not self.distance_tolerance > 0.0:
                self.client.wait_for_result()
                if self.duration > 0:
                    rospy.loginfo("Waiting for %f sec..." % self.duration)
    
                time.sleep(self.duration)

            else:
                # This is the loop which exist when the robot is near a certain GOAL point.
                distance = 10
                while (distance > self.distance_tolerance):
                    now = rospy.Time.now()
                    self.listener.waitForTransform(self.odom_frame_id, self.base_frame_id, now, rospy.Duration(4))
                    trans, rot = self.listener.lookupTransform(self.odom_frame_id, self.base_frame_id, now)
                    distance = math.sqrt(
                        pow(megapoint.pose.pose.position.x - trans[0], 2) + pow(megapoint.pose.pose.position.y - trans[1],
                                                                               2))
                    print("Robot "  + str(distance) + " from goal.")

                    report = [trans[0], trans[1], time_elapsed]
                    mission_report.append(report)


            mission_report.append("REACHED " + str(megapoint))
            toc = time.perf_counter()
            time_elapsed = toc - tic
            time_elapsed = str(time_elapsed)
            print(f"Leg " + str(aux_data[AUX_WAYPOINT_INDEX]) + " took " + time_elapsed + " seconds")

            # prev_waypoint = megapoint

        return 'success'


def convert_PoseWithCovArray_to_PoseArray(waypoints):
    """Used to publish waypoints as pose array so that you can see them in rviz, etc."""
    poses = PoseArray()
    poses.header.frame_id = rospy.get_param('~goal_frame_id', 'map')
    poses.poses = [pose.pose.pose for pose in waypoints]
    return poses


class GetRoute(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'killed'], input_keys=['waypoints'], output_keys=['waypoints'])
        # Parameters
        self.goal_frame_id = rospy.get_param('~goal_frame_id', 'map')

        # Subscribe to pose message to get new waypoints
        # self.addpose_topic = rospy.get_param('~addpose_topic', '/initialpose') // removed since not adding waypoints
        # Create publisher to publish waypoints as pose array so that you can see them in rviz, etc.
        self.posearray_topic = rospy.get_param('~posearray_topic', '/waypoints')
        self.poseArray_publisher = rospy.Publisher(self.posearray_topic, PoseArray, queue_size=1)

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
        self.poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))

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
                print(route_key, route_dict.keys())
                waypoints_info  = route_dict[route_key]
                start           = start_dict[route_key]
                goal            = goal_dict[route_key]

                if first_there == False:
                    megapoints.append((start, route_key))
                    first_there = True

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
        while not self.path_ready and not self.start_journey_bool:
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
        global mission_report

        rospy.loginfo('###############################')
        rospy.loginfo('##### REACHED FINISH GATE #####')
        rospy.loginfo('###############################')

        now_id = datetime.now().strftime("%Y_%m_%d-%I_%M_%S_%p")

        output_folder_default = os.path.join(rospkg.RosPack().get_path('fetch_study'), 'saved_path/reports')
        output_folder = rospy.get_param('~output_folder', output_folder_default)
        output_file_path_report = os.path.join(output_folder, now_id + "-mission_report.csv")
        with open(output_file_path_report, 'w') as file:
            file.write("X, Y, time_to_reach\n")
            for report in mission_report:
                if ',' in report:
                    file.write(str(report[0]) + ',' + str(report[1]) + ',' + str(report[2]) + '\n')
                else:
                    file.write(report + "\n")
            
            rospy.loginfo('Mission report filed to ' + output_file_path_report)

        return 'success'


def main():
    rospy.init_node('follow_route')

    sm = StateMachine(outcomes=['success'])
    with sm:
        StateMachine.add('GET_PATH', GetRoute(),
                         transitions={'success': 'FOLLOW_PATH', 'killed': 'success'},
                         remapping={'waypoints': 'waypoints'})
        StateMachine.add('FOLLOW_PATH', FollowRoute(),
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