#!/usr/bin/env python3
# coding:utf-8

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import os
import rospkg

AT_GOAL_DISTANCE = .05

def import_waypoints():
    output_folder_default = os.path.join(rospkg.RosPack().get_path('fetch_study'), 'waypoints/')
    output_folder = rospy.get_param('~output_folder', output_folder_default)

    path_dict = {}

    for path_name in ['waypoints']:
        waypoints_path = output_folder_default + path_name + ".csv"
    
        waypoints_info      = load_waypoints(waypoints_path_location)
        
        # KEY = (name, first, last)
        key             = (path_name, waypoints_info[0], waypoints_info[-1])
        path_dict[key]  = waypoints_info

    return path_dict


def load_waypoints(waypoints_path):
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

    return waypoints_info

def broadcast_waypoints_manager(waypoints_dict):
    # Get a path that starts where we are
    # Move along that path

    self.listener.waitForTransform(self.odom_frame_id, self.base_frame_id, now, rospy.Duration(4))
    trans, rot = self.listener.lookupTransform(self.odom_frame_id, self.base_frame_id, now)
    
    target_key = None
    for key in waypoints_dict.keys():
        pname, pstart, pgoal = key

        distance = math.sqrt(
            pow(pstart.pose.pose.position.x - trans[0], 2) 
                + pow(pstart.pose.pose.position.y - trans[1], 2))

        print("Robot "  + str(distance) + " from path start.")
        # distance_left = distance

        if distance < .1:
            target_key = key


    broadcast_single_path(waypoints_dict[key])






def broadcast_single_path(path_to_broadcast):
    try:
        while not rospy.is_shutdown() and not at_goal:
            now = rospy.Time.now()
            self.listener.waitForTransform(self.odom_frame_id, self.base_frame_id, now, rospy.Duration(4))
            trans, rot = self.listener.lookupTransform(self.odom_frame_id, self.base_frame_id, now)
            distance = math.sqrt(
                pow(final_destination.pose.pose.position.x - trans[0], 2) 
                    + pow(final_destination.pose.pose.position.y - trans[1], 2))

            print("Robot "  + str(distance) + " from goal.")
            

            if distance < AT_GOAL_DISTANCE:
                at_goal = True
                print("Reached goal!")


            # Then continue on the path
            waypoint_pub.publish(waypoints_info)
            rate.sleep()


    except rospy.ROSInterruptException:
        rospy.logerr('Get KeyBoardInterrupt... Shutdown')

    print("Successsfully completed path, moving onto the next")


# def main():
#     ### Set up the node
#     rospy.init_node('waypoints_loader')
#     waypoint_pub = rospy.Publisher('/waypoints', Path, queue_size=1)
#     rate = rospy.Rate(1)

#     # Load the path
#     waypoints_path = rospy.get_param("load_waypoints/waypoints_path")
#     waypoints_info = load_waypoints(waypoints_path)

#     distance_left = np.inf

#     final_destination = waypoints_info[-1]

#     while not rospy.is_shutdown() and distance_left > 0:
#         now = rospy.Time.now()
#         self.listener.waitForTransform(self.odom_frame_id, self.base_frame_id, now, rospy.Duration(4))
#         trans, rot = self.listener.lookupTransform(self.odom_frame_id, self.base_frame_id, now)
#         distance = math.sqrt(
#             pow(final_destination.pose.pose.position.x - trans[0], 2) 
#                 + pow(final_destination.pose.pose.position.y - trans[1], 2))

#         print("Robot "  + str(distance) + " from goal.")
#         distance_left = distance

#         # Keep publishing and spinning
#         waypoint_pub.publish(waypoints_info)
#         rate.sleep()


if __name__ == "__main__":
    try:
        #### Set up the node
        rospy.init_node('waypoints_loader')
        waypoint_pub = rospy.Publisher('/waypoints', Path, queue_size=1)
        rate = rospy.Rate(1)

        waypoints_dict = import_waypoints()

        broadcast_waypoints_manager(waypoints_dict)

    except rospy.ROSInterruptException:
        rospy.logerr('Get KeyBoardInterrupt... Shutdown')
