#!/usr/bin/env python

# https://github.com/slamcore/follow_waypoints/blob/master/nodes/follow_waypoints

import collections
import time

import actionlib
import rospy
import tf
from actionlib_msgs.msg import GoalID
from dynamic_reconfigure.server import Server as DynServer
from follow_waypoints.cfg import FollowWaypointsConfig
from geometry_msgs.msg import PointStamped, PoseArray, PoseWithCovarianceStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Trigger

# Miscellaneous functions
info = rospy.loginfo
warn = rospy.logwarn


class Commander(object):
    """
    Read goal poses/locations from the designated topics, instruct move_base to follow them.
    """

    def __init__(self):
        super(Commander, self).__init__()
        rospy.init_node("follow_waypoints")

        # Read parameters off the parameter server --------------------------------------------
        self.frame_id = rospy.get_param("goal_frame_id", "map")
        self.odom_frame_id = rospy.get_param("odom_frame_id", "odom")
        self.base_frame_id = rospy.get_param("base_frame_id", "base_footprint")
        self.wait_duration = rospy.get_param("wait_duration", 0.0)
        self.distance_tolerance = rospy.get_param("waypoint_distance_tolerance", 0.0)

        self.waypoints_to_follow_topic = rospy.get_param(
            "waypoints_to_follow_topic", "/initialpose"
        )
        self.waypoints_list_topic = rospy.get_param("waypoints_list_topic", "/waypoints")

        # listen to points or to poses?
        waypoints_are_poses = rospy.get_param("waypoints_are_poses", True)

        # list of waypoints to follow
        self.waypoints = collections.deque()

        # Is the path provided by the user ready to follow?
        self.path_ready = False

        # Dynamic Reconfigure -----------------------------------------------------------------
        self.dyn_config = {}
        self.dyn_server = DynServer(FollowWaypointsConfig, self.dyn_callback)

        # move_base Action client -------------------------------------------------------------
        self.move_base_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        info("Connecting to move_base...")
        self.move_base_client.wait_for_server()
        info("Connected to move_base.")

        # TF Initialisation -------------------------------------------------------------------
        self.tf = tf.TransformListener()
        self.listener = tf.TransformListener()

        # Subscribers & Publishers & Services initialisation ----------------------------------
        self.path_ready_srv = rospy.Service("path_ready", Trigger, self.make_path_ready)
        self.path_reset_srv = rospy.Service("path_reset", Trigger, self.do_path_reset)

        # Publish waypoints as pose array - visualise them in rviz
        self.pose_array_publisher = rospy.Publisher(
            self.waypoints_list_topic, PoseArray, queue_size=1, latch=True
        )

        # Listen to the goal locations
        self.waypoints_topic_type = (
            PoseWithCovarianceStamped if waypoints_are_poses else PointStamped
        )
        self.waypoints_topic_cb = self.pose_cb if waypoints_are_poses else self.point_cb
        self.waypoints_sub = rospy.Subscriber(
            self.waypoints_to_follow_topic, self.waypoints_topic_type, self.waypoints_topic_cb
        )

    def dyn_callback(self, config, _):
        """Dynamic configuration callback method."""
        self.dyn_config.update(config)
        return self.dyn_config

    def make_path_ready(self, _):
        self.path_ready = True
        return (True, "")

    def do_path_reset(self, _):
        warn("Issuing cancel command to move_base")
        pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        pub.publish(GoalID())
        warn("move_base goal cancelled.")

        if self.waypoints:
            self.waypoints = collections.deque()
            self.pub_waypoints_list()

        return (True, "")

    def pub_waypoints_list(self):
        """Helper method to publish the waypoints that should be followed."""
        self.pose_array_publisher.publish(toPoseArray(self.waypoints))

    def point_cb(self, point):
        """Accept a Point message as the next goal to follow.

        Internally converts it to a Pose using the default quaternion.

        .. todo:: Have an option to infer the orientation at the goal based on previous
        location and interconnecting link
        """

        self.waypoints.append(toPoseWithCov(point))
        self.pub_waypoints_list()
        info(
            "Added new waypoint -> (%s, %s) | # Waypoints: %s"
            % (point.point.x, point.point.y, len(self.waypoints))
        )

    def pose_cb(self, pose):
        """Accept a Pose message as the next goal to follow."""

        self.waypoints.append(pose)
        self.pub_waypoints_list()
        info(
            "Added new waypoint -> (%s, %s) | # Waypoints: %s"
            % (pose.pose.pose.position.x, pose.pose.pose.position.y, len(self.waypoints))
        )

    def send_move_base_goal(self, pose):
        """Assemble and send a new goal to move_base"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frame_id
        goal.target_pose.pose.position = pose.pose.pose.position
        goal.target_pose.pose.orientation = pose.pose.pose.orientation
        info(
            "Executing move_base goal -> (%s, %s) ..."
            % (pose.pose.pose.position.x, pose.pose.pose.position.y)
        )
        self.move_base_client.send_goal(goal)

    def run_once(self):
        """Single iteration of the node loop."""
        if not self.path_ready:
            time.sleep(1.0)
            return

        if not self.waypoints:
            warn("No more waypoints to follow.")
            self.path_ready = False
            return

        info("Following path with # %s waypoints..." % len(self.waypoints))

        # we have waypoints, let's follow them!
        while self.waypoints and not rospy.is_shutdown():
            goal = self.waypoints[0]
            if self.dyn_config["patrol_mode"]:
                if len(self.waypoints) == 1:
                    warn(
                        "In patrol mode but wth only one given waypoint, will simply navigate to it..."
                    )
                    self.waypoints.pop()
                else:
                    self.waypoints.rotate(-1)
            else:
                # not in patrol mode - forget about this waypoint
                self.waypoints.popleft()
            self.send_move_base_goal(goal)

            if not self.distance_tolerance > 0.0:
                # just wait until move_base reaches the goal...
                self.move_base_client.wait_for_result()
                info("Waiting for %f seconds before proceeding to the next goal..." % self.wait_duration)
                time.sleep(self.wait_duration)
            else:
                raise NotImplementedError("distance_tolerance not implemented yet.")

                # # TODO - Rewrite
                # # This is the loop which exist when the robot is near a certain GOAL point.
                # distance = 10
                # while distance > self.distance_tolerance:
                #     now = rospy.Time.now()
                #     self.listener.waitForTransform(
                #         self.odom_frame_id, self.base_frame_id, now, rospy.Duration(4.0)
                #     )
                #     trans, rot = self.listener.lookupTransform(
                #         self.odom_frame_id, self.base_frame_id, now
                #     )
                #     distance = math.sqrt(
                #         pow(goal.pose.pose.position.x - trans[0], 2)
                #         + pow(goal.pose.pose.position.y - trans[1], 2)
                #     )

            # current waypoint has now been reached! move on to the next waypoint on the next
            # iteration
            self.pub_waypoints_list()

    def run(self):
        while not rospy.is_shutdown():
            self.run_once()


# helper methods
def toPoseArray(waypoints):
    """Publish waypoints as a pose array so that you can see them in rviz."""
    poses = PoseArray()
    poses.header.frame_id = rospy.get_param("~goal_frame_id", "map")
    poses.poses = [pose.pose.pose for pose in waypoints]
    return poses


def toPoseWithCov(point):
    """Convert a PointStamped message to a PoseWithCovarianceStamped."""
    pose = PoseWithCovarianceStamped()
    pose.header = point.header
    pose.pose.pose.position = point.point
    angle = Quaternion()
    angle.x = 0
    angle.y = 0
    angle.z = 0
    angle.w = 1
    pose.pose.pose.orientation = angle

    return pose


def main():
    commander = Commander()
    commander.run()


if __name__ == "__main__":
    main()