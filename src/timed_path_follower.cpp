/*******************************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015 Marina Kollmitz
 *  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Marina Kollmitz
 ******************************************************************************/

#include <timed_path_follower.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(path_executer::PathFollower, nav_core::BaseLocalPlanner)

namespace path_executer
{

  //check for zero with rounding effects
#define ROUNDED_ZERO 1e-6

  PathFollower::PathFollower() :
    initialized_(false),
    costmap_ros_(NULL)
  {
  }


  PathFollower::PathFollower(std::string name, tf2_ros::Buffer* tf,
                             costmap_2d::Costmap2DROS* costmap_ros)
      : costmap_ros_(NULL), tfl_(NULL), initialized_(false)
  {
      initialize(name, tf, costmap_ros);
  }

  PathFollower::~PathFollower() {}


  void PathFollower::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
  {
    if(!initialized_)
    {
      //create ros node handles
      ros::NodeHandle nh;
      ros::NodeHandle private_nh("~/" + name);

      //initialize ros publisher and service client
      current_waypoint_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("waypoint", 1);
      replan_client_ = nh.serviceClient<std_srvs::Empty>("move_base/replan");

      //collect transform listener and the ros costmap from the nav stack
      tfl_ = tf;
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();
      
      //initialize the dynamic reconfigure server and register the callback
      dsrv_ = new dynamic_reconfigure::Server<path_executer::PathExecuterConfig>
          (private_nh);
      dynamic_reconfigure::Server<path_executer::PathExecuterConfig>::CallbackType cb;

      cb = boost::bind(&PathFollower::reconfigureCallback, this, _1, _2);
      dsrv_->setCallback(cb);

      //get the planner preferences from ros params
      private_nh.param("allow_backwards", allow_backwards_, false);
      private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.1);
      private_nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.2);

      initialized_ = true;
    }
  }

  auto createQuaternionMsgFromYaw(double yaw)
  {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
  }

  void PathFollower::reconfigureCallback
      (path_executer::PathExecuterConfig &config, uint32_t level)
  {
    k_rho_ = config.k_rho;
    k_alpha_ = config.k_alpha;
    k_beta_ = config.k_beta;
    max_vel_x_ = config.max_vel_x;
    max_vel_phi_ = config.max_vel_phi;

    ROS_INFO("changed controller values: \n \n gains: \n k_rho = %f \n k_alpha %f \n "
             "k_beta %f \n \n max_velocity: \n max_vel_x %f \n mav_vel_phi %f",
             k_rho_, k_alpha_, k_beta_, max_vel_x_, max_vel_phi_);
  }

  bool PathFollower::findWaypointAtTime(ros::Time time,
                                        geometry_msgs::PoseStamped& waypoint,
                                        geometry_msgs::Twist& waypoint_vel)
  {
    //make sure planner has been initialized
    if(!initialized_)
    {
      ROS_ERROR("path executer: planner has not been initialized");
      //return empty container
      return false;
    }

    //make sure the global plan has been set and the waypoint velocities have
    //been calculated
    if(global_plan_waypoints_.empty() || global_plan_velocities.empty())
    {
      ROS_ERROR_STREAM("path executer: could not find waypoint at time " << time
                       << ": global plan (waypoints or velocities) is empty.");
    }

    //make sure requested time is represented by the global plan
    if(time > global_plan_waypoints_.back().header.stamp ||
       time < global_plan_waypoints_.front().header.stamp)
    {
      ROS_ERROR("requested waypoint is not within time range of path");
      return false;
    }

    int i=0;

    //check array for the last waypoint before the requested time
    while(i < global_plan_waypoints_.size() && global_plan_waypoints_.at(i).header.stamp <= time)
    {
      waypoint = global_plan_waypoints_.at(i);
      waypoint_vel = global_plan_velocities.at(i);
      i++;
    }

    // calculate the time difference between the waypoint of the path
    // and the requested time
    ros::Duration diff = time - waypoint.header.stamp;



    //calculate the waypoint coordinates at requested time from the motion equation
    double x = waypoint.pose.position.x;
    double y = waypoint.pose.position.y;
    double theta = tf2::getYaw(waypoint.pose.orientation);

    //calculate coordinates: motion is a circle
    if(waypoint_vel.angular.z != 0)
    {
      double arc_radius = waypoint_vel.linear.x / waypoint_vel.angular.z;

      x = x + arc_radius * (- sin(theta) +
                            sin(theta + waypoint_vel.angular.z * diff.toSec()));
      y = y + arc_radius * (cos(theta) -
                            cos(theta + waypoint_vel.angular.z * diff.toSec()));
      theta = theta + waypoint_vel.angular.z * diff.toSec();
    }

    //calculate coordinates: linear motion
    else
    {
      x = x + waypoint_vel.linear.x * cos(theta) * diff.toSec();
      y = y + waypoint_vel.linear.x * sin(theta) * diff.toSec();
    }

    waypoint.header.stamp = time;
    waypoint.pose.position.x = x;
    waypoint.pose.position.y = y;
    // https://answers.ros.org/question/364561/tfcreatequaternionfromyaw-equivalent-in-ros2/
    // waypoint.pose.orientation = createQuaternionMsgFromYaw(theta);

    //publish the waypoint for visualization
    ROS_INFO("Publishing a waypoint!");
    geometry_msgs::PoseStamped waypoint_test;
    current_waypoint_pub_.publish(waypoint_test);
    ROS_INFO("Published the wpt");

    return true;
  }

  bool PathFollower::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
  {
    ROS_INFO("Following velocity commands");

    //make sure planner had been initialized
    if(!initialized_)
    {
      ROS_ERROR("path executer: planner has not been initialized");
      //return false
      return false;
    }

    geometry_msgs::Twist zero_vel;

    //if a replanning request was send (because the pose of the robot deviated
    //too much from the desired path pose), set the command velocity to zero and
    //wait for the new plan
    if(replanning_requested_)
    {
      ROS_INFO("replanning requested");
      cmd_vel = zero_vel;
      return true;
    }

    //get the current robot pose in the costmap
    // tf2::Stamped<tf2::Pose> robot_pose;
    geometry_msgs::PoseStamped robot_pose_stamped;
    if(!costmap_ros_->getRobotPose(robot_pose_stamped))
    {
      cmd_vel = zero_vel;
      ROS_ERROR("path_executer: cannot get robot pose");
      return false;
    }

    tf2::Transform robot_pose;
    tf2::fromMsg(robot_pose_stamped.pose, robot_pose);

    ROS_INFO("Got the pose from the robot pose");
    ROS_INFO("%f, %f", robot_pose_stamped.pose.position.x, robot_pose_stamped.pose.position.y);

    //if the robot pose and the path (and goal) are represented in different
    //coordinate systems, transform the robot pose
    if (robot_pose_stamped.header.frame_id.compare(goal_.header.frame_id) != 0)
    {
      ROS_WARN_ONCE("path_executer: the specified fixed frame for the costmap (%s) "
                    "does not math the fixed frame of the path (%s). Therefore, I "
                    "have to transform the robot pose in each control loop. "
                    "This could cause a decreased control frequency. Consider "
                    "changing the (local) costmap's fixed frame",
                    robot_pose_stamped.header.frame_id.c_str(), goal_.header.frame_id.c_str());

      try
      {
        ROS_INFO("Trying to look up a transform from buffer");
        // ROS_INFO("Values are: %f, %f, %s", goal_.header.frame_id, robot_pose_stamped.header.frame_id, robot_pose_stamped.header.stamp.c_str());

        // ros::Time now = ros::Time::now();

        // Used to be waitForTransform, transformPose 
        // Look up the mapping between the base_link and the map
        // This can actually take quite a bit
        // const std::string &target_frame, const std::string &source_frame, 
          // const ros::Time &time, const ros::Duration timeout)
        geometry_msgs::PoseStamped robot_pose_stamped = tfl_->lookupTransform(goal_.header.frame_id, robot_pose_stamped.header.frame_id,
                              robot_pose_stamped.header.stamp, ros::Duration(5.0));

       
        // OG signature
        // const std::string &target_frame, const geometry_msgs::PoseStamped &stamped_in, geometry_msgs::PoseStamped &stamped_out
        // transformPose(goal_.header.frame_id, robot_pose, robot_pose);
        // NOW
        // const A &in, B &out, const std::string &target_frame, ros::Duration timeout=ros::Duration(0.0)
        // ROS_INFO("Now I want to apply it");
        // tfl_->transform(robot_pose_stamped, robot_pose_stamped, goal_.header.frame_id);
        // ROS_INFO("Successfully transformed");
      }

      catch(tf2::TransformException ex)
      {
        ROS_ERROR("path_executer: could not transform robot pose in goal frame, "
                  "tf anwered: %s", ex.what());
        cmd_vel = zero_vel;
        return true;
      }
    }

    geometry_msgs::PoseStamped waypoint;
    geometry_msgs::Twist waypoint_vel;

    tf2::Transform new_robot_pose;
    tf2::fromMsg(robot_pose_stamped, new_robot_pose);

    //check if we are already within the goal tolerance
    tf2::Transform goal;
    tf2::fromMsg(goal_.pose, goal);

    //calculate the transformation between the robot and the goal pose
    tf2::Transform robot_in_goal = goal.inverse() * new_robot_pose;

    //calculate the euclidian distance between the current robot pose and the goal
    double goal_distance =
        hypot(robot_in_goal.getOrigin().getX(), robot_in_goal.getOrigin().getY());

    ROS_INFO("Goal distance is %f", goal_distance);

    //calculate the angular distance between the current robot pose and the goal
    tf2::Quaternion quat;
    quat = new_robot_pose.getRotation();
    double angular_goal_distance =
        angles::shortest_angular_distance(tf2::getYaw(goal_.pose.orientation),
                                          tf2::getYaw(quat));

    //check if robot is within the goal distance
    if(goal_distance < xy_goal_tolerance_ && fabs(angular_goal_distance) < yaw_goal_tolerance_)
    {
      goal_reached_ = true;
      cmd_vel = zero_vel;
      return true;
    }

    //calculate the error between the current position and the desired position.
    //The desired position is the robot the robot should be on right now,
    //according to the given plan
    ros::Time now = ros::Time::now();
    ROS_INFO("Got my current time");

    if(findWaypointAtTime(now, waypoint, waypoint_vel))
    {
      ROS_INFO("Found a waypoint");
      tf2::Transform waypnt;
      tf2::fromMsg(waypoint.pose, waypnt);

      //calculate transformation between the robot position and the desired position
      tf2::Transform robot_in_wpnt = waypnt.inverse() * robot_pose;

      double delta_x = robot_in_wpnt.getOrigin().getX();
      double delta_y = robot_in_wpnt.getOrigin().getY();
      double phi = tf2::getYaw(robot_in_wpnt.getRotation());

      //It is not possible to set the velocities v_x, v_y and omega directly to
      //eliminate the pose error between the current position of the robot and
      //the goal because of the differential constraints.
      //Therefore, we use rho, alpha and beta as control values. The
      //controller design follows the feedback motion controller for differential
      //drive robots suggested in the book "Introduction to Autonomous Mobile Robots"
      //by Siegwart et al. (MIT Press 2011).
      double rho = hypot(delta_x, delta_y);
      double alpha = atan2(-1*delta_y, -1*delta_x) - phi;

      //if delta_x and delta_y are both very small, the angle is not properly
      //defined. hence, only take orientation difference phi into account.
      //also, if the desired forward vel is zero, only turn in place
      if(fabs(rho) < 0.01 || waypoint_vel.linear.x == 0)
      {
        //only turn in place
        ROS_DEBUG("turn in place");
        alpha = 0;
        rho = 0;
      }

      //if the distance between the robot and the desired waypoint is too big,
      //we should trigger replanning
      if(rho > 0.25)
      {
        ROS_INFO("distance to path is too big, triggering replanning");
        std_srvs::Empty srv;
        //call the planner with the replanning request
        replan_client_.call(srv);
        //set the velocity to zero and wait for the new plan
        cmd_vel = zero_vel;
        replanning_requested_ = true;
        return true;
      }

      double beta = -1* alpha - phi;

      if(allow_backwards_)
      {
        //only change the driving direction if driving backwards is beneficial
        //and allow_backwards flag is set to true
        if(fabs(alpha) > M_PI_2)
        {
          rho = -1* rho;
          if(alpha > 0)
            alpha = -M_PI + alpha;

          else
            alpha = M_PI + alpha;
        }
      }

      //controll values
      ROS_DEBUG("delta_x: %f, delta_y: %f, phi: %f", delta_x, delta_y, phi);
      ROS_DEBUG("rho: %f, alpha: %f, beta: %f", rho, alpha, beta);

      //calculate control velocities with the control values and the
      //controller gains. Always make sure that the velocity is within the bounds
      cmd_vel.linear.x = k_rho_ * rho;
      cmd_vel.angular.z = k_alpha_ * alpha + k_beta_ * beta;

      if(cmd_vel.linear.x > max_vel_x_)
        cmd_vel.linear.x = max_vel_x_;

      else if(cmd_vel.linear.x < -max_vel_x_)
        cmd_vel.linear.x = -max_vel_x_;

      if(cmd_vel.angular.z > max_vel_phi_)
        cmd_vel.angular.z = max_vel_phi_;

      else if(cmd_vel.angular.z < -max_vel_phi_)
        cmd_vel.angular.z = -max_vel_phi_;

      return true;
    }

    else
    {
      ROS_ERROR_STREAM("path_follower: could not find waypoint at time " << now);
      cmd_vel = zero_vel;
      return false;
    }
  }

  std::vector<geometry_msgs::Twist>
      PathFollower::computeWaypointVelocities(const std::vector<geometry_msgs::PoseStamped> &timed_plan)
  {
    std::vector<geometry_msgs::Twist> waypoint_velocities;

    geometry_msgs::Twist one_vel;
    geometry_msgs::Twist zero_vel;

    //if the plan only has zero or one entry, we cannot calculate waypoint velocities
    if(timed_plan.size() < 2)
    {
      ROS_INFO("path_executer: timed plan is empty");
      waypoint_velocities.push_back(zero_vel);
      return waypoint_velocities;
    }

    ROS_INFO("path_executer: timed plan of length");
    ROS_INFO("%lu", timed_plan.size());

    //calculate necessary velocity commands for feed forward control
    for(int i=0; i<timed_plan.size() - 1; i++)
    {
      //calculate velocity command between two successive poses (first, second)
      geometry_msgs::PoseStamped first;
      tf2::fromMsg(timed_plan.at(i), first);

      geometry_msgs::PoseStamped second;
      tf2::fromMsg(timed_plan.at(i+1), second);

      tf2::Transform first_p;
      tf2::Transform second_p;

      tf2::fromMsg(first.pose, first_p);
      tf2::fromMsg(second.pose, second_p);

      ROS_INFO("converted happily");

      //time difference between the poses
      double time_diff = (second.header.stamp - first.header.stamp).toSec();
      ROS_INFO("time_diff: %f", time_diff);

      if(fabs(time_diff) < ROUNDED_ZERO)
      {
        //if the time difference between both poses is very small, do not calculate
        //the velocity but skip one pose
        ROS_WARN("path_executer: noticed very small time difference between two poses");
        waypoint_velocities.push_back(zero_vel);
        continue;
      }


      //calculate the transformation between both poses (difference between both)
      tf2::Transform diff = first_p.inverse() * second_p;
      double delta_x = diff.getOrigin().getX();
      double delta_y = diff.getOrigin().getY();
      double delta_phi = tf2::getYaw(diff.getRotation());

      //recover the velocity between both poses from the equation of motion,
      //depending whether the motion between the poses is an arc, a straight line
      //or a turn in place. Assume constant velocity motion between both poses
      if(fabs(delta_phi) > ROUNDED_ZERO)
      {
        //there is an angle difference between both poses
        if(fabs(delta_x) > ROUNDED_ZERO && fabs(delta_y) > ROUNDED_ZERO)
        {
          //path increment is an arc.
          double rad = (pow(delta_x, 2) + pow(delta_y, 2)) / (2 * delta_y);
          double alpha = atan2(delta_x, rad - delta_y);

          if(rad < 0)
            alpha = alpha - M_PI;

          if(alpha > M_PI)
            alpha -= 2 * M_PI;

          else if(alpha < - M_PI)
            alpha += 2 * M_PI;

          double distance = rad * alpha;

          one_vel.linear.x = distance / time_diff;
          one_vel.angular.z = alpha / time_diff;
        }
        else
        {
          //turn in place.
          one_vel.linear.x = 0;
          one_vel.angular.z = delta_phi / time_diff;
        }
      }
      else
      {
        //path increment is line.
        one_vel.angular.z = 0;

        //check if the path is consistent with differential drive constraints
        if(fabs(delta_y) > ROUNDED_ZERO)
        {
          ROS_ERROR("cannot calculate velocity for diff-drive robot, delta y is %f", delta_y);
          waypoint_velocities.clear();
          waypoint_velocities.push_back(zero_vel);
          return waypoint_velocities;
        }
        one_vel.linear.x = delta_x / time_diff;
      }
      ROS_INFO("adding calculated waypoint velocity");
      //add the calculated waypoint velocity
      waypoint_velocities.push_back(one_vel);
    }

    //add one zero velocity element that corresponds to the last pose
    geometry_msgs::Twist empty;
    waypoint_velocities.push_back(empty);

    return waypoint_velocities;
  }

  bool PathFollower::setPlan(const std::vector<geometry_msgs::PoseStamped>&
                             global_plan)
  {
    //make sure planner had been initialized
    if(!initialized_)
    {
      ROS_ERROR("path executer: planner has not been initialized");
      return false;
    }

    //reset the planning flags
    goal_reached_ = false;
    replanning_requested_ = false;

    //set the plan and the goal and recover the waypoint velocities from the
    //timed plan
    global_plan_waypoints_ = global_plan;
    global_plan_velocities = computeWaypointVelocities(global_plan);
    ROS_INFO("Computed all waypoint velocities!");
    goal_ = global_plan.back();
    ROS_INFO("I set the plan to...");

    return true;
  }

} //namespace path_executer
