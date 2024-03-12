#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <fetch_study/StatusAction.h>

class StatusAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<fetch_study::StatusAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result

  fetch_study::StatusActionFeedback feedback_;
  fetch_study::StatusActionResult result_;

public:

  StatusAction(std::string name, float wait_type) :
    as_(nh_, name, boost::bind(&StatusAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  StatusAction(std::string name) :
    as_(nh_, name, boost::bind(&StatusAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~StatusAction(void)
  {
  }

  void executeCB(const fetch_study::StatusActionGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // // push_back the seeds for the fibonacci sequence
    // feedback_.sequence.clear();
    // feedback_.sequence.push_back(0);
    // feedback_.sequence.push_back(1);

    // publish info to the console for the user
    // ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

    ROS_INFO("%s: Executing" action_name_.c_str());

    // start executing the action
    for(int i=1; i<=goal->order; i++)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
      // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }

    if(success)
    {
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "status_action");

  StatusAction status_action("status_action");
  ros::spin();

  return 0;
}
