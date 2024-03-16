#include <actionlib/server/simple_action_server.h>
//#include <actionlib_tutorials/FibonacciAction.h>
#include <action_client_server_flying_photo_pkg/FlyingdroneAction.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>

class FlyingDroneAction {
protected:
  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange
  // error occurs.
  actionlib::SimpleActionServer<
      action_client_server_flying_photo_pkg::FlyingdroneAction>
      as_;
  std::string action_name_;
  // create messages that are used to publish feedback and result
  action_client_server_flying_photo_pkg::FlyingdroneFeedback feedback_;
  action_client_server_flying_photo_pkg::FlyingdroneResult result_;
  ros::Publisher pub_takeoff;
  ros::Publisher pub_land;
  ros::Publisher pub_cmdvel;
  std_msgs::Empty emptyMsgTakeoff;
  std_msgs::Empty emptyMsgLanding;
  geometry_msgs::Twist ling;

  void setCircularMovement() {
    ling.linear.x = 2;
    ling.linear.y = 0;
    ling.linear.z = 0;
    ling.angular.x = 0;
    ling.angular.y = 0;
    ling.angular.z = 2;
  }

public:
  FlyingDroneAction(std::string name)
      : as_(nh_, name, boost::bind(&FlyingDroneAction::executeCB, this, _1),
            false),
        action_name_(name) {
    pub_takeoff = nh_.advertise<std_msgs::Empty>("/drone/takeoff", 1000);
    pub_cmdvel = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    pub_land = nh_.advertise<std_msgs::Empty>("/drone/land", 1000);
    as_.start();
  }

  ~FlyingDroneAction(void) {}

  void
  executeCB(const action_client_server_flying_photo_pkg::FlyingdroneGoalConstPtr
                &goal) {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // TODO Do what ever the goal is
    // TODO calculate FEEDBACK here
    //  push_back the seeds for the flyingdrone sequence
    feedback_.sequence.clear();

    // publish info to the console for the user
    ROS_INFO("%s: Executing, creating flyingdrone sequence of order %i  ",
             action_name_.c_str(), goal->order);

    // start executing the action
    for (int i = 1; i <= goal->order; i++) {
      // TODO Preempt Here
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok()) {
        ROS_INFO("%s: Preempted Canceling goal...", action_name_.c_str());
        // set the action state to preempted
        pub_land.publish(emptyMsgLanding);
        as_.setPreempted();
        success = false;
        break;
      }
      ROS_INFO("Flying and taking pictures....");
      pub_takeoff.publish(emptyMsgTakeoff);
      setCircularMovement();
      pub_cmdvel.publish(ling);
      feedback_.sequence.push_back(i);
      // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for
      // demonstration purposes
      r.sleep();
    }

    if (success) {
      // TODO RESULT here
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      pub_land.publish(emptyMsgLanding);
      as_.setSucceeded(result_);
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "flyingdrone");

  FlyingDroneAction flyingdrone("flyingdrone");
  ros::spin();

  return 0;
}
/*
To Start the server:
- roslaunch action_client_server_flying_photo_pkg
action_server_flying_photo.launch
- rostopic pub /flyingdrone/goal actionlib_tutorials/FibonacciActionGoal
"header: seq: 0 stamp: secs: 0 nsecs: 0 frame_id: '' goal_id: stamp: secs: 0
    nsecs: 0
  id: ''
goal:
  order: 300"
To preempt:
rostopic pub /flyingdrone/cancel actionlib_msgs/GoalID "stamp:
  secs: 0
  nsecs: 0
id: ''"
To listen result: rostopic echo /flyingdrone/result
To Listen Feedback: rostopic echo /flyingdrone/feedback
To Listen to Cancel: rostopic echo /flyingdrone/cancel
*/