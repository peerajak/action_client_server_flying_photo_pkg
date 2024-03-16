#include <actionlib/client/simple_action_client.h>
//#include <ardrone_as/ArdroneAction.h> // Note: "Action" is appended
//#include <actionlib_tutorials/FibonacciAction.h>
#include <action_client_server_flying_photo_pkg/FlyingdroneAction.h>
#include <ros/ros.h>

int nImage = 0;

void doneCb(
    const actionlib::SimpleClientGoalState &state,
    const action_client_server_flying_photo_pkg::FlyingdroneResultConstPtr
        &result) {
  ROS_INFO("[State Result]: %s", state.toString().c_str());
  ROS_INFO("The Action has been completed");
  ROS_INFO("Landing...");

  ros::shutdown();
}

// Called once when the goal becomes active
void activeCb() { ROS_INFO("Goal just went active"); }

void feedbackCb(
    const action_client_server_flying_photo_pkg::FlyingdroneFeedbackConstPtr
        &feedback) {
  ROS_INFO("[Feedback] image n.%d received", nImage);
  ++nImage;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "drone_action_client");
  ros::NodeHandle nh;
  actionlib::SimpleActionClient<
      action_client_server_flying_photo_pkg::FlyingdroneAction>
      client("flyingdrone", true);
  client.waitForServer();
  action_client_server_flying_photo_pkg::FlyingdroneGoal goal;
  goal.order = 100;

  client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

  // client.waitForResult();

  ros::Rate loop_rate(2);

  actionlib::SimpleClientGoalState state_result = client.getState();
  ROS_INFO("[State Result]: %s", state_result.toString().c_str());

  while (state_result == actionlib::SimpleClientGoalState::ACTIVE ||
         state_result == actionlib::SimpleClientGoalState::PENDING) {
    ROS_INFO("Flying and taking pictures....");
    ros::spinOnce();
    loop_rate.sleep();
    // ROS_INFO("Canceling goal...");
    // client.cancelGoal();
    state_result = client.getState();
    ROS_INFO("[State Result]: %s", state_result.toString().c_str());
  }

  return 0;
}
// rostopic pub  /drone/land std_msgs/Empty "{}"