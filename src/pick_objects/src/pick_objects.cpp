#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "pick_objects/go_to_xy.h"

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

MoveBaseClient* ac_ptr;

bool process_go_to_xy_callback(
    pick_objects::go_to_xy::Request &req,
    pick_objects::go_to_xy::Response &res
  ) {
  ROS_INFO("Going to -> %f %f", req.x, req.y);
  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = req.x;
  goal.target_pose.pose.position.y = req.y;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal for pickup");
  ac_ptr->sendGoal(goal);

  // Wait an infinite time for the results
  ac_ptr->waitForResult();

  // Check if the robot reached its goal
  if(ac_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, location reached");
  else
    ROS_INFO("The base failed to move for some reason");
  
  res.status = true;
  return true;
}

int main(int argc, char** argv){
  bool isTest = false;
  for(int i = 0; i < argc; i++) {
    ROS_INFO("%s", argv[i]);
    if(strcmp(argv[i], "--test") == 0)
      isTest = true;
  }

  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "simple_navigation_goals");

  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("go_to_xy", process_go_to_xy_callback);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  ac_ptr = &ac;

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  if(isTest) {
    ROS_INFO("isTest is true");
    move_base_msgs::MoveBaseGoal goal;

    // set up the frame parameters
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach
    goal.target_pose.pose.position.x = 0;
    goal.target_pose.pose.position.y = 6;
    goal.target_pose.pose.orientation.w = 1.0;

    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending goal for pickup");
    ac_ptr->sendGoal(goal);

    // Wait an infinite time for the results
    ac_ptr->waitForResult();

    // Check if the robot reached its goal
    if(ac_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Hooray, location reached");
    else
      ROS_INFO("The base failed to move for some reason");

    // second position
    sleep(5);

    // Define a position and orientation for the robot to reach
    goal.target_pose.pose.position.x = -9;
    goal.target_pose.pose.position.y = -10;
    goal.target_pose.pose.orientation.w = 1.0;

    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending goal for dropoff");
    ac_ptr->sendGoal(goal);

    // Wait an infinite time for the results
    ac_ptr->waitForResult();

    // Check if the robot reached its goal
    if(ac_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Hooray, dropoff reached");
    else
      ROS_INFO("The base failed to move for some reason");
  }
  ros::spin();
  return 0;
}