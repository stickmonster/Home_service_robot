#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 1.0;
 goal.target_pose.pose.position.y =  0.0;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
   { ROS_INFO("Huzzah, the base moved 1 metre forward");

    ros::Duration(5.0).sleep();

    while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

    move_base_msgs::MoveBaseGoal second_goal;

    // set up the frame parameters
    second_goal.target_pose.header.frame_id = "map";
    second_goal.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach
    second_goal.target_pose.pose.position.x = -3.0;
    second_goal.target_pose.pose.position.y = 2.0;
    second_goal.target_pose.pose.orientation.w = 1.0;

    // Send the goal position and orientation for the robot to reach
   ROS_INFO("Sending second goal");
   ac.sendGoal(second_goal);

   // Wait an infinite time for the results
   ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Huzzah, the turtlebot has arrived at the second goal");

    else


      ROS_INFO("Oh no, the turtlebot has not reached its second target");
}
  return 0;
}