#include <ros/ros.h>
#include <iostream>

//Following mimics what is found in DockDriveActionClient.py
#include <actionlib/client/simple_action_client.h>
#include <kobuki_msgs/AutoDockingAction.h>
#include <kobuki_msgs/AutoDockingGoal.h> //May be uneeded

typedef actionlib::SimpleActionClient <kobuki_msgs::AutoDockingAction> dockingClient;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "autoDockingTest"); //initialises node
  ros::NodeHandle n;

  dockingClient client("dock_drive_action", true); // true -> don't need ros::spin()

  client.waitForServer(); //Wait for feedback from the Action server

  std::cout << "Server feedback recieved" << std::endl; //check

  kobuki_msgs::AutoDockingGoal goal;
  
  std::cout << "Goal set" << std::endl; //check

  // Fill in goal here <- from tutorial (no idea what it means)

  client.sendGoal(goal);

  std::cout << "Goal sent" << std::endl;//check

  client.waitForResult(); //ros::Duration(5.0) for maximum wait time?

  std::cout << "Result recieved" << std::endl;//check

  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    std::cout << "I reached my goal!" << std::endl;
  }
  else
  {
    std::cout << "Current State: " << client.getState().toString().c_str() << std::endl;
  }

  return 0;
}