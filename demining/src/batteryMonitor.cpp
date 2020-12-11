#include <ros/ros.h>
#include <iostream> //iostream present for testing and error messages

#include <geometry_msgs/Pose.h> //msgs for start pos subscriber
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>

#include <kobuki_msgs/PowerSystemEvent.h>  //Kobuki_node capable of detecting changes to the Power system http://docs.ros.org/en/api/kobuki_msgs/html/msg/PowerSystemEvent.html
#include <sensor_msgs/BatteryState.h>      //Used for laptop battery information (could be used for Kobuki as well but havent figured out how yet)
#include <kobuki_msgs/AutoDockingAction.h> //Used for the autodocking feature
#include <kobuki_msgs/AutoDockingGoal.h>   //Used for the autodocking feature

//Following inclusions used for MovingToPosition class
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

//#include <std_msgs/Int16.h>

//Actionlib
//typedef actionlib::SimpleActionClient<kobuki_msgs::AutoDockingAction> dockingClient;
//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveBaseClient;
//kobuki_msgs::AutoDockingGoal dockingGoal;
move_base_msgs::MoveBaseGoal homeGoal;

//Declaration of subscribers
ros::Subscriber kobukiBatStateSub;
ros::Subscriber laptopBatlevelSub;
ros::Subscriber startPoseSub;
//ros::Publisher  stopOtherTasksPub;

//Declaration of callback messagetypes
kobuki_msgs::PowerSystemEvent kobBatState;
sensor_msgs::BatteryState laptopBatLevel;

//Initialisation and declaration of global variables
bool fullyCharged = false;      //
double savedPose[2];            //
double laptopBatteryPercentage; //Has to be double as sensor_msgs::BatteryState laptopBatLevel.percentage is a double

class MovingToPosition
{
private:
  tf::TransformListener listener;

  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  typedef actionlib::SimpleActionClient<kobuki_msgs::AutoDockingAction> dockingClient;
public:
  double getPosition(double mapPose[])//Function records a pair of x and y coordinates in an array at index 0 and 1
  {
    ros::spinOnce();
    geometry_msgs::PoseStamped pBase, pMap;
    pBase.header.frame_id = "base_link";
    pBase.pose.position.x = 0.0;
    pBase.pose.position.y = 0.0;
    pBase.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    ros::Time current_transform = ros::Time::now();
    ros::Duration(0.2).sleep();
    listener.getLatestCommonTime(pBase.header.frame_id, "map", current_transform, NULL);
    ros::Duration(0.2).sleep();
    pBase.header.stamp = current_transform;
    listener.transformPose("map", pBase, pMap);
    mapPose[0] = pMap.pose.position.x;
    mapPose[1] = pMap.pose.position.y;
  }
  
  void startDocking() //Docking procedure
  {
    kobuki_msgs::AutoDockingGoal dockingGoal;
    
    dockingClient dc("dock_drive_action", true); //Starts client, needs to be called "dock_drive_action" to work (true -> don't need ros::spin())
    dc.waitForServer();                          //Wait for feedback from the Action server

    dc.sendGoal(dockingGoal);                    //Sends new goal to nodelet managing the docking procedure (check /opt/ros/kinetic/share/kobuki_auto_docking/launch/minimal.launch for additions to launch file)
    dc.waitForResult();                          //ros::Duration(5.0) for maximum wait time?

    if (dc.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Turtlebot reached dock");
    }
    else
    {
      ROS_INFO("Turtlebot failed to reach dock");
    }
  }

  void moveToDock(double posX, double posY, const char *oriantation)
  {
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = posX;
    goal.target_pose.pose.position.y = posY;

    //orientation based on Quaternions http://wiki.ogre3d.org/Quaternion+and+Rotation+Primer
    if (oriantation == "Right")
    {
      goal.target_pose.pose.orientation.w = -sqrt(0.5);
      goal.target_pose.pose.orientation.z = sqrt(0.5);
    }
    else if (oriantation == "Left")
    {
      goal.target_pose.pose.orientation.w = sqrt(0.5);
      goal.target_pose.pose.orientation.z = -sqrt(0.5);
    }
    else if (oriantation == "Backwards")
    {
      goal.target_pose.pose.orientation.w = 0;
      goal.target_pose.pose.orientation.z = 1;
    }
    else
    {
      goal.target_pose.pose.orientation.w = 1;
      goal.target_pose.pose.orientation.z = 0;
    }
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Sucess, the base moved to the goal, starting docking");
      startDocking();
    }
    else
      ROS_INFO("The base failed to move to the goal");
  }
};

//Callback function saves x and y coordinates from subscriber
void callbackStartPose(const geometry_msgs::Pose startPose)
{
  homeGoal.target_pose.pose.position.x = startPose.position.x;
  homeGoal.target_pose.pose.position.y = startPose.position.y;
  std::cout << "Ready to go! Start position at x = " << homeGoal.target_pose.pose.position.x << ", y = " << homeGoal.target_pose.pose.position.y << std::endl;
  startPoseSub.shutdown(); //Subscriber only needs to update home once (tested this, works)
}

//Function for sending the robot back to the dock
void headHomeToCharge()
{
  //Stop current task
  //std_msgs::Int16 stopTasksMsg;
  //stopTasksMsg.data = 1;
  //stopOtherTasksPub.publish(stopTasksMsg);
  //ros::spinOnce();

  MovingToPosition moveClass;
  moveClass.getPosition(savedPose);
  moveClass.moveToDock(homeGoal.target_pose.pose.position.x, homeGoal.target_pose.pose.position.y, "Forwards");
}

/*
//Function to return to previous location so the robot can resume work
void resumeDemining()
{
  //Move out of dock (badness 10000)
  MovingToPosition moveClass;
  moveClass.moveToDock(homeGoal.target_pose.pose.position.x, homeGoal.target_pose.pose.position.y, "Backwards");
  
  //Find its way back towards saved location
  moveClass.moveToDock(savedPose[0], savedPose[1], "Forwards");
  
  //Resume demining task
}
*/

void callbackKobukiBatState(const kobuki_msgs::PowerSystemEvent kobBatState) //removed & before kobBatState
{
  switch (kobBatState.event)
  {
  case 0: //Unplugged from charger
    if (!fullyCharged)
    {
      std::cout << "Base not fully charged, operation time will be reduced" << std::endl;
    }
    fullyCharged = false;
    break;

  case 1: //Charging with adapter, for active operation use dock instead.
    std::cout << "Base charging with adapter, but why?" << std::endl;
    break;

  case 2: //Base has reached dock
    std::cout << "Base has started charging" << std::endl;
    break;

  case 3: //Base has reached maximum charge
    std::cout << "Base fully charged! Ready to go" << std::endl;
    fullyCharged = true;

    //resumeDemining(); //Return to demining task
    break;

  case 4: //Base is low on battery (15%)
    std::cout << "Base is low on battery (15%). Pausing operation and heading to dock" << std::endl;

    headHomeToCharge(); //Send robot home to recharge base
    break;

  case 5: //Base battery level is critical! (5%)
    std::cout << "Base battery level is critical! (5%)" << std::endl;
    break;

  default: //Error! this should be unreachable
    std::cout << "Something when wrong with /mobile_base/events/power_system, it sent wrong data" << std::endl;
    break;
  }
}

void callbackLaptopBat(const sensor_msgs::BatteryState laptopBatLevel)
{
  laptopBatteryPercentage = laptopBatLevel.percentage;

  //When laptop reaches 15% remaining power it the robot shall return home
  if (laptopBatLevel.percentage <= 15 && laptopBatLevel.power_supply_status != 1)
  {
    std::cout << "Laptop is low on battery (" << laptopBatLevel.percentage << "%). Heading to dock" << std::endl;

    headHomeToCharge(); //Send robot home to recharge laptop
  }

  //When laptop reaches 5% remaining power it will send a message for testing the requirement
  else if (laptopBatLevel.percentage <= 5 && laptopBatLevel.power_supply_status != 1)
  {
    std::cout << "Laptop battery level is critical! (" << laptopBatLevel.percentage << "%)" << std::endl;

    headHomeToCharge(); //Send robot should not stop trying to get home
  }

  /*
  //When the laptop is fully charged it will move back to the demining task
  else if (laptopBatLevel.percentage == 100 && laptopBatLevel.power_supply_status == 1)
  {
    fullyCharged = true;

    //resumeDemining(); //Return to demining task
  }
  */
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "batteryMonitor"); //initialises node as batteryMonitor
  ros::NodeHandle n;
  MovingToPosition moveClass;

  //Subscribes to the start position determined by another node
  startPoseSub = n.subscribe("start_position", 1, callbackStartPose);

  //Subscribes to the laptop battery
  laptopBatlevelSub = n.subscribe("/laptop_charge", 1, callbackLaptopBat);

  //Subscibes to the PowerSystemEvent message, it updates whenever something happens with the power system of the kobuki base
  kobukiBatStateSub = n.subscribe("/mobile_base/events/power_system", 10, callbackKobukiBatState);

  //stopOtherTasksPub = n.advertise<std_msgs::Int16>("pause_Tasks", 10, true);

  //The following is for testing only
  //int lastInput;
  ros::spinOnce();
  
  while (ros::ok())
  {
    ros::spinOnce(); //Updates all subscribed and published information
    //std::cout << "input a number from 1 to 3: " << std::endl;
    //std::cin >> lastInput;

    //switch (lastInput)
    //{
    //case 1:
    //  std::cout << "Start position at x = " << homeGoal.target_pose.pose.position.x << ", y = " << homeGoal.target_pose.pose.position.y << std::endl;
    //  break;
    //case 2:
    //  headHomeToCharge();
    //  std::cout << "Saved coordinates are: x = " << savedPose[0] << ", y = " << savedPose[1] << std::endl;
    //  break;
    //case 3:
    //  ros::spin();
    //  return 0;
    //default:
    //  std::cout << "Other input recieved, Ending program" << std::endl;
    //  return 0;
    //}
  }
  return 0;
}