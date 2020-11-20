#include <ros/ros.h>
#include <iostream> //iostream present for testing purposes

#include <kobuki_msgs/PowerSystemEvent.h> //Kobuki_node capable of detecting changes to the Power system http://docs.ros.org/en/api/kobuki_msgs/html/msg/PowerSystemEvent.html
#include <sensor_msgs/BatteryState.h>     //Used for laptop battery information (could be used for Kobuki as well but havent figured out how yet)

#include <actionlib/client/simple_action_client.h>//actionlib for 
#include <kobuki_msgs/AutoDockingAction.h>//Used for the autodocking feature
#include <kobuki_msgs/AutoDockingGoal.h>  //Used for the autodocking feature
typedef actionlib::SimpleActionClient <kobuki_msgs::AutoDockingAction> dockingClient; //dockingClient 

// Current laptop battery charge topic /laptop_charge (/percentage)
// Current kobuki battery charge topic /mobile_base/sensors/core/battery

//Initialisation of global variables
int const kobuki_max_charge_voltage = 163;//Voltage from base battery at full charge (measured in 0.1V) 
bool fullyCharged = false;

//Declaration of callback constants messagetypes
kobuki_msgs::PowerSystemEvent kobBatState;
sensor_msgs::BatteryState kobBatLevel;
sensor_msgs::BatteryState laptopBatLevel;

//Declaration of subscribers
ros::Subscriber kobukiBatStateSub;
ros::Subscriber kobukiBatlevelSub;
ros::Subscriber laptopBatlevelSub;

//Function for sending the robot back to the start
void headHomeToCharge()
{
  //Save current location (and path?)

  //Drive towards starting position (following a safe route)

  //If manual control is taken pause this task and resume afterwards


  dockingClient client("dock_drive_action", true);//Starts client, needs to be called "dock_drive_action" to work (true -> don't need ros::spin())
  client.waitForServer();                         //Wait for feedback from the Action server
  kobuki_msgs::AutoDockingGoal goal;              //Sets docking as the goal
  
  client.sendGoal(goal);  //Sends new goal to nodelet managing the docking procedure (check /opt/ros/kinetic/share/kobuki_auto_docking/launch/minimal.launch for additions to launch file)
  client.waitForResult(); //ros::Duration(5.0) for maximum wait time?

  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    std::cout << "Turtlebot reached dock, and is charging" << std::endl;
  }
  else
  {
    std::cout << "Error did not reach dock. Current State: " << client.getState().toString().c_str() << std::endl;
  }
}

void callbackKobukiBatState(const kobuki_msgs::PowerSystemEvent &kobBatState)
{
  switch(kobBatState.event)
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
    break;

  case 4: //Base is low on battery (15%)
    std::cout << "Base is low on battery. Pausing operation and heading to dock" << std::endl;
    
    headHomeToCharge(); //Send robot home to recharge base
    break;

  case 5: //Base battery level is critical! (5%)
    std::cout << "Base battery level is critical!" << std::endl;
    break;

  default://Error! this should be unreachable
    std::cout << "Something when wrong with /mobile_base/events/power_system, it sent wrong data" << std::endl;
    break;
  }
}

void callbackLaptopBat(const sensor_msgs::BatteryState laptopBatLevel)
{
  std::cout << "Laptop battery is currently at " << laptopBatLevel.percentage << "%" << std::endl;

  if (laptopBatLevel.percentage <= 15)//When laptop reaches 15% remaining power it the robot shall return home
  //Testet og virker
  {
    std::cout << "Laptop is low on battery. Pausing operation and heading to dock" << std::endl;

    headHomeToCharge(); //Send robot home to recharge laptop
  }
  //Is charging?
}

/* attempt at better solution for kobuki base battery
void callbackKobukiBat(const sensor_msgs::BatteryState kobBatLevel)
{
  std::cout << "kobuki battery is currently at " << kobBatLevel.percentage << "%" << std::endl;
  
  //std::cout << "laptop battery is currently at " << laptopBatLevel << "%" << std::endl;
}
*/

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "batteryMonitor"); //initialises node as batteryMonitor
  ros::NodeHandle n;

  //Subscribes to the laptop battery
  laptopBatlevelSub = n.subscribe("/laptop_charge", 1, callbackLaptopBat);
  
  //More precise battery function with percentage option
  //kobukiBatlevelSub = n.subscribe("/mobile_base/sensors/core", 10, callbackKobukiBat);
  //Subscibes to the PowerSystemEvent message, it updates whenever a power systems related issue happens
  kobukiBatStateSub = n.subscribe("/mobile_base/events/power_system", 10, callbackKobukiBatState);

  while(ros::ok())
  {
    ros::spinOnce();//Updates all subscribed and published information
  }
  
  return 0;
}