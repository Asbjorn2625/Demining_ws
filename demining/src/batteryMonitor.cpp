#include <ros/ros.h>

#include <iostream> //iostream present for testing purposes

#include <kobuki_msgs/PowerSystemEvent.h>   //Kobuki_node capable of detecting changes to the Power system
#include <sensor_msgs/BatteryState.h>       //Currenty unused

// Current laptop battery charge topic /laptop_charge (/percentage)
// Current kobuki battery charge topic /mobile_base/sensors/core/battery

//Initialising global variables
int const kobuki_max_charge_voltage = 163;//Voltage from base battery at full charge (measured in 0.1V) 
int const laptop_max_charge = 200;        //Needs to be found
bool fullyCharged = false;

//sensor_msgs::BatteryState kobBatLevel;
sensor_msgs::BatteryState laptopBatLevel;
kobuki_msgs::PowerSystemEvent kobBatState;
ros::Subscriber kobukiBatStateSub;
ros::Subscriber laptopBatlevelSub;

//void callbackKobukiBat(const sensor_msgs::BatteryState kobBatLevel)
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
    
    //Drive towards starting position
    //Activate auto-docking procedure
    //kobuki_msgs/AutoDocking.action http://docs.ros.org/en/api/kobuki_msgs/html/action/AutoDocking.html
    break;

  case 5: //Base battery level is critical! (5%)
    std::cout << "Base battery level is critical!" << std::endl;
    break;

  default://Error! this should be unreachable
    
    break;
  }
}

void callbackLaptopBat(const sensor_msgs::BatteryState laptopBatLevel)
{
  std::cout << "laptop battery is currently at " << laptopBatLevel.percentage << "%" << std::endl;

  //std::cout << "laptop battery is currently at " << laptopBatLevel << "%" << std::endl;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "batteryMonitor"); //initialises node as batteryMonitor
  ros::NodeHandle n;
  laptopBatlevelSub = n.subscribe("/laptop_charge", 10, callbackLaptopBat);
  
  
  //kobukiBatlevelSub = n.subscribe("/mobile_base/sensors/core/battery", 10, callbackKobukiBat);

  //Subscibes to the PowerSystemEvent message, it updates whenever a power systems related issue happens
  kobukiBatStateSub = n.subscribe("/mobile_base/events/power_system", 10, callbackKobukiBatState);

  while(ros::ok())
  {
    //sensor_msgs::BatteryState test; 

    ros::spinOnce();
  }
  
  return 0;
}