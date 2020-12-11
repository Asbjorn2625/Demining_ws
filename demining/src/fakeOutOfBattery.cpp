#include <ros/ros.h>
#include <iostream>  
#include <kobuki_msgs/PowerSystemEvent.h>  //Kobuki_node capable of detecting changes to the Power system http://docs.ros.org/en/api/kobuki_msgs/html/msg/PowerSystemEvent.html
//#include <sensor_msgs/BatteryState.h>      //Used for laptop battery information (could be used for Kobuki as well but havent figured out how yet)

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "fakeOutOfBattery");
  ros::NodeHandle n;

  ros::Publisher fakeBatBasePub = n.advertise<kobuki_msgs::PowerSystemEvent>("/mobile_base/events/power_system" ,10, true);
  //ros::Publisher fakeBatLaptPub = n.advertise<sensor_msgs::BatteryState>("/laptop_charge" ,1000, true);

  int fakeBatBaseVal =  4;//State message for low battery
  //int fakeBatLaptVal = 14;//Percentage level for low battery

  kobuki_msgs::PowerSystemEvent fakeBatBaseMsg;
  //sensor_msgs::BatteryState     fakeBatLaptMsg;
  fakeBatBaseMsg.event =      fakeBatBaseVal;
  //fakeBatLaptMsg.percentage = fakeBatLaptVal;

  /*
  int lastKey = 0;
  std::cout << "Input 1 for low base battery" << std::endl 
            << "or 2 for low laptop battery." << std::endl
            << "Input: "                      << std::endl;
  std::cin  >> lastKey;

  if (lastKey == 1)
  {
    */
    fakeBatBasePub.publish(fakeBatBaseMsg);
    std::cout << "Base battery level is now set to low" << std::endl;
    ros::spinOnce();
    /*
  }
  else if (lastKey == 2)
  {
    fakeBatLaptPub.publish(fakeBatLaptMsg);
    std::cout << "Laptop battery level is now set to low" << std::endl;
    ros::spin();
  }
  else
  {
    std::cout << "Wrong input, closing program" << std::endl;
  }
  */
  return 0;
}
