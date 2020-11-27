#include "ros/ros.h"
#include "std_msgs/String.h"
#include "kobuki_msgs/BumperEvent.h"

ros::Subscriber bumperSub;


int main(int argc, char **argv[]){
  
  ros::init(argc, argv, "evade");


  ros::NodeHandle n;

  
  ros::Subscriber bumperSub = n.subscribe("bumper", 1000);

  while (ros::ok()){
    if (bumper = 1){
      std::cout <<"fuck"<< std::endl;
    };
  };


  ros::spinOnce();

  return 0;
};