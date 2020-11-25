
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <string>


std::string Text = "";

int main(int argc, char **argv){
ros::init(argc, argv, "UserInput");

ros::NodeHandle n;

ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter",1000);

ros::Rate loop_rate(10);

int count =0; //To track number of send message to create uniqe string


while(ros::ok()){

std_msgs::String msg; //This is our message objekt to fill with data and send it
std::stringstream ss;

std::cout << "Please, enter : ";
  std::getline (std::cin, Text);
ss << Text;
msg.data = ss.str();
ROS_INFO("%s", msg.data.c_str());

chatter_pub.publish(msg);

ros::spinOnce();
loop_rate.sleep();

}

return 0;
}
