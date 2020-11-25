#include <ros/ros.h>
#include "kobuki_msgs/BumperEvent.h"

void BumberROBSubCallBack(const kobuki_msgs::BumperEvent bumperMessage){
    
    if(bumperMessage.bumper == 0){
        std::cout << "!!!!!BANG!!!!! (Left)\n";
    }
    else if(bumperMessage.bumper == 1){
        std::cout << "!!!!!BANG!!!!! (Center)\n";
    }
    else if(bumperMessage.bumper == 2){
        std::cout << "!!!!!BANG!!!!! (Right)\n";
    }
    else{
        std::cout << ":-(\n";
    }
    

    
}



int main(int argc, char **argv){
ros::init(argc, argv, "Bumpertest");

ros::NodeHandle n;

ros::Subscriber BumberROBSub = n.subscribe("/mobile_base/events/bumper", 1000, BumberROBSubCallBack);


while (ros::ok){
    
ros::spinOnce();

}
return 0;
}
