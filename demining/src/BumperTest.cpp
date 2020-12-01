#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <stdio.h>
#include "kobuki_msgs/BumperEvent.h" //Get input from bumper on bot
#include "std_msgs/Int32.h"

int rotates = 0;
int current_rotate =0;

void BumberROBSubCallBack(const kobuki_msgs::BumperEvent bumperMessage){      //Input from bumpersensor
    
    if(bumperMessage.bumper == 0){
        std::cout << "!!!!!BANG!!!!! (Left)\n";
        rotates++;
    }
    else if(bumperMessage.bumper == 1){
        std::cout << "!!!!!BANG!!!!! (Center)\n";
        rotates++;
    }
    else if(bumperMessage.bumper == 2){
        std::cout << "!!!!!BANG!!!!! (Right)\n";
        rotates++;
    }
    else{
        std::cout << ":-(\n";
    }   
}

void FindMinesSubCallBack(const std_msgs::Int32 MineMessage){               //Input from Minecamera
std::cout << "Recived : " << MineMessage.data << "\n";
//rotates++;
}





class MineReactor{
  private:
  ros::NodeHandle n;
  //init publisher
  ros::Publisher cmd_vel_pub_ = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
  //Init subscriber
  ros::Subscriber BumberROBSub = n.subscribe("/mobile_base/events/bumper", 1000, BumberROBSubCallBack);

  //init direction that turtlebot should go
  geometry_msgs::Twist base_cmd;

  void movement(const char* direction, int timer){
if(direction == "Left"){
  base_cmd.linear.x = 0;
  base_cmd.linear.y = 0;
  base_cmd.angular.z = 1.57/2;

}

else if(direction == "Right"){
  base_cmd.linear.x = 0;
  base_cmd.linear.y = 0;
  base_cmd.angular.z = -1.57/2;
}

else if(direction == "Forward"){
  base_cmd.linear.x = 0.25;
  base_cmd.linear.y = 0;
  base_cmd.angular.z = 0;
}

else if(direction == "Back"){
  base_cmd.linear.x = -0.25;
  base_cmd.linear.y = 0;
  base_cmd.angular.z = 0;
}

if(direction == "FLeft"){
  base_cmd.linear.x = 0.25;
  base_cmd.linear.y = 0;
  base_cmd.angular.z = 1.57/2;

}

else if(direction == "FRight"){
  base_cmd.linear.x = 0.25;
  base_cmd.linear.y = 0;
  base_cmd.angular.z = -1.57/2;
}

if(direction == "BLeft"){
  base_cmd.linear.x = -0.25;
  base_cmd.linear.y = 0;
  base_cmd.angular.z = 1.57/2;

}

else if(direction == "BRight"){
  base_cmd.linear.x = -0.25;
  base_cmd.linear.y = 0;
  base_cmd.angular.z = -1.57/2;
}


else if(direction == "Stop"){
  base_cmd.linear.x = 0;                            //Stopping
  base_cmd.linear.y = 0;
  base_cmd.angular.z = 0;
}

for(int n=timer; n>0; n--) {
  cmd_vel_pub_.publish(base_cmd);
  ros::Duration(0.1).sleep();
  ros::spinOnce();
}

}

void Foundmine(const geometry_msgs::PoseStamped MineMessage){
  std::cout << "Mine Found!(Mine number: " << MineMessage.header.frame_id <<" at : "  << MineMessage.pose.position.x << "," << MineMessage.pose.position.y << " )\n";
  //printf(("Mine Found! \nMine number = %s \nposition x = %.2f y = %.2f"), MineMessage.header.frame_id, MineMessage.pose.position.x,MineMessage.pose.position.y);
  MineMessage.header.frame_id;
  MineMessage.pose.position.x;
  MineMessage.pose.position.y;
  movement("Stop",1);
  movement("Back",5);
  movement("Right",20);
  movement("Forward",10);
  movement("Left",10);
  movement("Forward",10);

  
}
  public:
    //MineReactor():
    ros::Subscriber FindingminesNode = n.subscribe("mineCounter",1000, &MineReactor::Foundmine, this);
    //~MineReactor(){};


};



//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

int main(int argc, char **argv){ 
ros::init(argc, argv, "Bumpertest");

MineReactor running;

  //init publisher
  //ros::Publisher cmd_vel_pub_ = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
//Init subscriber
//ros::Subscriber BumberROBSub = n.subscribe("/mobile_base/events/bumper", 1000, BumberROBSubCallBack);
//ros::Subscriber FindingminesNode = n.subscribe("mineCounter",1000, FindMinesSubCallBack);
//ros::Rate rate(10); // 10Hz
    
while (ros::ok()){
ros::spinOnce();
}
return 0;
}
