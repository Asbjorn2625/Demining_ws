#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include "kobuki_msgs/BumperEvent.h" //Get input from bumper on bot
#include "std_msgs/Int32.h"

int rotates = 0;
int current_rotate =0;

void BumberROBSubCallBack(const kobuki_msgs::BumperEvent bumperMessage){
    
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

void FindMinesSubCallBack(const std_msgs::Int32 MineMessage){
std::cout << "Recived : " << MineMessage.data << "\n";
rotates++;
}



/*
void Forward(){
    //"publish" sends the command to turtlebot to keep going
    ROS_INFO_STREAM("sending forward");
    //go forward for 2 seconds
    for(int n=10; n>0; n--) {
      cmd_vel_pub_.publish(base_cmd);
      rate.sleep();
    }
}

void back(){
    //"publish" sends the command to turtlebot to keep going
    ROS_INFO_STREAM("sending backwards");
    //go forward for 2 seconds
    for(int n=10; n>0; n--) {
      cmd_vel_pub_.publish(base_cmd_turn_back);
      rate.sleep();
    }
}

void Left(){
ROS_INFO_STREAM("Sending left");
    //turn 90 degrees (takes 2 seconds)
    for(int n=10; n>0; n--) {
      cmd_vel_pub_.publish(base_cmd_turn_left);
      rate.sleep();
    }
}

void Right(){
ROS_INFO_STREAM("Sending Right");
    //turn 90 degrees (takes 2 seconds)
    for(int n=10; n>0; n--) {
      cmd_vel_pub_.publish(base_cmd_turn_right);
      rate.sleep();
    }
}
*/



int main(int argc, char **argv){ //--------------------------------------------------------
ros::init(argc, argv, "Bumpertest");

ros::NodeHandle n;

  //init publisher
  ros::Publisher cmd_vel_pub_ = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
//Init subscriber
ros::Subscriber BumberROBSub = n.subscribe("/mobile_base/events/bumper", 1000, BumberROBSubCallBack);
ros::Subscriber FindingminesNode = n.subscribe("mineCounter",1000, FindMinesSubCallBack);
ros::Rate rate(5); // 5Hz
    
  //init direction that turtlebot should go
  geometry_msgs::Twist base_cmd;
  geometry_msgs::Twist base_cmd_turn_left;
  geometry_msgs::Twist base_cmd_turn_right;
  geometry_msgs::Twist base_cmd_turn_back;

  base_cmd.linear.x = 0;
  base_cmd.linear.y = 0;
  base_cmd.angular.z = 0;
  base_cmd_turn_left.linear.x = 0; 
  base_cmd_turn_left.linear.y = 0;
  base_cmd_turn_left.angular.z = 0;
  base_cmd_turn_right.linear.x = 0; 
  base_cmd_turn_right.linear.y = 0;
  base_cmd_turn_right.angular.z = 0;
  base_cmd_turn_back.linear.x = 0; 
  base_cmd_turn_back.linear.y = 0;
  base_cmd_turn_back.angular.z = 0;


  //and let's go forward by setting X to a positive value
  base_cmd.linear.x = 0.25;
  base_cmd.angular.z = 0.0;
  ROS_INFO_STREAM("And Crashing ... ctrl + c to stop me :)");

  //base_cmd_turn_left will be used to turn turtlebot 90 degrees
  base_cmd_turn_back.linear.x = -0.25; //m/s
  base_cmd_turn_back.angular.z = 0.0; //45 deg/s * 2 sec = 90 degrees 

  //base_cmd_turn_left will be used to turn turtlebot 90 degrees
  base_cmd_turn_left.linear.x = 0; //m/s
  base_cmd_turn_left.angular.z = 1.57/2; //45 deg/s * 2 sec = 90 degrees 
  
  //base_cmd_turn_right will be used to turn turtlebot -90 degrees
  base_cmd_turn_right.linear.x = 0; //m/s
  base_cmd_turn_right.angular.z = -1.57/2; //45 deg/s * 2 sec = 90 degrees 





while (ros::ok()){


    if(rotates!=current_rotate && rotates>0){
      std::cout << "changing rotating\n";
      current_rotate=rotates;
      for(int n=10; n>0; n--) {
        cmd_vel_pub_.publish(base_cmd_turn_back);
        rate.sleep();
        ros::spinOnce();
      }

    }

    if(rotates>0){
      std::cout << "changing rotating\n";
    for(int n=10; n>0; n--) {
      cmd_vel_pub_.publish(base_cmd_turn_left);
      rate.sleep();
      ros::spinOnce();
    }
    for(int n=10; n>0; n--) {
      cmd_vel_pub_.publish(base_cmd);
      rate.sleep();
      ros::spinOnce();
    }
    for(int n=10; n>0; n--) {
      cmd_vel_pub_.publish(base_cmd_turn_right);
      rate.sleep();
      ros::spinOnce();
    }
    rotates=0;
    }
ros::spinOnce();
base_cmd.linear.x = 0;
  base_cmd.linear.y = 0;
  base_cmd.angular.z = 0;
  cmd_vel_pub_.publish(base_cmd);

}
base_cmd.linear.x = 0;
  base_cmd.linear.y = 0;
  base_cmd.angular.z = 0;
  cmd_vel_pub_.publish(base_cmd);

return 0;
}


