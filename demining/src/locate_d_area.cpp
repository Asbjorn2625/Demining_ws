#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/Sound.h>
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include <tf/transform_listener.h>

ros::Publisher sound_pub;
ros::Publisher vel_pub;
ros::Subscriber poseSub;

nav_msgs::Odometry odomPose;
geometry_msgs::Twist twist;
kobuki_msgs::Sound soundMsg;

void poseCallback(const nav_msgs::Odometry::ConstPtr & pose_message){
	odomPose.pose.pose.position.x=pose_message->pose.pose.position.x;
	odomPose.pose.pose.position.y=pose_message->pose.pose.position.y;
	odomPose.pose.pose.position.z=pose_message->pose.pose.position.z;

	odomPose.pose.pose.orientation.w=pose_message->pose.pose.orientation.w;
	odomPose.pose.pose.orientation.x=pose_message->pose.pose.orientation.x;
	odomPose.pose.pose.orientation.y=pose_message->pose.pose.orientation.y;
	odomPose.pose.pose.orientation.z=pose_message->pose.pose.orientation.z;
}

// makes conversion from radian to degree
double radian2degree(double radianAngle){
	return (radianAngle*57.2957795);
}



// makes conversion from degree to radian
double degree2radian(double degreeAngle){
	return (degreeAngle/57.2957795);
}

double posFunc(double pos[3]){
        soundMsg.value = 4;
        sound_pub.publish(soundMsg);
        //gotta turn off the sound, or it will sometimes loop the sound
        soundMsg.value = 1;
        sound_pub.publish(soundMsg);

    //save the positions into the variables
    pos[0] = odomPose.pose.pose.position.x;
    pos[1] = odomPose.pose.pose.position.y;
    pos[2] = radian2degree(tf::getYaw(odomPose.pose.pose.orientation));   
            //ROS_INFO("robot current pose: (x = %.2f, y = %.2f, angle = %.2f)\n", startX, startY, startAngle);
}

void moveForward(int lengthMeters){
  for(int i=0;i<lengthMeters*50;i++){
      twist.linear.x = 0.2;
      twist.angular.z = 0.0;
      vel_pub.publish(twist);
      ros::Duration(0.1).sleep();
    }
}
void moveBackward(int lengthMeters){
  for(int i=0;i<lengthMeters*50;i++){
      twist.linear.x = -0.2;
      twist.angular.z = 0.0;
      vel_pub.publish(twist);
      ros::Duration(0.1).sleep();
    }
}
void turn180deg(double currentAngle, double desiredAngle, int errorFactor){
 while(currentAngle <= desiredAngle-errorFactor || currentAngle >=desiredAngle+errorFactor){
      ros::spinOnce();
      twist.linear.x = 0.0;
      twist.angular.z = 1.0;
      vel_pub.publish(twist);
      currentAngle=radian2degree(tf::getYaw(odomPose.pose.pose.orientation));
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"locate_d_area");
    srand(time(NULL));
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    //advertisers
    sound_pub=n.advertise<kobuki_msgs::Sound> ("/mobile_base/commands/sound", 2);
    vel_pub=n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",10);
    //subscribers
    poseSub=n.subscribe("/odom", 10, poseCallback);
    
    
while(ros::ok())
{
  std::cout << ("start finding start position\n");
  //start
    moveBackward(1);

    //get the start position
    ros::spinOnce();
    double startPos[3];
    for(int i; 2 >= i; i++){
       posFunc(startPos);
    }
    printf("robot current pose: (x = %.2f, y = %.2f, angle = %.2f)\n", startPos[0], startPos[1], startPos[2]);


    //turning 180 degress
    if(startPos[2] > 0){
      std::cout << "Angle goal: " << startPos[2]-180 << "\n";
      turn180deg(startPos[2],startPos[2]-180,1);
    }else{
      std::cout << "Angle goal: " << startPos[2]+180 << "\n";
      turn180deg(startPos[2],startPos[2]+180,1);
    }
  std::cout << "angle reached: " << radian2degree(tf::getYaw(odomPose.pose.pose.orientation)) << "\n";

//input for the mine zone
bool errors = true;
int corners = 2;
int mineZone[corners];
while(errors == true){
  while(true){
  std::cout << "please input the size of the mine zone, in meters \nwidth: ";
  std::cin >> mineZone[0];
    if(mineZone[0] > 10)
    break;
  std::cout << "height: ";
  std::cin >> mineZone[1];
    if(mineZone[1] > 10)
    break;
    errors = false;
    break;
  }
}

    return 0;

}


    return 0;
}

