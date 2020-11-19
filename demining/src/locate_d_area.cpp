#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/Sound.h>
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include <tf/transform_listener.h>


kobuki_msgs::BumperEvent bumpMsg;
kobuki_msgs::Sound soundMsg;
ros::Publisher sound_pub;
ros::Subscriber bumperSub;
ros::Subscriber poseSub;
nav_msgs::Odometry odomPose;

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

void bumperHit(const kobuki_msgs::BumperEvent &bumpMsg){
    if(bumpMsg.state==1){

        soundMsg.value = 4;
        sound_pub.publish(soundMsg);
        //gotta turn off the sound, or it will sometimes loop the sound
        soundMsg.value = 1;
        sound_pub.publish(soundMsg);

    //init the variables for positions
    double startX, startY, startAngle;

    //save the positions into the variables
    startX = odomPose.pose.pose.position.x;
    startY = odomPose.pose.pose.position.y;
    startAngle = radian2degree(tf::getYaw(odomPose.pose.pose.orientation));   
            ROS_INFO("robot current pose: (x = %.2f, y = %.2f, angle = %.2f)\n", startX, startY, startAngle);
    }

}

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"locate_d_area");
    srand(time(NULL));
    ros::NodeHandle n;
    //advertisers
    sound_pub=n.advertise<kobuki_msgs::Sound> ("/mobile_base/commands/sound", 2);

    //subscribers
    poseSub=n.subscribe("/odom", 10, poseCallback);
    bumperSub=n.subscribe("/mobile_base/events/bumper",1,bumperHit);
    
while(ros::ok())
{
  
    ros::spinOnce();

}


    return 0;
}

