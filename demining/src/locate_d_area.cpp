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
ros::Subscriber pose_subscriber;
nav_msgs::Odometry turtlebot_odom_pose;

void poseCallback(const nav_msgs::Odometry::ConstPtr & pose_message){
	turtlebot_odom_pose.pose.pose.position.x=pose_message->pose.pose.position.x;
	turtlebot_odom_pose.pose.pose.position.y=pose_message->pose.pose.position.y;
	turtlebot_odom_pose.pose.pose.position.z=pose_message->pose.pose.position.z;

	turtlebot_odom_pose.pose.pose.orientation.w=pose_message->pose.pose.orientation.w;
	turtlebot_odom_pose.pose.pose.orientation.x=pose_message->pose.pose.orientation.x;
	turtlebot_odom_pose.pose.pose.orientation.y=pose_message->pose.pose.orientation.y;
	turtlebot_odom_pose.pose.pose.orientation.z=pose_message->pose.pose.orientation.z;
}

/* makes conversion from radian to degree */
double radian2degree(double radianAngle){
	return (radianAngle*57.2957795);
}


/* makes conversion from degree to radian */
double degree2radian(double degreeAngle){
	return (degreeAngle/57.2957795);
}

void bumperHit(const kobuki_msgs::BumperEvent &bumpMsg){
    if(bumpMsg.state==1){
        soundMsg.value = 4;
        sound_pub.publish(soundMsg);
        //skal slukke for lyden igen ellers køre den nogen gange i loop
        soundMsg.value = 1;
        sound_pub.publish(soundMsg);

            ROS_INFO("robot current pose: (x = %.2f, y = %.2f, angle = %.2f)\n",
                                                        turtlebot_odom_pose.pose.pose.position.x,
										                turtlebot_odom_pose.pose.pose.position.y,
										                radian2degree(tf::getYaw(turtlebot_odom_pose.pose.pose.orientation)));
    
    }

}

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"locate_d_area");
    srand(time(NULL));
    ros::NodeHandle n;
    sound_pub=n.advertise<kobuki_msgs::Sound > ("/mobile_base/commands/sound", 2);
    pose_subscriber = n.subscribe("/odom", 10, poseCallback);
    bumperSub=n.subscribe("/mobile_base/events/bumper",1,bumperHit);
    
while(ros::ok())
{
  
    ros::spinOnce();

}


    return 0;
}

