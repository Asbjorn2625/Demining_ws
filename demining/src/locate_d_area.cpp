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

    for(int i=0;i<50;i++){
      twist.linear.x = -0.2;
      twist.angular.z = 0.0;
      vel_pub.publish(twist);
      loop_rate.sleep();
    }

    ros::spinOnce();
    //double startingAngle = radian2degree(tf::getYaw(odomPose.pose.pose.orientation));
    //double currentAngle = 20.01;
    for(int i; i<M_PI*10; i++){
      twist.linear.x = 0.0;
      twist.angular.z = 1.0;
      vel_pub.publish(twist);
      loop_rate.sleep();
      ros::spinOnce();
      //currentAngle=radian2degree(tf::getYaw(odomPose.pose.pose.orientation));
    }
    double startPos[3];
    //ros::Duration(3).sleep();
    for(int i; 2 >= i; i++){
       posFunc(startPos);
       ros::spinOnce();
    }
    
    std::cout << "robot current pose: (x = " << startPos[0] << " y = " << startPos[1] << " angle = " << startPos[2] << ")\n";
    ros::spinOnce();

    return 0;

}


    return 0;
}

