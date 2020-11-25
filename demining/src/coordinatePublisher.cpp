#include <ros/ros.h>
#include <iostream>                       
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>

ros::Publisher startPosePub;
ros::Subscriber startPoseSub;

geometry_msgs::Pose2D startPose;

void startPoseAssign(const nav_msgs::Odometry odomPose)
{
  startPose.x = odomPose.pose.pose.position.x;
  startPose.y = odomPose.pose.pose.position.y;
  startPoseSub.shutdown();
  startPosePub.publish(startPose);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "coordinatePublisher");
  ros::NodeHandle n;

  startPosePub = n.advertise <geometry_msgs::Pose2D> ("/start_position", 1, true);
  startPoseSub = n.subscribe ("/odom", 1, startPoseAssign);

  ros::spin();

  return 0;
}
