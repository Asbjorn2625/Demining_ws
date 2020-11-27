#include <ros/ros.h>
#include <iostream>                       
#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>

//Following inclusions used for MovingToPosition class
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

class MovingToPosition
{
private:
  tf::TransformListener listener;

  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

public:
  double getPosition(double mapPose[])
  {
    ros::spinOnce();
    geometry_msgs::PoseStamped pBase, pMap;
    pBase.header.frame_id = "base_link";
    pBase.pose.position.x = 0.0;
    pBase.pose.position.y = 0.0;
    pBase.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    ros::Time current_transform = ros::Time::now();
    ros::Duration(0.2).sleep();
    listener.getLatestCommonTime(pBase.header.frame_id, "map", current_transform, NULL);
    ros::Duration(0.2).sleep();
    pBase.header.stamp = current_transform;
    listener.transformPose("map", pBase, pMap);
    //printf("robot pose: (%.2f, %.2f)\n", pMap.pose.position.x, pMap.pose.position.y);
    mapPose[0] = pMap.pose.position.x;
    mapPose[1] = pMap.pose.position.y;
  }
};

ros::Publisher startPosePub;

geometry_msgs::Pose startPose;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "coordinatePublisher");
  ros::NodeHandle n;

  startPosePub = n.advertise <geometry_msgs::Pose> ("start_position", 1, true);

  double startPoseArr[2];
  MovingToPosition moveClass;
  moveClass.getPosition(startPoseArr);
  startPose.position.x = startPoseArr[0];
  startPose.position.y = startPoseArr[1];
  startPosePub.publish(startPose);

  ros::spin();
  return 0;
}
