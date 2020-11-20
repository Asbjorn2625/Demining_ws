#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/Sound.h>
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include <tf/transform_listener.h>
#include <std_msgs/Empty.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>


ros::Publisher sound_pub;
//ros::Publisher vel_pub;
ros::Publisher reset_pub;
ros::Publisher marker_pub;
ros::Subscriber poseSub;

move_base_msgs::MoveBaseGoal goal;
nav_msgs::Odometry odomPose;

geometry_msgs::Twist twist;
kobuki_msgs::Sound soundMsg;
std_msgs::Empty reset_msg;
visualization_msgs::Marker marker_array;

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
ros::Duration(0.1).sleep();
    //save the positions into the variables
    ros::spinOnce();
    pos[0] = odomPose.pose.pose.position.x;
    pos[1] = odomPose.pose.pose.position.y;
    pos[2] = radian2degree(tf::getYaw(odomPose.pose.pose.orientation));   
            //ROS_INFO("robot current pose: (x = %.2f, y = %.2f, angle = %.2f)\n", startX, startY, startAngle);
}

void setPoint(double posX, double posY, double size){
  uint32_t shape = visualization_msgs::Marker::CUBE;

  marker_array.header.frame_id = "/map";
marker_array.header.stamp = ros::Time::now();
marker_array.ns = "basic_shapes";
marker_array.id = 0;
marker_array.type = shape;
marker_array.action = visualization_msgs::Marker::ADD;
  marker_array.pose.position.x = posX;
  marker_array.pose.position.y = posY;
  marker_array.pose.position.z = 0;
  marker_array.pose.orientation.x = 0.0;
  marker_array.pose.orientation.y = 0.0;
  marker_array.pose.orientation.z = 0.0;
  marker_array.pose.orientation.w = 1.0;

  marker_array.scale.x = size;
  marker_array.scale.y = size;
  marker_array.scale.z = size;

  marker_array.color.r = 0.0f;
  marker_array.color.g = 1.0f;
  marker_array.color.b = 0.0f;
  marker_array.color.a = 1.0;

marker_array.lifetime = ros::Duration();


   while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
       break;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker_array);

    switch (shape)
    {
    case visualization_msgs::Marker::CUBE:
      shape = visualization_msgs::Marker::SPHERE;
      break;
    case visualization_msgs::Marker::SPHERE:
      shape = visualization_msgs::Marker::ARROW;
      break;
    case visualization_msgs::Marker::ARROW:
      shape = visualization_msgs::Marker::CYLINDER;
      break;
    case visualization_msgs::Marker::CYLINDER:
      shape = visualization_msgs::Marker::CUBE;
      break;
    }
}

bool moveToGoal(double xGoal, double yGoal){

   //define a client for to send goal requests to the move_base server through a SimpleActionClient
   actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

   //wait for the action server to come up
   while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
   }

   move_base_msgs::MoveBaseGoal goal;

   //set up the frame parameters
   goal.target_pose.header.frame_id = "map";
   goal.target_pose.header.stamp = ros::Time::now();

   /* moving towards the goal*/

   goal.target_pose.pose.position.x =  xGoal;
   goal.target_pose.pose.position.y =  yGoal;
   goal.target_pose.pose.position.z =  0.0;
   goal.target_pose.pose.orientation.x = 0.0;
   goal.target_pose.pose.orientation.y = 0.0;
   goal.target_pose.pose.orientation.z = 0.0;
   goal.target_pose.pose.orientation.w = 1.0;

   ROS_INFO("Sending goal location ...");
   ac.sendGoal(goal);

   ac.waitForResult();

   if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("You have reached the destination");
      return true;
   }
   else{
      ROS_INFO("Something went wrong");
      return false;
   }

}

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"locate_d_area");
    srand(time(NULL));
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    //advertisers
    reset_pub = n.advertise<std_msgs::Empty>("/mobile_base/commands/reset_odometry", 1);
    marker_pub = n.advertise<visualization_msgs::Marker>("minezone_marker", 1);
    sound_pub=n.advertise<kobuki_msgs::Sound> ("/mobile_base/commands/sound", 2);
    //subscribers
    poseSub=n.subscribe("/odom", 10, poseCallback);
    
while(ros::ok())
{

 ros::spinOnce();
    double startPos[3];
    for(int i; 2 >= i; i++){
       posFunc(startPos);
    }
  moveToGoal(startPos[0],startPos[1]+1.0);
    printf("robot current pose: (x = %.2f, y = %.2f, angle = %.2f)\n", startPos[0], startPos[1]+1.0, startPos[2]);
    setPoint(startPos[0], startPos[1]+1.0, 0.2);

//reset_pub.publish(reset_msg);
  /*
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
      turnAngle(startPos[2],startPos[2]-180,1,1);
    }else{
      std::cout << "Angle goal: " << startPos[2]+180 << "\n";
      turnAngle(startPos[2],startPos[2]+180,1,-1);
    }
    ros::spinOnce();
  std::cout << "angle reached: " << radian2degree(tf::getYaw(odomPose.pose.pose.orientation)) << "\n";

//input for the mine zone
bool errors = true;
int corners = 2;
float mineZone[corners];
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
for(int i; i <= mineZone[0]*2; i++){
moveForward(mineZone[1]);
ros::spinOnce();
double angleReached = radian2degree(tf::getYaw(odomPose.pose.pose.orientation));
if(angleReached > 0){
  turnAngle(angleReached,angleReached-90,1,-1);
  moveForward(0.5);
  turnAngle(angleReached,angleReached-180,1,-1);
}else{
  turnAngle(angleReached,angleReached+90,1,+1);
  moveForward(0.5);
  turnAngle(angleReached,angleReached+180,1,+1);
}
printf("lap %d completed\n", i+1);
ros::Duration(1).sleep();
}
*/
   // return 0;

}


    return 0;
}

