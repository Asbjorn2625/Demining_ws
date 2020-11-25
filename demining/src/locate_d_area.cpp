#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

class MovingToPosition{
private:
ros::NodeHandle n;

tf::TransformListener listener;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("Demining_markers", 1);
public:

double getPosition(double mapPose[]){
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

void moveTo(double posX, double posY, const char* oriantation){
 MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
move_base_msgs::MoveBaseGoal goal;



  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = posX;
  goal.target_pose.pose.position.y = posY;

if(oriantation == "Right"){
	goal.target_pose.pose.orientation.w = -sqrt(0.5);
	goal.target_pose.pose.orientation.z = sqrt(0.5);
}else if(oriantation == "Left"){
	goal.target_pose.pose.orientation.w = sqrt(0.5);
	goal.target_pose.pose.orientation.z = -sqrt(0.5);
}
else if(oriantation == "Backwards"){
	goal.target_pose.pose.orientation.w = 0;
	goal.target_pose.pose.orientation.z = 1;	
}else{
	goal.target_pose.pose.orientation.w = 1;
	goal.target_pose.pose.orientation.z = 0;	
}
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Sucess, the base moved to the goal");
  else
    ROS_INFO("The base failed to move to the goal");

}

void moveToMap(double posX, double posY, const char* oriantation){
 MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
move_base_msgs::MoveBaseGoal goal;



  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = posX;
  goal.target_pose.pose.position.y = posY;

//orientation based on Quaternions http://wiki.ogre3d.org/Quaternion+and+Rotation+Primer
if(oriantation == "Right"){
	goal.target_pose.pose.orientation.w = -sqrt(0.5);
	goal.target_pose.pose.orientation.z = sqrt(0.5);
}else if(oriantation == "Left"){
	goal.target_pose.pose.orientation.w = sqrt(0.5);
	goal.target_pose.pose.orientation.z = -sqrt(0.5);
}
else if(oriantation == "Backwards"){
	goal.target_pose.pose.orientation.w = 0;
	goal.target_pose.pose.orientation.z = 1;	
}else{
	goal.target_pose.pose.orientation.w = 1;
	goal.target_pose.pose.orientation.z = 0;	
}
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Sucess, the base moved to the goal");
  else
    ROS_INFO("The base failed to move to the goal");

}


void setPointMap(double posX, double posY, double size, double height, uint32_t shape){
//visualization_msgs::Marker::CYLINDER
visualization_msgs::Marker marker_array;
  marker_array.header.frame_id = "/map";
  marker_array.header.stamp = ros::Time::now();
  marker_array.ns = "map_pointers";
  marker_array.id = 0;
  marker_array.type = shape;
  marker_array.action = visualization_msgs::Marker::ADD;
  marker_array.pose.position.x = posX;
  marker_array.pose.position.y = posY;
  marker_array.pose.position.z = 0.0;
  marker_array.pose.orientation.x = 0.0;
  marker_array.pose.orientation.y = 0.0;
  marker_array.pose.orientation.z = 0.0;
  marker_array.pose.orientation.w = 1.0;

  marker_array.scale.x = size;
  marker_array.scale.y = size;
  marker_array.scale.z = height;

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
}
};

int main(int argc, char** argv){

	ros::init(argc, argv, "locate_d_area");
	MovingToPosition start;
	ros::Rate loop_rate(10.0);

	int corners = 2;
	double mineZone[corners];

while (ros::ok()){
		ros::spinOnce();loop_rate.sleep();
bool errors = true;

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

		//moving according to the robots position
    /*start.moveTo(-1.0, 0.0, "Backwards");
    start.getPosition(positionInMap);
    start.setPoint(positionInMap.pose.position.x,positionInMap.pose.position.y,0.2,0.8, visualization_msgs::Marker::CYLINDER);
		std::cout << "Starting demining";
		for (int i=0;i<mineZone[0];i++){
		start.moveTo(mineZone[1], 0.0, "Right");
		start.moveTo(0.5, 0.0,"Right");
    start.moveTo(mineZone[1], 0.0, "Left");
		start.moveTo(0.5, 0.0,"Left");
    }*/

  //moving according to the map
  double startPositions[3];
  double minePositions[3];
  ros::spinOnce();
  start.getPosition(startPositions);
  printf("robot pose: (%.2f, %.2f)\n", startPositions[0], startPositions[1]);
  start.moveTo(-1.0, 0.0, "Backwards");
  ros::spinOnce();
  start.getPosition(minePositions);
  printf("robot pose: (%.2f, %.2f)\n", minePositions[0], minePositions[1]);

  //calculate angle
  double x_distance = minePositions[0] - startPositions[0];
  double y_distance = minePositions[1] - startPositions[1];
  double heading =atan2(y_distance,x_distance);
  ROS_INFO("heading (%2f)", heading);

  double xPoint1;
  double xPoint2;
  double yPoint1;
  double yPoint2;
  if(heading < 0){
      //first point
  xPoint1 = minePositions[0]+mineZone[1]*sin(heading);
  yPoint1 = minePositions[1]+mineZone[1]*cos(heading);
  }else if(heading > 0){
  xPoint1 = minePositions[0]+mineZone[1]*cos(heading);
  yPoint1 = minePositions[1]+mineZone[1]*sin(heading);
  }else{
    std::cout << "failed angle \n";
  }

//second point
  if(heading-M_PI/2 < 0){
      //first point
  xPoint2 = xPoint1+mineZone[1]*sin(heading-M_PI/2);
  yPoint2 = yPoint1+mineZone[1]*cos(heading-M_PI/2);
  }else if(heading > 0){
  xPoint2 = xPoint1+mineZone[1]*cos(heading-M_PI/2);
  yPoint2 = yPoint1+mineZone[1]*sin(heading-M_PI/2);
  }else{
    std::cout << "failed angle \n";
  }
printf("location 1 = (%f,%f) location 2 = (%f,%f)\n", xPoint1,yPoint1,xPoint2,yPoint2);
  loop_rate.sleep();
  start.moveToMap(xPoint1,yPoint1,"Right");
loop_rate.sleep();
start.moveToMap(xPoint2,yPoint2,"Right");

		ros::spinOnce();loop_rate.sleep();ros::spinOnce();
		//printf("robot final pose: (%.2f, %.2f, %.2f)\n", turtlebot_odom_pose.pose.pose.position.x, turtlebot_odom_pose.pose.pose.position.y,radian2degree(tf::getYaw(turtlebot_odom_pose.pose.pose.orientation)));
		return 0;
	}



  return 0;
}