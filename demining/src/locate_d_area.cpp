#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>

class MovingToPosition{
private:
ros::NodeHandle n;
tf::TransformListener listener;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("Demining_markers", 10);
move_base_msgs::MoveBaseGoal goal;

public:
std::string userInput = "";
ros::Publisher deminingArea_pub = n.advertise<geometry_msgs::Pose>("start_position", 10);

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

mapPose[0] = pMap.pose.position.x;
mapPose[1] = pMap.pose.position.y;
mapPose[2] = pMap.pose.orientation.w;
mapPose[3] = pMap.pose.orientation.z;
}

void moveTo(double posX, double posY, const char* oriantation){
 MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

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
  else{
    ROS_INFO("The base failed to move to the goal. \nDo you wish to continue? y/n \n");
    std::getline(std::cin, userInput);
    if(userInput == "n"){
      std::exit(0);
    }else if(userInput == "y"){
    }
  }
}

void moveToMap(double posX, double posY){
 MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = posX;
  goal.target_pose.pose.position.y = posY;

  goal.target_pose.pose.orientation.w = 1;
	goal.target_pose.pose.orientation.z = 0;	

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();
  ros::Duration(0.2).sleep();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Sucess, the base moved to the goal");
  else{
    ROS_INFO("The base failed to move to the goal. \nDo you wish to continue? y/n \n");
    std::getline(std::cin, userInput);
    if(userInput == "n")
      std::exit(0);
  }
}


void setPointPath(double posX[],double posY[],int pointAmount){

    visualization_msgs::Marker points, line_strip, line_list;
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/map";
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = line_list.ns = "path_and_turn_points";
    points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

    points.id = 0;
    line_strip.id = 1;
    line_list.id = 2;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    points.scale.x = 0.2;
    points.scale.y = 0.2;

    line_strip.scale.x = 0.1;
    line_list.scale.x = 0.1;

    // Points are red
    points.color.r = 1.0;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // Line list is yellow
    line_list.color.r = 1.0;
    line_list.color.g = 1.0;
    line_list.color.a = 1.0;

    points.lifetime = line_list.lifetime = line_strip.lifetime = ros::Duration();

for(int i=0; i < pointAmount+1; i++){
      geometry_msgs::Point p;
      p.x = posX[i];
      p.y = posY[i];

      points.points.push_back(p);
      line_strip.points.push_back(p);

      // The line list needs two points for each line
      line_list.points.push_back(p);
      p.z += 1;  
      line_list.points.push_back(p);
}
    marker_pub.publish(points);
    marker_pub.publish(line_strip);
    marker_pub.publish(line_list);

    ros::Duration(0.3).sleep();
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
  double startPositions[4];
  double minePositions[4];
  int pointsOnMap = mineZone[0]*4;
  double xPoint[pointsOnMap];
  double yPoint[pointsOnMap];
  geometry_msgs::Pose start_msg;

  //moving according to the map
  ros::spinOnce();
  start.getPosition(startPositions);
  printf("robot pose: (%.2f, %.2f)\n", startPositions[0], startPositions[1]);
  start.moveTo(-1.0, 0.0, "Backwards"); //moving directly backwards without a map
  ros::spinOnce();
  start.getPosition(minePositions);
  printf("robot pose: (%.2f, %.2f)\n", minePositions[0], minePositions[1]);
  xPoint[0] = minePositions[0]; //saving pos for later, and publishing it
  yPoint[0] = minePositions[1];

  start_msg.position.x = minePositions[0];
  start_msg.position.y = minePositions[1];
  start_msg.orientation.w = minePositions[2];
  start_msg.orientation.z = minePositions[3];
  start.deminingArea_pub.publish(start_msg);

  //calculate angle
  double x_distance = minePositions[0] - startPositions[0];
  double y_distance = minePositions[1] - startPositions[1];
  double heading =atan2(y_distance,x_distance);
  ROS_INFO("heading (%2f)", heading);

for (int i=1;i<pointsOnMap+1;){
  //first point
  xPoint[i] = xPoint[i-1]+mineZone[1]*cos(heading);
  yPoint[i] = yPoint[i-1]+mineZone[1]*sin(heading);

  //second point
  xPoint[i+1] = xPoint[i]+1.0*cos(heading-M_PI/2);
  yPoint[i+1] = yPoint[i]+1.0*sin(heading-M_PI/2);

  //third point
  xPoint[i+2] = xPoint[i+1]-mineZone[1]*cos(heading);
  yPoint[i+2] = yPoint[i+1]-mineZone[1]*sin(heading);

  //fourth point
  xPoint[i+3] = xPoint[i+2]+1.0*cos(heading-M_PI/2);
  yPoint[i+3] = yPoint[i+2]+1.0*sin(heading-M_PI/2);
i = i+4;
}
//last point
  xPoint[pointsOnMap+1] = xPoint[pointsOnMap]+mineZone[1]*cos(heading);
  yPoint[pointsOnMap+1] = yPoint[pointsOnMap]+mineZone[1]*sin(heading);

start.setPointPath(xPoint,yPoint,pointsOnMap);

for(int i=0; i < pointsOnMap+1;){
printf("location %d = (%.2f,%.2f)\n", i,xPoint[i],yPoint[i]);
i++;
}

for (int i =1;i<mineZone[0]*4+1;i++){
  ros::Duration(1.0).sleep();
  start.moveToMap(xPoint[i],yPoint[i]);
}
		return 0;
	}
  return 0;
}