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
}

void moveTo(double posX, double posY, const char* oriantation){
 MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
move_base_msgs::MoveBaseGoal goal;

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
move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = posX;
  goal.target_pose.pose.position.y = posY;

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
    }
  }
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

  
  double startPositions[3];
  double minePositions[3];
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
  xPoint[0] = minePositions[0]; //saving pos for later
  yPoint[0] = minePositions[1];

  start_msg.position.x = minePositions[0];
  start_msg.position.y = minePositions[1];
  start.deminingArea_pub.publish(start_msg);

  //calculate angle
  double x_distance = minePositions[0] - startPositions[0];
  double y_distance = minePositions[1] - startPositions[1];
  double heading =atan2(y_distance,x_distance);
  ROS_INFO("heading (%2f)", heading);

for (int i=1;i<mineZone[0]*2+1;){
  if(heading < 0){
  //first point
  xPoint[i] = xPoint[i-1]+mineZone[1]*sin(heading);
  yPoint[i] = yPoint[i-1]+mineZone[1]*cos(heading);
  }else if(heading > 0){
  xPoint[i] = xPoint[i-1]+mineZone[1]*cos(heading);
  yPoint[i] = yPoint[i-1]+mineZone[1]*sin(heading);
  }else{
    std::cout << "failed angle \n";
  }
  if(heading-M_PI/2 < 0)
  {//second point
  xPoint[i+1] = xPoint[i]+0.5*sin(heading-M_PI/2);
  yPoint[i+1] = yPoint[i]+0.5*cos(heading-M_PI/2);
  }else if(heading > 0){
  xPoint[i+1] = xPoint[i]+0.5*cos(heading-M_PI/2);
  yPoint[i+1] = yPoint[i]+0.5*sin(heading-M_PI/2);
  }else{
    std::cout << "failed angle \n";
  }
  printf("location (%d) = (%f,%f)\nlocation (%d) = (%f,%f)\n", i,xPoint[i],yPoint[i],i+1,xPoint[i+1],yPoint[i+1]);
  i++;
  i++;
}
for (int i =1;i<mineZone[0]*4+1;i++){
  loop_rate.sleep();
  start.moveToMap(xPoint[i],yPoint[i]);
}
		return 0;
	}



  return 0;
}