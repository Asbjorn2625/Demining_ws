#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


class MovingToPosition{
private:
ros::NodeHandle n;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
/*
const int Right = 0;
const int Straight = 1;
const int Left = 2;
const int Backwards = 3;
*/
public:
void moveTo(double posX, double posY, int oriantation){
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

if(oriantation == 0){
	goal.target_pose.pose.orientation.w = -sqrt(0.5);
	goal.target_pose.pose.orientation.z = sqrt(0.5);
}else if(oriantation == 2){
	goal.target_pose.pose.orientation.w = sqrt(0.5);
	goal.target_pose.pose.orientation.z = -sqrt(0.5);
}
else if(oriantation == 3){
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
};

int main(int argc, char** argv){

	ros::init(argc, argv, "locate_d_area");
	MovingToPosition start;
	ros::Rate loop_rate(10.0);

	int corners = 2;
	double mineZone[corners];

while (ros::ok()){
		ros::spinOnce();loop_rate.sleep();
		/*printf("robot initial pose: (%.2f, %.2f, %.2f)\n",
										turtlebot_odom_pose.pose.pose.position.x,
										turtlebot_odom_pose.pose.pose.position.y,
										radian2degree(tf::getYaw(turtlebot_odom_pose.pose.pose.orientation)));
                    */
    //getting the minezone from Userinput
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
		//start.moveTo(1, 0.0, 1);
		std::cout << "Starting demining";
		
		//for (int i=0;i<mineZone[0];i++){
		start.moveTo(mineZone[1], 0.0,0);
		//start.moveTo(0.0, -0.5,1.0);
		//start.moveTo(-mineZone[1], 0.0,1.0);
		//start.moveTo(0.0, -0.5,1.0);
    //}
		ros::spinOnce();loop_rate.sleep();ros::spinOnce();
		//printf("robot final pose: (%.2f, %.2f, %.2f)\n", turtlebot_odom_pose.pose.pose.position.x, turtlebot_odom_pose.pose.pose.position.y,radian2degree(tf::getYaw(turtlebot_odom_pose.pose.pose.orientation)));
		return 0;
	}



  return 0;
}