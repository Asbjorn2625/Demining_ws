#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <stdio.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#define LINEAR_VELOCITY_MINIMUM_THRESHOLD 0.2
#define ANGULAR_VELOCITY_MINIMUM_THRESHOLD 0.4 //navigation method gotten from https://github.com/aniskoubaa/gaitech_edu/blob/master/src/turtlebot/navigation/free_space_navigation/free_space_navigation.cpp



class MineReactor{
  private:
  ros::NodeHandle n;
  //init publisher
  ros::Publisher cmd_vel_pub;

  geometry_msgs::Twist base_cmd;

double calculateYaw( double x1, double y1, double x2,double y2)
{

	double bearing = atan2((y2 - y1),(x2 - x1));
	//if(bearing < 0) bearing += 2 * PI;
	bearing *= 180.0 / M_PI;
	return bearing;
}
/* makes conversion from radian to degree */
double radian2degree(double radianAngle){
	return (radianAngle*57.2957795);
}

void moveStraight(double speed, double distance, bool isForward){
	tf::TransformListener listener;
	tf::StampedTransform init_transform;
	tf::StampedTransform current_transform;

	//set the linear velocity to a positive value if isFoward is true
	if (isForward)
		base_cmd.linear.x =abs(speed);
	else //else set the velocity to negative value to move backward
		base_cmd.linear.x =-abs(speed);
	//all velocities of other axes must be zero.
	base_cmd.linear.y = base_cmd.linear.z =base_cmd.angular.x =base_cmd.angular.y =base_cmd.angular.z =0;

	double distance_moved = 0.0;
	ros::Rate loop_rate(10); // we publish the velocity at 10 Hz (10 times a second)

	//First, we capture the initial transformation before starting the motion.
	//It is important to "waitForTransform" otherwise, it might not be captured.
	try{
		//wait for the transform to be found
		listener.waitForTransform("/base_footprint", "/odom", ros::Time(0), ros::Duration(10.0) );
		//Once the transform is found,get the initial_transform transformation.
		listener.lookupTransform("/base_footprint", "/odom",ros::Time(0), init_transform);
	}
	catch (tf::TransformException & ex){
		ROS_ERROR(" Problem %s",ex.what());
		ros::Duration(1.0).sleep();
	}



	do{
		//STEP1. PUBLISH THE VELOCITY MESSAGE

		cmd_vel_pub.publish(base_cmd);
		ros::spinOnce();
		loop_rate.sleep();

		//STEP2. ESTIMATE THE DISTANCE MOVED BY THE ROBOT

		try{
			//wait for the transform to be found
			listener.waitForTransform("/base_footprint", "/odom", ros::Time(0), ros::Duration(10.0) );
			//Once the transform is found,get the initial_transform transformation.
			listener.lookupTransform("/base_footprint", "/odom",ros::Time(0), current_transform);
		}
		catch (tf::TransformException & ex){
			ROS_ERROR(" Problem %s",ex.what());
			ros::Duration(1.0).sleep();
		}
		 //Using transform composition. We calculate the relative transform, then we determine its length
		 //transform.getOrigin().length(): return the displacement of the origin of the transformation
		tf::Transform relative_transform = init_transform.inverse() * current_transform;
		distance_moved= relative_transform.getOrigin().length();

		//cout<<"Method 2: distance moved: "<<distance_moved <<", "<<distance<<endl;

	}while((distance_moved<distance)&&(ros::ok()));
	//finally, stop the robot when the distance is moved
	base_cmd.linear.x =0;
	cmd_vel_pub.publish(base_cmd);
}

double rotate(double angular_velocity, double radians,  bool clockwise)
{
	tf::TransformListener TFListener;
	tf::StampedTransform init_transform;
	tf::StampedTransform current_transform;

	double angle_turned =0.0;

	//validate angular velocity; ANGULAR_VELOCITY_MINIMUM_THRESHOLD is the minimum allowed
	angular_velocity=((angular_velocity>ANGULAR_VELOCITY_MINIMUM_THRESHOLD)?angular_velocity:ANGULAR_VELOCITY_MINIMUM_THRESHOLD);

	while(radians < 0) radians += 2*M_PI;
	while(radians > 2*M_PI) radians -= 2*M_PI;

	//wait for the listener to get the first message
	TFListener.waitForTransform("base_footprint", "odom", ros::Time(0), ros::Duration(1.0));

	//record the starting transform from the odometry to the base frame
	TFListener.lookupTransform("base_footprint", "odom", ros::Time(0), init_transform);

	//the command will be to turn at 0.75 rad/s
	base_cmd.linear.x = base_cmd.linear.y = 0.0;
	base_cmd.angular.z = angular_velocity;
	if (clockwise) base_cmd.angular.z = -base_cmd.angular.z;

	//the turn direction
	tf::Vector3 desired_turn_axis(0,0,1);
	if (!clockwise) desired_turn_axis = -desired_turn_axis;

	ros::Rate rate(10.0);
	bool done = false;
	while (!done )
	{
		//send the drive command
		cmd_vel_pub.publish(base_cmd);
		rate.sleep();
		//get the current transform
		try
		{
			TFListener.waitForTransform("base_footprint", "odom", ros::Time(0), ros::Duration(1.0));
			TFListener.lookupTransform("base_footprint", "odom", ros::Time(0), current_transform);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s",ex.what());
			break;
		}
		tf::Transform relative_transform = init_transform.inverse() * current_transform;
		tf::Vector3 actual_turn_axis = relative_transform.getRotation().getAxis();
		angle_turned = relative_transform.getRotation().getAngle();

		if (fabs(angle_turned) < 1.0e-2) continue;
		if (actual_turn_axis.dot(desired_turn_axis ) < 0 )
			angle_turned = 2 * M_PI - angle_turned;

		if (!clockwise)
			base_cmd.angular.z = (angular_velocity-ANGULAR_VELOCITY_MINIMUM_THRESHOLD) * (fabs(radian2degree(radians-angle_turned)/radian2degree(radians)))+ANGULAR_VELOCITY_MINIMUM_THRESHOLD;
		else
			if (clockwise)
				base_cmd.angular.z = (-angular_velocity+ANGULAR_VELOCITY_MINIMUM_THRESHOLD) * (fabs(radian2degree(radians-angle_turned)/radian2degree(radians)))-ANGULAR_VELOCITY_MINIMUM_THRESHOLD;

		if (angle_turned > radians) {
			done = true;
			base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
			cmd_vel_pub.publish(base_cmd);
		}


	}
	if (done) return angle_turned;
	return angle_turned;
}



void Foundmine(const geometry_msgs::PoseStamped MineMessage){
  std::cout << "Mine Found!(Mine number: " << MineMessage.header.frame_id <<" at : "  << MineMessage.pose.position.x << "," << MineMessage.pose.position.y << " )\n";
  //printf(("Mine Found! \nMine number = %s \nposition x = %.2f y = %.2f"), MineMessage.header.frame_id, MineMessage.pose.position.x,MineMessage.pose.position.y);
  moveStraight(0.0,0.0,true); //stops motion
  moveStraight(0.2,0.5,false); //moves 0.5m backwards
  rotate(0.4,M_PI/2,true); //Turns 90 degrees to the right
  moveStraight(0.2,0.3,true); //moves 0.3m forward THIS SHOULD BE CHANGED IF THE ROBOT STILL COLLIDES WITH THE MINES
  rotate(0.4,M_PI/2,false); //Turns 90 degrees to the left
  moveStraight(0.2,0.3,true); // moves 0.3m forward to dodge the mine
}
  public: //this is pretty much all the program is doing
    //MineReactor():
    ros::Subscriber FindingminesNode = n.subscribe("mineCounter",1000, &MineReactor::Foundmine, this);
    //~MineReactor(){};
};

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

int main(int argc, char **argv){ 
ros::init(argc, argv, "mineAvoidance");
while (ros::ok())
{
MineReactor running;
ros::spin();
}
return 0;
}
