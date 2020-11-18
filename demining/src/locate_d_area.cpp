#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/Sound.h>
#include <tf/transform_listener.h>

kobuki_msgs::BumperEvent bumpMsg;
kobuki_msgs::Sound soundMsg;
ros::Publisher sound_pub;
ros::Subscriber bumperSub;
tf::TransformListener listener;
tf::StampedTransform transform;

void bumperHit(const kobuki_msgs::BumperEvent &bumpMsg){
    if(bumpMsg.state==1){
        soundMsg.value = 4;
        sound_pub.publish(soundMsg);
        //skal slukke for lyden igen ellers køre den nogen gange i loop
        soundMsg.value = 1;
        sound_pub.publish(soundMsg);
        std::cout << "y = " << transform.getOrigin().y() << "\n x = " << transform.getOrigin().x() << "\n";
    }

}

int main(int argc, char *argv[])
{
   ros::init(argc,argv,"bumper");
    srand(time(NULL));
    ros::NodeHandle n;
    sound_pub=n.advertise<kobuki_msgs::Sound > ("/mobile_base/commands/sound", 2);

   // listener.waitForTransform("/move_base/local_costmap/global_frame", "/move_base/local_costmap/robot_base_frame", ros::Time::now(), ros::Duration(2.0));
   // listener.lookupTransform("/move_base/local_costmap/global_frame", "/move_base/local_costmap/robot_base_frame", ros::Time::now(), transform);

    bumperSub=n.subscribe("/mobile_base/events/bumper",1,bumperHit);
while(ros::ok())
{
  
    ros::spinOnce();
}


    return 0;
}
