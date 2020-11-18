#include <iostream>
#include <ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<kobuki_msgs/BumperEvent.h>
#include<kobuki_msgs/Sound.h>

kobuki_msgs::BumperEvent bumpMsg;
kobuki_msgs::Sound soundMsg;
ros::Publisher sound_pub;
ros::Subscriber bumperSub;
//kobuki_msgs::Sound soundPub;


void callbackBump(const kobuki_msgs::BumperEvent &bumpMsg){
    if(bumpMsg.state==1){
        soundMsg.value = 4;
        sound_pub.publish(soundMsg);
        std::cout<<"\n fy faaaaaeeen \n";
        //skal slukke for lyden igen ellers køre den nogen gange i loop
        soundMsg.value = 1;
        sound_pub.publish(soundMsg);
    }

}

int main(int argc, char *argv[])
{
   ros::init(argc,argv,"bumper");
    srand(time(NULL));
    ros::NodeHandle n;
    sound_pub=n.advertise<kobuki_msgs::Sound > ("/mobile_base/commands/sound",-1);
    bumperSub=n.subscribe("/mobile_base/events/bumper",1,callbackBump) ;
while(ros::ok())
{
  
    ros::spinOnce();
}


    return 0;
}
