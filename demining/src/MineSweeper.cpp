
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/Color.h>
#include <turtlesim/SetPen.h>
#include <iostream>
#include "stdlib.h"
#include "time.h"
#include <conio.h> //used with getch() for input


ros::Rate rate(10); //the larger the value, the "smoother" , try value of 1 to see "jerk" movement

//Variable used :
const double PI = 3.14159265358979323846;
char Text = "";        //Used to recive input
int Current_Work = 101; //Variable used to check current work/
int Last_Work = 0; //Variable used to check last work
float AreaLenght = 0;   //Given area lenght
float AreaWidth = 0;    //given area width
float StartX = 0.0;     //Start position
float StartY = 0.0;     //Start position
float currentPosX = 0.0;//Current position
float currentPosY = 0.0;//Current position
//Color sensors
bool first_color_saved = false;
turtlesim::Color first_color;


int FoundMines = 0;     //Number of found mines



//CONNECTION --> 
//Publisher
ros::Publisher cmd_vel_pub;
//Subscriber


//roslaunch turtlebot_bringup minimal.launch


/*-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
//Here starts the programme

int main(int argc, char *argv[]){
ros::init(argc,argv, "roundround");

ros::NodeHandle n;
ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);

geometry_msgs::Twist cmd_vel_message;
cmd_vel_message.angular.z=1.0;
cmd_vel_message.linear.x=1.0;

 ----------------------------// NEW INPUT
std::string line;
if( !std::getline(std::cin, line) ){
    cout << "You entered " + line << endl;
    if(input(line) != Last_Work){
        Last_Work = Current_Work;
        Current_Work = input(line);


    }
    
}






switch (Current_Work){       //Switch to change program funktion
                            //(Scan_area / Sweep the area Etc.)
case 101: //Scan Area
    






    break;

case 404: //MineSweep

    break;

case 909: //RetrivOtherBot

    break;

case 808: //Return



    break;

case 001: //LOW BATTERY

    break;

case 505: //Manual Takeover

break;

case 002: //STOP

    break;

case 707: //Shut down
return 0;
break;

default:
    break;
}






ros::Rate loop_rate(10);
while(ros::ok()){
    cmd_vel_pub.publish(cmd_vel_message);
    loop_rate.sleep();
//python kobuki_battery.py // Kan bruges til at læse batteri - niveauet 

std::cout << "battery level is : " << std::endl;
}

    return 0;
}




/*-------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
//Functionscall :

void Forward(){//move forward
ros::Time start = ros::Time::now();
while(ros::Time::now() - start < ros::Duration(5.0)){
    geometry_msgs::Twist move;
    //velocity controls
    move.linear.x = 0.1; //speed value m/s
    move.angular.z = 0;
    movement_pub.publish(move);

    ros::spinOnce();
    rate.sleep();
}
}


void TurnRight(){//turn right
ros::Time start_turn = ros::Time::now();
while(ros::Time::now() - start_turn < ros::Duration(4.0)){
    geometry_msgs::Twist move;
    //velocity controls
    move.linear.x = 0; //speed value m/s
    move.angular.z = -2.25;
    movement_pub.publish(move);

    ros::spinOnce();
    rate.sleep();
}
}

void TurnLeft(){//turn left
ros::Time start_turn = ros::Time::now();
while(ros::Time::now() - start_turn < ros::Duration(4.0)){
    geometry_msgs::Twist move;
    //velocity controls
    move.linear.x = 0; //speed value m/s
    move.angular.z = 2.25;
    movement_pub.publish(move);

    ros::spinOnce();
    rate.sleep();
}
}

float distance(){      // distance to start position
float numb = sqrt((StartX-currentPosX)*(StartX-currentPosX) + (StartX-currentPosX)*(StartX-currentPosX));

return numb;
};     


void GoTo(float newX, float newY){ //Go to given location


if(newX != currentPosX || newY != currentPosY){


}


}




void Sensor1()
 if(!first_color_saved)
  {
    first_color = sensed_color;
    first_color_saved = true;
  }

}



void input(char text){
if (text= "MineSweep"){
    return 404;
}
else if(text= "Retrive"){
    return 909;
}
else if(text= "Return"){
    return 808;
}
else if(text= "Low Battery"){
    return 001;
}
else if(text= "Manual"){
    return 505;
}
else if(text= "Scan Area"){
    return 101;
}
else if(text= "STOP"){
    return 002;
}
else if(text= "Shut down"){
    return 707;
}
else
{
    std::cout << "Wrong input." << std::endl;
    std::cout << "Try one of following commands :" << std::endl;
    std::cout << "Scan Area" << std::endl;
    std::cout << "MineSweep" << std::endl;
    std::cout << "Retrive" << std::endl;
    std::cout << "Return" << std::endl;
    std::cout << "Low Battery" << std::endl;
    std::cout << "Manual" << std::endl;
    std::cout << "STOP" << std::endl;
    std::cout << "Shut down" << std::endl;
}


}

if(Current_Work != Last_Work){
    


}

/*
//Links for help:


- Move a certain distance, turn, then move (Odometry topic)
        https://answers.ros.org/question/205132/move-a-certain-distance-turn-then-move-odometry-topic/?answer=269733

*/