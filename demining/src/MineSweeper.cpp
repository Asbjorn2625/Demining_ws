
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/Color.h>
#include <turtlesim/SetPen.h>
#include <iostream>
#include "stdlib.h"
#include "time.h"
#include <string>
#include <sstream>
#include <fstream> //Read/write on local files


ros::Rate rate(10); //the larger the value, the "smoother" , try value of 1 to see "jerk" movement

//Variable used :
int running=1;
const double PI = 3.14159265358979323846;
std::string line = "";        //Used to recive input at 'line'
//std::string line.reserve(20); //reserve 20 stoage space for the string 'line'
int Current_Work = 101; //Variable used to check current work/
int Last_Work = 0; //Variable used to check last work
float AreaLenght = 1;   //Given area lenght
float AreaWidth = 1;    //given area width
float StartX = 0.0;     //Start position
float StartY = 0.0;     //Start position
float currentPosX = 0.0;//Current position
float currentPosY = 0.0;//Current position
//Color sensors
bool first_color_saved = false;
turtlesim::Color first_color;
//Saving data
std::ofstream myfile;


int FoundMines = 0;     //Number of found mines

//Other bots locations
int BotLocation=0;
float BotX=0;
float BotY=0;

/*
//CONNECTION --> 
//Publishers
ros::Publisher cmd_vel_pub;
ros::Publisher movement_pub;
//Subscribers
*/

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


/*--------------------------------------------------LOOP--------------------------------------------------*/
while(running){



 //---------------------------- NEW INPUT
//if( !std::getline(std::cin, line) ){ Virker ikke, næste linje er en test
if( !std::getline(std::cin,line)){
    std::cout << "You entered " << line << std::endl;
    if(input(line) != Current_Work){
        Last_Work = Current_Work;
        Current_Work = input(line);
        rate.sleep();
    }
}






switch (Current_Work){      //Switch to change program funktion
                            //(Scan_area / Sweep the area Etc.)
case 101: //Scan Area
    std::cout << "Scanning Area" << std::endl;

    if(AreaLenght || AreaWidth){
    std::cout << "Area Scanned" << std::endl;
    std::cout << "Lenght is : " << AreaLenght << std::endl;
    std::cout << "Width is  : " << AreaWidth << std::endl;
    return 404;
    }
    break;

case 404: //MineSweep

    break;

case 909: //RetrivOtherBot
    StopAll();
    GoTo(BotX, BotY);
    if(BotX==currentPosX && BotY==currentPosY){
        std::cout << "Arrived to Bot" << std::endl;
        ChangeWork(002); //Changing to Stop
    }

    break;

case 808: //Return
    StopAll();
    GoTo(StartX, StartY); //Go to starting position
    if(StartX==currentPosX && StartY==currentPosY){
        std::cout << "Returned to Start" << std::endl;
        ChangeWork(002); //Changing to Stop
    }

    break;

case 001: //LOW BATTERY
    StopAll();
    std::cout << "Low battery! \n Returning" << std::endl;
    for(int i=1; i<=3; i++){ros::Duration(0.5).sleep(); std::cout << ".";} //Niceness --> counting to 1.5 sec (as delay)
    SaveData();
    ChangeWork(808); //Changing to Return

    break;

case 505: //Manual Takeover

    break;

case 002: //STOP
    StopAll();
    std::cout << "Stopped" << std::endl;
    std::cout << "Waiting for instructions" << std::endl;    
    while(Current_Work = 002){
        if( !std::getline(std::cin, line) ){    //Stopping all actions and waiting for indput
            std::cout << "You entered " << line << std::endl;
            ChangeWork(input(line));
        }
    }

    break;

case 707: //Shut down
    StopAll();
    SaveData();
    running=0;
    std::cout << "Shutting down..." << std::endl;
    break;

default: //FEJL
    std::cout << "ERROR" << std::endl;
    break;
}






ros::Rate loop_rate(10);
while(ros::ok()){
    cmd_vel_pub.publish(cmd_vel_message);
    loop_rate.sleep();
    //python kobuki_battery.py // Kan bruges til at læse batteri - niveauet 
    
    std::cout << "battery level is : " << std::endl;
    }


}
    return 0;
}




/*-------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
//Functionscall :




/*
void Forward(){//moves forward
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


void TurnRight(){//turns right
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

void TurnLeft(){//turns left
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
*/

float distance(){      // distance to start position
    float numb = sqrt((StartX-currentPosX)*(StartX-currentPosX) + (StartX-currentPosX)*(StartX-currentPosX));

    return numb;
};     


void GoTo(float newX, float newY){ //Go to given location
    if(newX != currentPosX || newY != currentPosY){


    }

}



/*
void Sensor1(){                 //Used to recive data from Sensor1
 if(!first_color_saved){
    first_color = sensed_color;
    first_color_saved = true;
  }

}
*/


int input(std::string text){    //Will compare input with sub-function names and return sub-function number
if (text == "MineSweep" || text =="minesweep"){
    return 404;
}
else if(text == "Retrive" || text =="retrive"){
    if(BotLocation){
        std::cout << "Found Bot at location : " << BotX << "," << BotY << std::endl;
        std::cout << "Use this location? (y/n)" << std::endl;
        std::getline(std::cin, line);
        if(line=="y"){
            std::cout << "Using : " << BotX << "," << BotY << " as location." << std::endl;
        }
        else if(line!="y" || line!="n"){
            std::cout << "Try again" << std::endl;
        }
    }
    else{
        std::cout << "Please input X-Coordinate of bot : " << std::endl;
        std::cin >> BotX;
        std::cout << "Please input Y-Coordinate of bot : " << std::endl;
        std::cin >> BotY;
        std::cout << "Using : " << BotX << "," << BotY << " as location." << std::endl;
    }

    return 909;
}
else if(text == "Return" || text =="return"){
    return 808;
}
else if(text == "Low Battery" || text =="low battery"){
    return 001;
}
else if(text == "Manual" || text =="manual"){
    return 505;
}
else if(text == "Scan Area" || text =="scan area"){
    return 101;
}
else if(text == "STOP" || text =="stop" || text =="Stop"){
    return 002;
}
else if(text == "Shut down" || text == "shut down"){
    return 707;
}
else{                               //Print out input options if wrong
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

void ChangeWork(int ChangeAble){    //Function to change work
    if(ChangeAble != Current_Work){
        Last_Work = Current_Work;
        Current_Work = ChangeAble;
        std::cout << "Changed work to " << NameTag(Current_Work);
        rate.sleep();
    }

}

void StopAll(){                     //Function to stop all actions
    std::cout << "Stopping All Actions and Movements" << std::endl;


}

void SaveData(){                    //Function to save data locally on a file
    int counter=1;
    while(counter){
        std::cout << "Saving Data" << std::endl;
        myfile.open ("example.txt");
        if(myfile.is_open()){
            myfile << "Saving from this run. Date/time : " << ros::Time::now() << std::endl;
            myfile << "Found mines: " << FoundMines << std::endl;
            myfile << "Area size:   " << AreaLenght << " X " << AreaWidth << "equals to : " << AreaLenght*AreaWidth << "m²" << std::endl;
            myfile << "\n \n " << std::endl;
            myfile.close();
            counter=0;
        }
        else if(counter>=20){
            std::cout << "ERROR - Creating file..." << std::endl;
            myfile.open("example.txt", std::fstream::out);
            counter=1;
        }
        else{
           std::cout << "no connection to file" << std::endl;
            counter++;
        }
        
    }
}



std::string NameTag(int Worknumb){  // Print out name of function from function number
    if(Worknumb == 404) {return "MineSweep";}
    else if(Worknumb==909){return "Retrive";}
    else if(Worknumb==808){return "Return";}
    else if(Worknumb==001){return "Low Battery";}
    else if(Worknumb==505){return "Manual";}
    else if(Worknumb==101){return "Scan Area";}
    else if(Worknumb==002){return "STOP";}
    else if(Worknumb==707){return "Shut Down";}

}




//if(Current_Work != Last_Work){}

/*
//Links for help:


- Move a certain distance, turn, then move (Odometry topic)
        https://answers.ros.org/question/205132/move-a-certain-distance-turn-then-move-odometry-topic/?answer=269733

*/ 