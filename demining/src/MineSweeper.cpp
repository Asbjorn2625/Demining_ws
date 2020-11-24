
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
        std::cout << "Found Bot at location : " << BotX << "," << BotY << "\n";
        std::cout << "Use this location? (y/n)" << "\n";
        std::getline(std::cin, line);
        if(line=="y"){
            std::cout << "Using : " << BotX << "," << BotY << " as location." << "\n";
        }
        else if(line!="y" || line!="n"){
            std::cout << "Try again" << "\n";
        }
    }
    else{
        std::cout << "Please input X-Coordinate of bot : " << "\n";
        std::cin >> BotX;
        std::cout << "Please input Y-Coordinate of bot : " << "\n";
        std::cin >> BotY;
        std::cout << "Using : " << BotX << "," << BotY << " as location." << "\n";
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
    std::cout << "Wrong input." << "\n";
    std::cout << "Try one of following commands :" << "\n";
    std::cout << "Scan Area" << "\n";
    std::cout << "MineSweep" << "\n";
    std::cout << "Retrive" << "\n";
    std::cout << "Return" << "\n";
    std::cout << "Low Battery" << "\n";
    std::cout << "Manual" << "\n";
    std::cout << "STOP" << "\n";
    std::cout << "Shut down" << "\n";
}

}

std::string NameTag(int Worknumb){  // Print out name of function from function number
         if(Worknumb==404){return "MineSweep";}
    else if(Worknumb==909){return "Retrive";}
    else if(Worknumb==808){return "Return";}
    else if(Worknumb==001){return "Low Battery";}
    else if(Worknumb==505){return "Manual";}
    else if(Worknumb==101){return "Scan Area";}
    else if(Worknumb==002){return "STOP";}
    else if(Worknumb==707){return "Shut Down";}

}



void ChangeWork(int ChangeAble){    //Function to change work
    if(ChangeAble != Current_Work){
        Last_Work = Current_Work;
        Current_Work = ChangeAble;
        std::cout << "Changed work to " << NameTag(Current_Work) << "\n";
    }

}

void StopAll(){                     //Function to stop all actions
    std::cout << "Stopping All Actions and Movements" << "\n";


}

void SaveData(){                    //Function to save data locally on a file
    int counter=1;
    while(counter){
        std::cout << "Saving Data" << "\n";
        myfile.open ("example.txt");
        if(myfile.is_open()){
            //myfile << "Saving from this run. Date/time : " << ros::Time::now() << "\n";
            myfile << "Found mines: " << FoundMines << "\n";
            myfile << "Area size:   " << AreaLenght << " X " << AreaWidth << "equals to : " << AreaLenght*AreaWidth << "m²" << "\n";
            myfile << "\n \n " << "\n";
            myfile.close();
            counter=0;
        }
        else if(counter>=20){
            std::cout << "ERROR - Creating file..." << "\n";
            myfile.open("example.txt", std::fstream::out);
            counter=1;
        }
        else{
           std::cout << "no connection to file" << "\n";
            counter++;
        }
        
    }
}







/*-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
//Here starts the programme

int main(int argc, char *argv[]){
ros::init(argc,argv, "MineUI");

ros::NodeHandle n;
ros::Rate rate(10); //the larger the value, the "smoother" , try value of 1 to see "jerk" movement

//Publishers
//ros::Publisher
//subscribers
//ros::Subscriber


/*--------------------------------------------------LOOP--------------------------------------------------*/
while(ros::ok()){



 //---------------------------- NEW INPUT
//if( !std::getline(std::cin, line) ){ Virker ikke, næste linje er en test
/*if( !std::getline(std::cin,line)){
    std::cout << "You entered " << line << "\n";
    if(input(line) != Current_Work){
        Last_Work = Current_Work;
        Current_Work = input(line);
        rate.sleep();
    }
}*/






switch (Current_Work){      //Switch to change program funktion
                            //(Scan_area / Sweep the area Etc.)
case 101: //Scan Area
    std::cout << "Scanning Area" << "\n";

    if(AreaLenght || AreaWidth){
    std::cout << "Area Scanned" << "\n";
    std::cout << "Lenght is : " << AreaLenght << "\n";
    std::cout << "Width is  : " << AreaWidth << "\n";
        ChangeWork(404);
    }
    break;

case 404: //MineSweep

    break;

case 909: //RetrivOtherBot
    StopAll();
    GoTo(BotX, BotY);
    if(BotX==currentPosX && BotY==currentPosY){
        std::cout << "Arrived to Bot" << "\n";
        ChangeWork(002); //Changing to Stop
    }

    break;

case 808: //Return
    StopAll();
    GoTo(StartX, StartY); //Go to starting position
    if(StartX==currentPosX && StartY==currentPosY){
        std::cout << "Returned to Start" << "\n";
        ChangeWork(002); //Changing to Stop
    }

    break;

case 001: //LOW BATTERY
    StopAll();
    std::cout << "Low battery! \n Returning" << "\n";
    for(int i=1; i<=3; i++){ros::Duration(0.5).sleep(); std::cout << ".";} //Niceness --> counting to 1.5 sec (as delay)
    SaveData();
    ChangeWork(808); //Changing to Return

    break;

case 505: //Manual Takeover

    break;

case 002: //STOP
    StopAll();
    std::cout << "Stopped" << "\n";
    std::cout << "Waiting for instructions" << "\n";    
    while(Current_Work = 002){
        if( !std::getline(std::cin, line) ){    //Stopping all actions and waiting for indput
            std::cout << "You entered " << line << "\n";
            ChangeWork(input(line));
        }
    }

    break;

case 707: //Shut down
    StopAll();
    SaveData();
    running=0;
    std::cout << "Shutting down..." << "\n";
    break;

default: //FEJL
    std::cout << "ERROR" << "\n";
    break;
}







}
    return 0;
}







//if(Current_Work != Last_Work){}

/*
//Links for help:


- Move a certain distance, turn, then move (Odometry topic)
        https://answers.ros.org/question/205132/move-a-certain-distance-turn-then-move-odometry-topic/?answer=269733

*/ 