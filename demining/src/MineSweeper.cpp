
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/Color.h>
#include <turtlesim/SetPen.h>
#include <iostream>
//#include <conio.h> //Not recommented but should be able to read if input is avaliable
#include "stdlib.h"
#include <stdio.h> // To input shit
#include <ctime>
#include "time.h"
#include <string>
#include <sstream>
#include <fstream> //Read/write on local files
#include "std_msgs/String.h" //SubScribe input from others
#include "kobuki_msgs/BumperEvent.h"




//Variable used :
int running=1;
const double PI = 3.14159265358979323846;
std::string line = "";        //Used to recive input at 'line'
std::string BumberDATA;         //
//std::string line.reserve(20); //reserve 20 stoage space for the string 'line' (Buffer)
int Current_Work = 002; //Variable used to check current work/
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
float CurrentMinePosX;
float CurrentMinePosY;
float minePosX[1000];
float minePosY[1000];

//Other bots locations
int BotLocation=1;
float BotX=1;
float BotY=1;



float distance(){      // distance to start position
    float numb = sqrt((StartX-currentPosX)*(StartX-currentPosX) + (StartX-currentPosX)*(StartX-currentPosX));

    return numb;
};     


void GoTo(float newX, float newY){ //Go to given location
    while(newX != currentPosX || newY != currentPosY){
        std::cout << "Current position is : " << currentPosX << "," << currentPosY << "\n";
        if(newX<currentPosX){
            if((currentPosX-newX)>=1){
                currentPosX=currentPosX-1;
            }
            else if((currentPosX-newX)>=0.1){
                currentPosX=currentPosX-0.1;
            }
            else if((currentPosX-newX)>=0.01){
                currentPosX=currentPosX-0.01;
            }

        }
        else if(newX>currentPosX){
            if((newX-currentPosX)>=1){
                currentPosX=currentPosX+1;
            }
            else if((newX-currentPosX)>=0.1){
                currentPosX=currentPosX+0.1;
            }
            else if((newX-currentPosX)>=0.01){
                currentPosX=currentPosX+0.01;
            }

        }
        if(newY<currentPosY){
            if((currentPosY-newY)>=1){
                currentPosY=currentPosY-1;
            }
            else if((currentPosY-newY)>=0.1){
                currentPosY=currentPosY-0.1;
            }
            else if((currentPosY-newY)>=0.01){
                currentPosY=currentPosY-0.01;
            }

        }
        else if(newY>currentPosY){
            if((newY-currentPosY)>=1){
                currentPosY=currentPosY+1;
            }
            else if((newY-currentPosY)>=0.1){
                currentPosY=currentPosY+0.1;
            }
            else if((newY-currentPosY)>=0.01){
                currentPosY=currentPosY+0.01;
            }

        }

    }
    std::cout << "Reached position! \n";
    std::cout << "Current position is : " << currentPosX << "," << currentPosY << "\n";

}



// Returns true if s is a number else false
bool isNumber(std::string s){
    for (int i = 0; i < s.length(); i++){
        if (isdigit(s[i]) == false){return false;}
        else {return true;}
    }
}

void Question_Input(std::string TalBogstav){
    if(TalBogstav=="Bogstav"){
        line="";
        while(line==""){
            ros::spinOnce();
        }
        std::cout << "String is : " << line << "\n";
    }
    else if(TalBogstav=="Tal"){
        bool taltest=false;
        while(taltest==false){
            line="";

            while(line==""){
                ros::spinOnce();
        
            }
            if(isNumber(line)==true){
                taltest=true;
            }
            else{
                std::cout <<"not a number! Try again\n";
            }

        }
        std::cout << "Number is : " << line << "\n";
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
if (text == "MineSweep" || text =="minesweep" || text =="demine" || text =="Demine"){
    return 404;
}
else if(text == "Retrive" || text =="retrive"){
    if(BotLocation){
        std::cout << "Found Bot at location : " << BotX << "," << BotY << "\n";
        std::cout << "Use this location? (y/n)" << "\n";
        Question_Input("Bogstav");
        if(line=="y"){
            std::cout << "Using : " << BotX << "," << BotY << " as location." << "\n";
        }
        else if(line=="n"){
            BotLocation=0;
            //std::cout << "Try again" << "\n";
            std::cout << "Please input X-Coordinate of bot : " << "\n";
            Question_Input("Tal");
            BotX = std::stof(line);
            std::cout << "Please input Y-Coordinate of bot : " << "\n";
            Question_Input("Tal");
            BotY = std::stof(line);
            std::cout << "Using : " << BotX << "," << BotY << " as location." << "\n";
            line ="";
        }
        else{
            std::cout <<"Wrong Input!\n";
        }
    }
    else{
        std::cout << "Please input X-Coordinate of bot : " << "\n";
        Question_Input("Tal");
        BotX = std::stof(line);
        std::cout << "Please input Y-Coordinate of bot : " << "\n";
        Question_Input("Tal");
        BotY = std::stof(line);
        std::cout << "Using : " << BotX << "," << BotY << " as location." << "\n";
        line ="";
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
else if(text == ""){
    return Current_Work;
}
else if(text == "help" || text == "Help"){                               //Print out input options
    std::cout << "Avaliable commands is :" << "\n";
    std::cout << "Scan Area" << "\n";
    std::cout << "MineSweep" << "\n";
    std::cout << "Retrive" << "\n";
    std::cout << "Return" << "\n";
    std::cout << "Low Battery" << "\n";
    std::cout << "Manual" << "\n";
    std::cout << "STOP" << "\n";
    std::cout << "Shut down" << "\n\n";
    return Current_Work;
}
else{                               //Print out input options if wrong
    std::cout << "                                      !!!WRONG INPUT!!!." << "\n";
    std::cout << "Try one of following commands :" << "\n";
    std::cout << "Scan Area" << "\n";
    std::cout << "MineSweep" << "\n";
    std::cout << "Retrive" << "\n";
    std::cout << "Return" << "\n";
    std::cout << "Low Battery" << "\n";
    std::cout << "Manual" << "\n";
    std::cout << "STOP" << "\n";
    std::cout << "Shut down" << "\n\n";
    return Current_Work;
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
        //std::cout << "Changed work to " << NameTag(Current_Work) << "\n";
    }
    else{
        //std::cout << "Same Work. Try again \n";
        line = "";
    }
    

}

void StopAll(){                     //Function to stop all actions
    std::cout << "Stopping All Actions and Movements" << "\n";

//cmd_vel_pub_.publish(base_cmd);

}

void SaveData(){                    //Function to save data locally on a file
    int counter=1;
    while(counter){
        std::cout << "Saving Data" << "\n";
        myfile.open ("The Program that dosn't work.txt", std::fstream::app);
        if(myfile.is_open()){
            time_t timeNow = time(0);
            myfile << "                         Saving from this run. Date/time : " << ctime(&timeNow) << "\n";
            if(FoundMines!=0){
                myfile << "Found mines: " << FoundMines << "\n";
                myfile << "     Mine(s) is located at : \n";
                    for(int i=1; i<=FoundMines;i++){
                        myfile << "Mine " << i << " at : " << minePosX[i] << "," << minePosY[i] << "\n\n";
                    }
            }
            else{ myfile << "No Mines Found \n";}
            myfile << "Area size:   " << AreaLenght << " X " << AreaWidth << " equals to : " << AreaLenght*AreaWidth << "m²" << "\n";
            myfile << "\n";
            myfile << "you Launched MineSweeper an got this!\n\n\n\n\n";
            myfile.close();
            counter=0;
        }
        else if(counter>=20){
            std::cout << "ERROR - Creating file..." << "\n";
            myfile.open("The Program that dosn't work.txt", std::fstream::out);
            counter=1;
        }
        else{
           std::cout << "no connection to file" << "\n";
            counter++;
        }
        
    }
}



/*-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
//Subscriber inputez

void chatterCallBack(const std_msgs::String::ConstPtr& msg){
    //ROS_INFO("I heard this : [%s]", msg->data.c_str());
    if(msg->data.c_str()!=line){
    line = msg->data.c_str();
        //std::cout <<"line is equal to : " << line << "\n";
    }
}

void mineCounterCallBack(const geometry_msgs::PoseStamped MineMessage){
    CurrentMinePosX = MineMessage.pose.position.x;
    CurrentMinePosY = MineMessage.pose.position.y;
    FoundMines++;
    minePosX[FoundMines] = CurrentMinePosX;
    minePosY[FoundMines] = CurrentMinePosY;
    std::cout << "Got a Mine!\n";
    std::cout << "Mine " << FoundMines << " at : " << minePosX[FoundMines] << "," << minePosY[FoundMines] << "\n\n";

}





/*-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
//Here starts the programme

int main(int argc, char *argv[]){
ros::init(argc,argv, "MineUI");

ros::NodeHandle n;
ros::Rate rate(10); //the larger the value, the "smoother" , try value of 1 to see "jerk" movement

//Publishers
//ros::Publisher
  //init publisher
  ros::Publisher cmd_vel_pub_ = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

//subscribers
ros::Subscriber UserInputSub = n.subscribe("chatter", 1000, chatterCallBack);
ros::Subscriber GetMinePos = n.subscribe("mineCounter",1000, mineCounterCallBack);
//ros::Subscriber BumberROBSub = n.subscribe("/mobile_base/events/bumper", 1000, BumberROBSubCallBack);
//ros::Subscriber

  //init direction that turtlebot should go
  geometry_msgs::Twist base_cmd;

  base_cmd.linear.x = 0;                            //Stopping
  base_cmd.linear.y = 0;
  base_cmd.angular.z = 0;

/*--------------------------------------------------LOOP--------------------------------------------------*/
while(ros::ok() && running){



 //---------------------------- NEW INPUT
ros::spinOnce();
ChangeWork(input(line));

//ROS_INFO("Running case: [%i]", Current_Work);

switch (Current_Work){      //Switch to change program funktion
                            //(Scan_area / Sweep the area Etc.)
case 101: //Scan Area
    ros::spinOnce();
    if(AreaLenght || AreaWidth){
    std::cout << "Pre-known Lenght and width found!" << "\n";
    std::cout << "Lenght is : " << AreaLenght << "\n";
    std::cout << "Width is  : " << AreaWidth << "\n";
    std::cout << "Will you use these sizes? (y/n)" << "\n";
    Question_Input("Bogstav");
        if(line=="y"){
            std::cout << "Using : " << AreaLenght << " and " << AreaWidth << " as Lenght and Width." << "\n";
        }
        else if(line=="n"){
            std::cout << "Do you know the sizes of the Area? (y/n)" << "\n";
            Question_Input("Bogstav");
            if(line=="y"){
                std::cout << "Please input Lenght of Area : " << "\n";
                Question_Input("Tal");
                AreaLenght=std::stof(line);
                std::cout << "Please input Width og Area : " << "\n";
                Question_Input("Tal");
                AreaWidth=std::stof(line);
                std::cout << "Using : " << AreaLenght << " and " << AreaWidth << " as Lenght and Width." << "\n";
                line="";
            }
            else if(line=="n"){
                line="";
                std::cout << "Finding Lenght and Width for you \n";
                //MISSING --> TEMP BUFFER INPUT INSTEAD UNTIL FIXED
                ChangeWork(101);
            }
            else {
                std::cout << "Try again" << "\n";
            }
        }
        else if(line!="y" || line!="n"){
            std::cout << "Try again" << "\n";
        }

    
    }
    std::cout << "Scanning Area..." << "\n";

    if(AreaLenght || AreaWidth){
    std::cout << "Area Scanned" << "\n";
    std::cout << "Lenght is : " << AreaLenght << "\n";
    std::cout << "Width is  : " << AreaWidth << "\n";
        ChangeWork(404);
    }
    break;

case 404: //MineSweep
    ros::spinOnce();
    ros::Duration(1).sleep();
    std::cout << "demining..." << "\n";
    
    break;

case 909: //RetrivOtherBot
    ros::spinOnce();
    StopAll();
    GoTo(BotX, BotY);
    if(BotX==currentPosX && BotY==currentPosY){
        std::cout << "Arrived to Bot" << "\n";
        ChangeWork(002); //Changing to Stop
    }
    break;

case 808: //Return
    ros::spinOnce();
    StopAll();
    GoTo(StartX, StartY); //Go to starting position
    if(StartX==currentPosX && StartY==currentPosY){
        std::cout << "Returned to Start" << "\n";
        ChangeWork(002); //Changing to Stop
    }

    break;

case 001: //LOW BATTERY
    ros::spinOnce();
    StopAll();
    std::cout << "Low battery! \n Returning" << "\n";
    for(int i=1; i<=3; i++){ros::Duration(0.5).sleep(); std::cout << ".";} //Niceness --> counting to 1.5 sec (as delay)
    SaveData();
    ChangeWork(808); //Changing to Return

    break;

case 505: //Manual Takeover
    ros::spinOnce();
    std::cout << "Manual takeover... (Not made yet)" << "\n";
    ros::Duration(1).sleep();
    
    break;

case 002: //STOP
    ros::spinOnce();
    StopAll();
    std::cout << "Stopped" << "\n\n\n";
    std::cout << "Waiting for instructions : ('Help' for Avaliable commands)" << "\n";
    line = "";
    while(Current_Work == 002){
        ros::spinOnce();
        //StopAll();
        cmd_vel_pub_.publish(base_cmd);
        //std::cout << "STOP - Waiting position :-(" << "\n";
        //std::cout << "Current Work is: " << Current_Work << "\n";
        if(line!= ""){
            std::cout <<"you entered : " << line << "\n";
            ChangeWork(input(line));
        }
    }

    break;

case 707: //Shut down
    ros::spinOnce();
    StopAll();
    SaveData();
    std::cout << "Shutting down";
        //for(int i=1; i<=3; ++i){ros::Duration(1.5).sleep(); std::cout << ".";} //Niceness --> counting to 1.5 sec (as delay)
    std::cout <<"\n";
    running=0;
    std::terminate;
    break;

default: //FEJL
    ros::spinOnce();
    std::cout << "                         !!!ERROR!!!" << "\n";
    break;
}

}
    return 0;
}











