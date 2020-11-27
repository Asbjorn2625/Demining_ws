#include <ros/ros.h>
#include <iostream>

#include <sensor_msgs/LaserScan.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "laser_scan_publisher");

  ros::NodeHandle n;
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 1000);

  unsigned int num_readings = 100;
  double laser_frequency = 40;
  double ranges[num_readings];
  double intensities[num_readings];

  int count = 0;
  ros::Rate r(1.0);
  while(n.ok()){
    /*
    //generate some fake data for our laser scan
    for(unsigned int i = 0; i < num_readings; i = i + 1){
      if(i = 5;)
      ranges[i] = count;
      intensities[i] = 100 + count;
    }*/
    
    
    
    
/*
float32 angle_min        # start angle of the scan [rad]
float32 angle_max        # end angle of the scan [rad]
float32 angle_increment  # angular distance between measurements [rad]
float32 time_increment   # time between measurements [seconds]
float32 scan_time        # time between scans [seconds]
float32 range_min        # minimum range value [m]
float32 range_max        # maximum range value [m]
float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
float32[] intensities    # intensity data [device-specific units]
*/


    // ranges[10] = count;

    ros::Time scan_time = ros::Time::now();

    //populate the LaserScan message
    sensor_msgs::LaserScan scan;
    scan.header.stamp = scan_time;
    scan.header.frame_id = "map";
    scan.angle_min = -1.57; //
    scan.angle_max = 1.57; // 
    scan.angle_increment = 3.14 * 2 / num_readings;
    scan.time_increment = (1 / laser_frequency) / (num_readings);
    scan.range_min = 0.0;
    scan.range_max = 10.0;

    scan.ranges.resize(num_readings);
    scan.intensities.resize(num_readings);
     for(unsigned int i = 0; i < num_readings; ++i){
      scan.ranges[i] = ranges[i];
      scan.intensities[i] = intensities[i];
      
    }

    scan_pub.publish(scan);
    ++count;
    r.sleep();
  }
}

//Find mere her: http://library.isr.ist.utl.pt/docs/roswiki/navigation(2f)Tutorials(2f)RobotSetup(2f)Sensors.html
//For at se det i rviz, skal du: køre projektet, køre roscore, og åbne rviz. 
//I Rviz skal du skrive "laser_frame"(måske "map") i stedet for map.
//Add LaserScan og i dens topic skrives der "/scan".
//Og til sidst ændres Size (m) til "0.2".
