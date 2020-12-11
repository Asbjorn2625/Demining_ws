#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <kobuki_msgs/Sound.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <fstream> //Read/write on local files


cv::Mat HSVImage;
cv::Mat ThreshImage;
cv::Mat Blurimage;
cv::Mat Edgedimage;
cv::Point P1;
cv::Point P2;
cv::Point P3;
cv::Point P4;
cv::Point P5;
cv::Point P6;
cv::Point P7;
cv::Point P8;
cv::Scalar red(4, 0, 255);
cv::Scalar NavyBlue(237, 27, 36);

static const std::string OPENCV_WINDOW = "Image window";


//Defining Global Variables
int mineCounter = 0;
int markerId;
ros::Time currentTimer;
int firstRun = 0;

float minePosX[1000];   //Mine number via 'mineCounter' and its X-coordinate
float minePosY[1000];   //Mine number via 'mineCounter' and its Y-coordinate
//Saving data
std::ofstream myfile; //Used to save a file (this is the same as 'nh_' when using Sub or Pub)


class ImageConverter
{
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_; 
  image_transport::Publisher image_pub_;
  tf::TransformListener listener;
  kobuki_msgs::Sound soundMSG;
  ros::Publisher sound_pub =nh_.advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound", 2);
  ros::Publisher marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("mine_markers", 1);
  ros::Publisher mine_pub = nh_.advertise<geometry_msgs::PoseStamped>("mineCounter",1);
  geometry_msgs::PoseStamped mineMessage,pBase, pMap;


public:

double getMinePosition(double mapPose[]){
ros::spinOnce();

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

double siny_cosp = 2 * (pMap.pose.orientation.w * pMap.pose.orientation.z);
double cosy_cosp = 1 - 2 * (pMap.pose.orientation.z * pMap.pose.orientation.z);
double radians = std::atan2(siny_cosp, cosy_cosp);

mapPose[0] = pMap.pose.position.x+0.5*cos(radians);
mapPose[1] = pMap.pose.position.y+0.5*sin(radians);
}


void SaveData(){                    //Function to save data locally on a file
    int counter=1;
    while(counter){
        myfile.open ("demining_project/Mine_Pictures/minelog.txt");
        if(myfile.is_open()){
            time_t timeNow = time(0);
            myfile << "                         Saving from this run. Date/time : " << ctime(&timeNow) << "\n";
            if(mineCounter!=0){
                myfile << "Found mines: " << mineCounter << "\n";
                myfile << "     Mine(s) is located at : \n";
                    for(int i=1; i<=mineCounter;i++){
                        myfile << "Mine " << i << " at : " << minePosX[i] << "," << minePosY[i] << "\n\n";
                    }
            }
            else{ myfile << "No Mines Found \n";}
            myfile.close();
            counter=0;
        }
        else if(counter>=20){
            std::cout << "ERROR - Creating file..." << "\n";
            myfile.open("demining_project/Mine_Pictures/minelog.txt", std::fstream::out);
            counter=1;
        }
        else{
           std::cout << "no connection to file" << "\n";
            counter++;
            return;
        }
        
    }
}



  void setPointMap(double posX, double posY, double size, double height, uint32_t shape)
  {
    //visualization_msgs::Marker::CYLINDER
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray marker_array;
    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "mine_pointers";
    if(firstRun = 0){
    int markerId = 0;
    }
    markerId++;
    marker.id = markerId;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = posX;
    marker.pose.position.y = posY;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = size;
    marker.scale.y = size;
    marker.scale.z = height;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    marker_array.markers.push_back(marker);
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

  ImageConverter()
      : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera2/rgb/image_raw", 1,
                               &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }
  //Opencv need an special format for their pictures so in this function we are making a bridge so that the openCV library can read the ros sensor data.
  void imageCb(const sensor_msgs::ImageConstPtr &msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //goes from BGR to HSV filter
    cv::cvtColor(cv_ptr->image, HSVImage, CV_BGR2HSV);

    //draws our detection window
    P1.x = 100;
    P1.y = 432;
    P2.x = 582;
    P2.y = 204;
    P3.x = 0;
    P3.y = 479;
    P4.x = P1.x;
    P4.y = 0;
    P5.x = 639;
    P5.y = P1.y;
    P6.x = P2.x;
    P6.y = P3.y;
    P7.x = P5.x;
    P7.y = P4.y;
    P8.x = P3.x;
    P8.y = P2.y;
  
    ros::Duration delay(7.0);

    //kanter rundt om vores detection window
    cv::rectangle(HSVImage, P3, P4, red, -1);
    cv::rectangle(HSVImage, P3, P5, red, -1);
    cv::rectangle(HSVImage, P6, P7, red, -1);
    cv::rectangle(HSVImage, P7, P8, red, -1);

    cv::rectangle(cv_ptr->image, P1, P2, red);

    //Scans the image for red, the Scalar is calibrated with the calibration.cpp
    cv::inRange(HSVImage, cv::Scalar(47, 79, 111), cv::Scalar(104, 255, 255), ThreshImage);
    //blurs the image to filter the noise
    cv::blur(ThreshImage, Blurimage, cv::Size(3, 3));
    //Makes the lines more visible
    cv::Canny(Blurimage, Edgedimage, 50, 200, 3);

    //We will make an vector named contour which we can use to analyse our matrix
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(Edgedimage, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    if (contours.size() != 0)
    {
      std::vector<std::vector<cv::Point>> contours_poly(contours.size());
      std::vector<cv::Rect> boundRect(contours.size());

      for (std::size_t i = 0; i < contours.size(); i++)
      {
        cv::approxPolyDP(contours[i], contours_poly[i], 3, true);
        boundRect[i] = cv::boundingRect(contours_poly[i]);
        double area = cv::contourArea(contours[i]);
        if (area >= 500 && currentTimer + delay < ros::Time::now())
        {
          mineCounter++;
          currentTimer = ros::Time::now();
          std::cout << "der er en mine i dette område" << std::endl;
          setPointMap(0.3, 0.0, 0.2, 0.2, visualization_msgs::Marker::CUBE);
          soundMSG.value=6;
          sound_pub.publish(soundMSG);
          cv::imwrite(cv::format("demining_project/Mine_Pictures/mine%d.bmp",mineCounter), cv_ptr->image);
          
          
          double currentPos[4];
          getMinePosition(currentPos);
          mineMessage.header.frame_id=std::to_string(mineCounter);
          mineMessage.pose.position.x = currentPos[0];
          mineMessage.pose.position.y = currentPos[1];
          mine_pub.publish(mineMessage);
          minePosX[mineCounter] = currentPos[0]; //Define X-coordinate for mine to SaveData();
          minePosY[mineCounter] = currentPos[1]; //Define Y-coordinate for mine to SaveData();
          SaveData();
      
        };

        // Drawing the contours on our image window
        cv::drawContours(cv_ptr->image, contours_poly, int(i), NavyBlue);
        cv::rectangle(cv_ptr->image, boundRect[i].tl(), boundRect[i].br(), NavyBlue);
      };
    };

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
  };

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  if(firstRun == 0){
    currentTimer = ros::Time::now();
    firstRun = 1;
  }
  ros::spin();
  return 0;
}