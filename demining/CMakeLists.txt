cmake_minimum_required(VERSION 2.8.3)
project(demining)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  turtlesim
  sensor_msgs
  cv_bridge
  std_msgs
  image_transport
  geometry_msgs
  move_base_msgs 
  actionlib
  visualization_msgs
  tf
)

###################################
## catkin specific configuration ##
###################################

  find_package(OpenCV REQUIRED)

  catkin_package()
###########
## Build ##
###########
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
  )

# Declare Kill Turtles C++ executable and rename to short name (in case catkin_make is used).
add_executable(hello_world src/hello_world.cpp)
add_executable(calibration src/calibration.cpp)
#add_executable(MineSweeper src/MineSweeper.cpp)
add_executable(locate_d_area src/locate_d_area.cpp)
add_executable(batteryMonitor src/batteryMonitor.cpp)
add_executable(coordinatePublisher src/coordinatePublisher.cpp)

# Link to Catkin libraries
target_link_libraries(hello_world ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})
target_link_libraries(calibration ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})
#target_link_libraries(MineSweeper ${catkin_LIBRARIES})
target_link_libraries(locate_d_area ${catkin_LIBRARIES})
target_link_libraries(batteryMonitor ${catkin_LIBRARIES})
target_link_libraries(coordinatePublisher ${catkin_LIBRARIES})


#############
## Install ##
############w#

# Mark executables for installation
install(TARGETS hello_world locate_d_area calibration 


  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} 
)

install(TARGETS batteryMonitor RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS coordinatePublisher RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#install(TARGETS MineSweeper RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
