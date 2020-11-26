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
)

###################################
## catkin specific configuration ##
###################################
find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
    cv_bridge
    image_transport
  )

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
add_executable(MineSweeper src/MineSweeper.cpp)
add_executable(locate_d_area src/locate_d_area.cpp)
add_executable(Input_User src/Input_User.cpp)
add_executable(BumperTest src/BumperTest.cpp)
add_executable(FMineTest src/FMineTest.cpp)
# Link to Catkin libraries
target_link_libraries(hello_world ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})
target_link_libraries(calibration ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})
target_link_libraries(MineSweeper ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})
target_link_libraries(locate_d_area ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})
target_link_libraries(Input_User ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})
target_link_libraries(BumperTest ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})
target_link_libraries(FMineTest ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})



#############
## Install ##
############w#

# Mark executables for installation
install(TARGETS hello_world locate_d_area calibration MineSweeper Input_User BumperTest FMineTest


  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
