cmake_minimum_required(VERSION 2.8.3)
project(demining)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
<<<<<<< HEAD
  roscpp
  turtlesim
  sensor_msgs
  cv_bridge
  std_msgs
  image_transport
  geometry_msgs
  move_base_msgs 
  actionlib
  tf
  nav_msgs
  visualization_msgs
=======
roscpp
turtlesim
sensor_msgs
cv_bridge
std_msgs
image_transport
geometry_msgs
move_base_msgs 
actionlib
tf
nav_msgs
visualization_msgs
>>>>>>> Asbjørns-legeplads
)

###################################
## catkin specific configuration ##
###################################
<<<<<<< HEAD
=======

>>>>>>> Asbjørns-legeplads

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
add_executable(mine_detector src/mine_detector.cpp)
add_executable(calibration src/calibration.cpp)
add_executable(MineSweeper src/MineSweeper.cpp)
add_executable(locate_d_area src/locate_d_area.cpp)
<<<<<<< HEAD
add_executable(batteryMonitor src/batteryMonitor.cpp)
add_executable(coordinatePublisher src/coordinatePublisher.cpp)

=======
add_executable(Input_User src/Input_User.cpp)
add_executable(BumperTest src/BumperTest.cpp)
add_executable(FMineTest src/FMineTest.cpp)
>>>>>>> origin/Subfunctions
# Link to Catkin libraries
target_link_libraries(mine_detector ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})
target_link_libraries(calibration ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})
<<<<<<< HEAD
#target_link_libraries(MineSweeper ${catkin_LIBRARIES})
target_link_libraries(locate_d_area ${catkin_LIBRARIES})
target_link_libraries(batteryMonitor ${catkin_LIBRARIES})
target_link_libraries(coordinatePublisher ${catkin_LIBRARIES})
=======
target_link_libraries(MineSweeper ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})
target_link_libraries(locate_d_area ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})
target_link_libraries(Input_User ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})
target_link_libraries(BumperTest ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})
target_link_libraries(FMineTest ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

>>>>>>> origin/Subfunctions


#############
## Install ##
############w#

# Mark executables for installation
<<<<<<< HEAD
<<<<<<< HEAD
install(TARGETS hello_world locate_d_area calibration 
=======
install(TARGETS mine_detector locate_d_area calibration
>>>>>>> Asbjørns-legeplads


  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} 
)

install(TARGETS batteryMonitor RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS coordinatePublisher RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#install(TARGETS MineSweeper RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
=======
install(TARGETS hello_world locate_d_area calibration MineSweeper Input_User BumperTest FMineTest


  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
>>>>>>> origin/Subfunctions
