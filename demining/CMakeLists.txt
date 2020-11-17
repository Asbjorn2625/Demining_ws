cmake_minimum_required(VERSION 2.8.3)
project(demining)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  turtlesim
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Declare Kill Turtles C++ executable and rename to short name (in case catkin_make is used).
add_executable(hello_world src/hello_world.cpp)
# Link to Catkin libraries
target_link_libraries(hello_world
  ${catkin_LIBRARIES}
)





#############
## Install ##
#############

# Mark executables for installation
install(TARGETS hello_world


  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
