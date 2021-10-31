cmake_minimum_required(VERSION 3.0.2)
project(teleop_wasd_keyboard)

## Compile as C++11, supported in ROS Kinetic and newer

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
)

## ------------------------------------------------------------------------- Declare ROS messages, services and actions 

## ------------------------------------------------------------------------- Declare ROS dynamic reconfigure parameters 

## ------------------------------------------------------------------------- catkin specific configuration 

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES teleop_wasd_keyboard
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

## ------------------------------------------------------------------------- Build 
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



## ------------------------------------------------------------------------- Install 

## ------------------------------------------------------------------------- Testing 