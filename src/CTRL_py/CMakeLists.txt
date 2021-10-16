cmake_minimum_required(VERSION 2.8.3)
project(CTRL_py)

find_package(catkin REQUIRED COMPONENTS
  ur3_driver
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lab2pkg_py
#  CATKIN_DEPENDS ece470_ur3_driver roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)