cmake_minimum_required(VERSION 3.0.2)
project(mot)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  
#)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES mot
  CATKIN_DEPENDS rospy message_runtime
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
