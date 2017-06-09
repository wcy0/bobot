cmake_minimum_required(VERSION 2.8.3)
project(bo_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roslib
  sensor_msgs
  std_msgs
  message_generation
  actionlib_msgs
)

add_message_files(
  FILES
  bo_Uint8Array.msg
  bo_DataGram.msg
)

add_service_files(

)

add_action_files(

)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
  actionlib_msgs
)

catkin_package (CATKIN_DEPENDS
geometry_msgs roslib sensor_msgs std_msgs
#DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
