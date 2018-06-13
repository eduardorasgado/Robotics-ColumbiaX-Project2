cmake_minimum_required(VERSION 2.8.3)
project(project2)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  rospy
  std_msgs
  tf
  tf2
  tf2_ros
  geometry_msgs
  visualization_msgs
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   visualization_msgs
 )

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

 add_executable(marker_publisher src/marker_publisher.cpp)

 add_dependencies(marker_publisher marker_publisher_generate_messages_cpp)

 target_link_libraries(marker_publisher
   ${catkin_LIBRARIES})
