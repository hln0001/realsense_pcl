cmake_minimum_required(VERSION 2.8.3)
project(messages)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
  CollisionTrajectory.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  messages
)

catkin_package(CATKIN_DEPENDS std_msgs message_runtime)
