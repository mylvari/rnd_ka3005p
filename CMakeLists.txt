cmake_minimum_required(VERSION 3.0.2)
project(rnd_ka3005p)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  serial
  message_generation
)

add_message_files(
  FILES
  RndKa3005pStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(${PROJECT_NAME}_node src/rnd_ka3005p_node.cpp src/rnd_ka3005p.cpp src/rnd_ka3005p_ros.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
