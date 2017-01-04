cmake_minimum_required(VERSION 2.8.3)
project(ifor_offb2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

set (CMAKE_CXX_STANDARD 11)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(ifor_offb2_node src/ifor_offb2_node.cpp)

target_link_libraries(ifor_offb2_node ${catkin_LIBRARIES})