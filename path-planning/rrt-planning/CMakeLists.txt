cmake_minimum_required(VERSION 2.8.3)
project(rrt-planning)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rrt src/rrt.cpp src/rrtImpl.h)
target_link_libraries(rrt ${catkin_LIBRARIES})
add_dependencies(rrt ros_rrt_generate_messages_cpp)
