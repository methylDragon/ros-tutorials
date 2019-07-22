cmake_minimum_required(VERSION 2.8.3)
project(class_pub_sub)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS src
  CATKIN_DEPENDS roscpp std_msgs 
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(class_pub_sub src/class_pub_sub.cpp)
target_link_libraries(class_pub_sub ${catkin_LIBRARIES})
