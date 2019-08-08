cmake_minimum_required(VERSION 2.8.3)
project(pub_sub_nodelet_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nodelet
)

catkin_package(
  INCLUDE_DIRS src
  CATKIN_DEPENDS roscpp std_msgs nodelet
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/PubSubNodelet.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
