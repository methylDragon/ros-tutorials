cmake_minimum_required(VERSION 2.8.3)
project(dynamic_reconfigure_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/Example.cfg
)

catkin_package(
  INCLUDE_DIRS src
  CATKIN_DEPENDS roscpp std_msgs 
  DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(dynamic_reconfigure_example src/dynamic_reconfigure_example.cpp)
target_link_libraries(dynamic_reconfigure_example ${catkin_LIBRARIES})
add_dependencies(dynamic_reconfigure_example ${PROJECT_NAME}_gencfg)

add_executable(dynamic_reconfigure_client src/dynamic_reconfigure_client.cpp)
target_link_libraries(dynamic_reconfigure_client ${catkin_LIBRARIES})
add_dependencies(dynamic_reconfigure_client ${PROJECT_NAME}_gencfg)
