cmake_minimum_required(VERSION 2.8.3)
project(dynamic_reconfigure_example)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  dynamic_reconfigure
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/Example.cfg
)

catkin_package()

