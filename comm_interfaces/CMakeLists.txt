cmake_minimum_required(VERSION 2.8.0)
project(comm_interfaces)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  rospy
)

catkin_python_setup()

catkin_package()
