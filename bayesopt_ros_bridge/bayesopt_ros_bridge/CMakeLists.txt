cmake_minimum_required(VERSION 2.8.0)
project(bayesopt_ros_bridge)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  rospy
)

catkin_python_setup()

catkin_package()
