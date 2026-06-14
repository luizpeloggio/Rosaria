cmake_minimum_required(VERSION 3.0.2)
project(rosaria_astar)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  nodes/ros_astar_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
