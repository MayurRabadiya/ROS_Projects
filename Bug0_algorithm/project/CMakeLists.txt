cmake_minimum_required(VERSION 3.0.2)
project(project)

# Find the required catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf
)

# Declare the catkin package
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs nav_msgs tf
)

# Specify include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/bug.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
