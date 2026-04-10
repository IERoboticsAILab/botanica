cmake_minimum_required(VERSION 3.0.2)
project(robomaster_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
  nav_msgs
)

catkin_package()

# Install Python scripts
catkin_install_python(PROGRAMS
  src/robomaster_driver_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
