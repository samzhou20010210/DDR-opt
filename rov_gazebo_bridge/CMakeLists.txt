cmake_minimum_required(VERSION 3.0.2)
project(rov_gazebo_bridge)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS scripts/gazebo_bridge.py scripts/pool_obstacles.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
