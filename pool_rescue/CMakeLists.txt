cmake_minimum_required(VERSION 3.0.2)
project(pool_rescue)

set(CMAKE_CXX_STANDARD 14)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  tf2_ros
  carstatemsgs
  message_generation
)

add_message_files(FILES
  Person.msg
  PersonArray.msg
  ExtractionZone.msg
  RescueTask.msg
)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs tf tf2_ros carstatemsgs message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(ddr_adapter src/ddr_adapter.cpp)
add_dependencies(ddr_adapter ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(ddr_adapter ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS scripts/gen_ddr_params.py scripts/check_params.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
