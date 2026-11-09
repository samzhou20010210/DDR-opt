cmake_minimum_required(VERSION 3.0.2)
project(holo_control)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs nav_msgs carstatemsgs back_end)
find_package(OsqpEigen REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs carstatemsgs back_end)

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} /usr/local/include/osqp)

add_executable(holo_nmpc src/holo_nmpc.cpp)
target_link_libraries(holo_nmpc OsqpEigen::OsqpEigen ${catkin_LIBRARIES})
add_dependencies(holo_nmpc ${catkin_EXPORTED_TARGETS})

add_executable(thruster_allocation src/thruster_allocation.cpp)
target_link_libraries(thruster_allocation OsqpEigen::OsqpEigen ${catkin_LIBRARIES})
add_dependencies(thruster_allocation ${catkin_EXPORTED_TARGETS})
