cmake_minimum_required(VERSION 2.8.3)
project(uav_simulation)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES 
#  CATKIN_DEPENDS 
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable (referee_system src/referee_system.cpp )
target_link_libraries(referee_system
${catkin_LIBRARIES})

add_executable (detect_sim src/detect_sim.cpp )
target_link_libraries(detect_sim
${catkin_LIBRARIES})



catkin_install_python(PROGRAMS
  src/keyboard_control.py
  src/command_process.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
