
cmake_minimum_required(VERSION 3.0.2)
project(traj_coordinator)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -lpthread")

find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  traj_utils
  separator # MADER linear separator
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES traj_coordinator
  CATKIN_DEPENDS roscpp traj_utils separator
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

# # Declare a C++ library
add_library(${PROJECT_NAME}
  src/mader.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
