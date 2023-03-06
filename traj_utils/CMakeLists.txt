cmake_minimum_required(VERSION 3.0.0)
project(traj_utils)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# set(ENABLE_PRECOMPILED_HEADERS "OFF")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  decomp_ros_utils
)

find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

find_package(PCL 1.7 REQUIRED)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  DataDisp.msg
  PolyTraj.msg
  MINCOTraj.msg
  BsplineTraj.msg
  BezierTraj.msg
  CentralPos.msg
  CentralState.msg
  MultiBsplines.msg
  GoalSet.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES traj_utils
  CATKIN_DEPENDS decomp_ros_utils roscpp geometry_msgs

  # DEPENDS system_lib
)

include_directories(
  SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library(traj_utils

  # src/planning_visualization.cpp
  src/corridor.cpp
  src/visualizer.cpp
  src/uniform_bspline.cpp
  src/bernstein.cpp
)
target_link_libraries(traj_utils
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${PCL_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(test_bernstein test/test_bernstein.cpp)

  # add_rostest_gtest(test_bernstein test/test_bernstein.cpp)
  target_link_libraries(test_bernstein traj_utils ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
endif()
