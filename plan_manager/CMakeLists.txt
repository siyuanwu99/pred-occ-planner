cmake_minimum_required(VERSION 3.0.2)
project(plan_manager)

set(CMAKE_BUILD_TYPE "Release")

# set(CMAKE_BUILD_TYPE "Debug")
add_compile_options(-std=c++14)

# set(CMAKE_CXX_FLAGS_DEBUG "-O1 -Wall -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -lpthread")
find_package(PCL 1.10 REQUIRED)
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_filters
  sensor_msgs
  image_transport
  nav_msgs
  quadrotor_msgs
  traj_utils
  traj_opt
  decomp_ros_msgs
  plan_env
  path_searching
  traj_coordinator
  quadrotor_msgs
  # separator # MADER linear separator
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES plan_manager
  CATKIN_DEPENDS roscpp traj_opt traj_utils traj_coordinator plan_env path_searching
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

# # Declare a C++ library
add_library(${PROJECT_NAME}
  src/baseline.cpp
  src/plan_manager.cpp
  # src/mader_deconfliction.cpp
  src/baseline_fake.cpp # Debug
  src/fake_plan_manager.cpp # Debug
  # robust mader
  # src/rmader/fake_rmader.cpp
  # src/rmader/fake_rmader_plan_manager.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# add_executable(test_baseline src/test_baseline_node.cpp)
# target_link_libraries(test_baseline ${catkin_LIBRARIES} ${PROJECT_NAME})
add_executable(baseline src/planner_node.cpp)
target_link_libraries(baseline ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(fake_baseline src/fake_planner_node.cpp)
target_link_libraries(fake_baseline ${catkin_LIBRARIES} ${PROJECT_NAME})

# add_executable(test_firi src/test_firi_node.cpp)
# target_link_libraries(test_firi ${catkin_LIBRARIES} ${PROJECT_NAME})

# add_executable(rmader src/rmader/fake_rmader_node.cpp)
# target_link_libraries(rmader ${catkin_LIBRARIES} ${PROJECT_NAME})
