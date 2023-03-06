cmake_minimum_required(VERSION 3.0.2)
project(traj_server)

set(CMAKE_BUILD_TYPE "Debug")
add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# add_definitions(-std=c++14 -g -O3 -ftree-vectorize -ffast-math -march=native -lpthread)
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  traj_utils
  quadrotor_msgs
  trajectory_msgs
  geometry_msgs
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_executable(poly_traj_server src/poly_traj_server.cpp)
target_link_libraries(poly_traj_server ${catkin_LIBRARIES})

add_executable(bezier_traj_server src/bezier_traj_server.cpp)
target_link_libraries(bezier_traj_server ${catkin_LIBRARIES})

add_executable(trigger src/trigger.cpp)
target_link_libraries(trigger ${catkin_LIBRARIES})

# TODO
# add_executable(bspline_server src/bspline_traj_server.cpp)
# add_executable(biezer_server src/biezer_traj_server.cpp)
# add_executable(minco_server src/minco_traj_server.cpp)
# add_executable(minvo_server src/minvo_traj_server.cpp)
