cmake_minimum_required(VERSION 2.8.3)
project(path_searching)

# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    visualization_msgs
    plan_env
    cv_bridge
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES path_searching
    CATKIN_DEPENDS plan_env

    # DEPENDS system_lib
)

include_directories(
    SYSTEM
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

add_library(path_searching
    src/dyn_a_star.cpp
    src/fake_risk_hybrid_a_star.cpp
)
target_link_libraries(path_searching
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_executable(a_star_node src/vis_a_star_node.cpp)
target_link_libraries(a_star_node
    ${catkin_LIBRARIES}
    path_searching
)

add_executable(fake_risk_a_star_node src/vis_fake_risk_a_star_node.cpp)
target_link_libraries(fake_risk_a_star_node
    ${catkin_LIBRARIES}
    path_searching
)
