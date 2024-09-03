cmake_minimum_required(VERSION 2.8.3)
project(plan_env)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_BUILD_TYPE "RelWithDebInfo")
ADD_COMPILE_OPTIONS(-std=c++11)
ADD_COMPILE_OPTIONS(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    visualization_msgs
    cv_bridge
    message_filters
    traj_coordinator # for adding other traj into map
    quadrotor_msgs # only for examples
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

set(Munkres_INCLUDE_DIRS
    /usr/local/include/munkres
)
include_directories(${Munkres_INCLUDE_DIRS})
set(Munkres_LIBS
    /usr/local/lib/libmunkres.a
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp std_msgs cv_bridge traj_coordinator
    DEPENDS OpenCV Munkres
)

include_directories(
    SYSTEM
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library(plan_env
    src/map.cpp
    src/grid_map.cpp
    src/raycast.cpp
    src/obj_predictor.cpp
    src/gt_visualize.cpp
    # src/dsp_map_new.cpp
    src/risk_voxel.cpp
    # src/fake_dsp_map.cpp
    src/risk_base.cpp
    src/fake_particle_risk_voxel.cpp
)
target_link_libraries(plan_env
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${Munkres_LIBS}
)

# add_executable(obj_generator
# src/obj_generator.cpp
# )
# target_link_libraries(obj_generator
# ${catkin_LIBRARIES}
# )

# add_executable(test_grid_map_node
# src/test_grid_map_node.cpp
# )
# target_link_libraries(test_grid_map_node
# plan_env
# ${catkin_LIBRARIES}
# )

# add_executable(test_dsp_map_node
# src/test_dsp_map_node.cpp
# )
# target_link_libraries(test_dsp_map_node
# plan_env
# ${catkin_LIBRARIES}
# )

# add_executable(dsp_map_node
# src/dsp_map_node.cpp
# )
# target_link_libraries(dsp_map_node
# plan_env
# ${catkin_LIBRARIES}
# )
add_executable(risk_voxel_node src/risk_voxel_node.cpp)
target_link_libraries(risk_voxel_node
    plan_env
    ${catkin_LIBRARIES}
)

add_executable(risk_mapping src/risk_mapping_node.cpp)
target_link_libraries(risk_mapping
    plan_env
    ${catkin_LIBRARIES}
)

add_executable(pedestrian_mapping src/ped_mapping_node.cpp)
target_link_libraries(pedestrian_mapping
    plan_env
    ${catkin_LIBRARIES}
)

add_executable(gt_visualizer src/gt_visualizer_node.cpp)
target_link_libraries(gt_visualizer
    plan_env
    ${catkin_LIBRARIES}
)

# add_executable(risk_voxel_moving src/risk_voxel_moving.cpp)
# target_link_libraries(risk_voxel_moving
#     plan_env
#     ${catkin_LIBRARIES}
# )
