/**
 * @file test_minisnap.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-08-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Eigen>
#include <iostream>
#include <random>
#include <vector>

#include "traj_utils/visualizer.hpp"
#include "polynomial/mini_snap.h"

/* initialize random seed */
std::random_device                     rd;
std::default_random_engine             eng(rd());
std::uniform_real_distribution<double> _x_rand(-1.0, 1.0);
std::uniform_real_distribution<double> _z_rand(0.0, 3.0);
std::uniform_real_distribution<double> _v_rand(0.0, 5.0);
std::uniform_real_distribution<double> _t_rand(0.0, 2.0);  // time interval

Eigen::Vector3d _start_pos(0.0, 0.0, 0.0);
Eigen::Vector3d _start_vel(0.0, 0.0, 0.0);
Eigen::Vector3d _start_acc(0.0, 0.0, 0.0);
Eigen::Vector3d _end_pos(0.0, 0.0, 0.0);
Eigen::Vector3d _end_vel(0.0, 0.0, 0.0);
Eigen::Vector3d _end_acc(0.0, 0.0, 0.0);

polynomial::Trajectory       _traj;
polynomial::CorridorMiniSnap _optimizer;
visualizer::Visualizer::Ptr       _vis;

/**
 * @brief optimize trajectory
 *
 * @param init
 * @param goal
 * @param t_alloc
 * @param corridors
 * @return true
 * @return false
 */
bool Optimize(const Eigen::Matrix3d &                          init,
              const Eigen::Matrix3d &                          goal,
              const std::vector<double> &                      t_alloc,
              const std::vector<Eigen::Matrix<double, 6, -1>> &corridors) {
  bool                is_solved = false;
  std::vector<double> factors   = {0.0, 0.0, 0.0, 0.0, 1.0};

  /* initial guess */
  _optimizer.reset(init, goal, t_alloc, corridors);
  try {
    is_solved = _optimizer.optimize(0.0);

  } catch (int e) {
    ROS_ERROR("Optimization failed!");
    return false;
  }

  if (is_solved) {
    _optimizer.getTrajectory(&_traj);
  } else {
    ROS_ERROR("Optimization failed!");
    return false;
  }

  /* reoptimization */
  int i = 0, I = 10;
  while (!_optimizer.isCorridorSatisfied(_traj, 10.0, 10.0, 0) && i++ < I) {
    try {
      is_solved = _optimizer.reOptimize();
    } catch (int e) {
      ROS_ERROR("Re-optimization failed!");
      return false;
    }

    if (is_solved) {
      _optimizer.getTrajectory(&_traj);
    } else {
      ROS_ERROR("Re-optimization failed!");
      return false;
    }
  }
  return true;
}

/**
 * @brief randomly generate time allocations
 *
 * @param n
 * @return std::vector<double>
 */
std::vector<double> getRandomTimeAlloc(int n) {
  std::vector<double> time_alloc(n);
  for (int i = 0; i < n; i++) {
    time_alloc[i] = _t_rand(eng);
  }
  return time_alloc;
}

/**
 * @brief Get the Random Waypoint object
 *
 * @param n
 * @param srt
 * @param end
 * @return std::vector<Eigen::Vector3d>
 */
std::vector<Eigen::Vector3d> getRandomWaypoint(int                    n,
                                               const Eigen::Vector3d &srt,
                                               const Eigen::Vector3d &end) {
  std::vector<Eigen::Vector3d> waypoints(n + 1);
  waypoints[0] = srt;
  waypoints[n] = end;
  for (int i = 1; i < n; i++) {
    waypoints[i] = srt + (end - srt) * i / n;
    waypoints[i][0] += _x_rand(eng);
    waypoints[i][1] += _x_rand(eng);
    waypoints[i][2] = _z_rand(eng);
  }
  return waypoints;
}

std::vector<Eigen::Matrix<double, 6, -1>> getRandomCorridors(
    int n, const std::vector<Eigen::Vector3d> &wps) {
  double          l_margin = 0.5;
  double          w_margin = 3.0;
  double          h        = 2.0;
  Eigen::Vector3d up(0.0, 0.0, 1.0);

  std::vector<Eigen::Matrix<double, 6, -1>> crds;
  for (int i = 0; i < n; i++) {
    Eigen::Matrix<double, 6, 6> crd;
    Eigen::Vector3d             srt = wps[i];
    Eigen::Vector3d             end = wps[i + 1];
    Eigen::Vector3d             mdl = (srt + end) / 2;

    Eigen::Vector3d lvec = end - srt;
    lvec                 = lvec / lvec.norm();
    Eigen::Vector3d wvec = up.cross(lvec);

    crd.col(0).head<3>() = -lvec;
    crd.col(0).tail<3>() = srt - lvec * l_margin;
    crd.col(1).head<3>() = lvec;
    crd.col(1).tail<3>() = end + lvec * l_margin;
    crd.col(2).head<3>() = up;
    crd.col(2).tail<3>() = mdl + up * h;
    crd.col(3).head<3>() = -up;
    crd.col(3).tail<3>() = mdl - up * h;
    crd.col(4).head<3>() = wvec;
    crd.col(4).tail<3>() = mdl + wvec * w_margin;
    crd.col(5).head<3>() = -wvec;
    crd.col(5).tail<3>() = mdl - wvec * w_margin;
    crds.push_back(crd);
  }
  return crds;
}

void clickCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  _end_pos(0) = msg->pose.position.x;
  _end_pos(1) = msg->pose.position.y;
  _end_pos(2) = _z_rand(eng);

  Eigen::Quaterniond q;
  q.x() = msg->pose.orientation.x;
  q.y() = msg->pose.orientation.y;
  q.z() = msg->pose.orientation.z;
  q.w() = msg->pose.orientation.w;
  Eigen::Vector3d axis(1.0, 0.0, 0.0);
  _end_vel = q * axis * _v_rand(eng);

  int n = 2 + std::rand() % 10;
  ROS_INFO("number of pieces = %d", n);

  Eigen::Matrix3d init_state;
  Eigen::Matrix3d goal_state;
  init_state.col(0) = _start_pos;
  init_state.col(1) = _start_vel;
  init_state.col(2) = _start_acc;
  goal_state.col(0) = _end_pos;
  goal_state.col(1) = _end_vel;
  goal_state.col(2) = _end_acc;

  std::vector<double> time_alloc = getRandomTimeAlloc(n);

  std::cout << "time_alloc = " << std::endl;
  for (auto it : time_alloc) {
    std::cout << it << " ";
  }
  std::cout << std::endl;

  std::vector<Eigen::Vector3d>              wpts      = getRandomWaypoint(n, _start_pos, _end_pos);
  std::vector<Eigen::Matrix<double, 6, -1>> corridors = getRandomCorridors(n, wpts);
  std::cout << "goal" << goal_state << std::endl;
  ros::Time tic    = ros::Time::now();
  bool      status = Optimize(init_state, goal_state, time_alloc, corridors);
  ros::Time toc    = ros::Time::now();
  double    t_comp = (toc - tic).toSec() * 1000;
  ROS_INFO("Trajectory optimized in %f seconds", t_comp);

  double duration = 0.0;
  for (auto it = time_alloc.begin(); it != time_alloc.end(); it++) {
    duration += *it;
  }

  Eigen::Vector3d zero(0.0, 0.0, 0.0);
  ROS_INFO("Visualizing target");
  _vis->visualizeStartGoal(_end_pos, 1);
  ROS_INFO("Visualizing path");
  _vis->visualizePath(wpts);
  ROS_INFO("Visualizing corridor");
  _vis->visualizeCorridors(corridors, zero);
  ROS_INFO("Visualizing trajectory");
  _vis->visualizePolyTraj(zero, _traj, 4.0);
  ROS_INFO("Visualizing optimization info");
  double max_vel = _traj.getMaxVelRate();
  double max_acc = _traj.getMaxAccRate();
  _vis->displayOptimizationInfo(t_comp, max_vel, max_acc, duration);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_minisnap_node");
  ros::NodeHandle nh("~");
  _vis = std::shared_ptr<visualizer::Visualizer>(new visualizer::Visualizer(nh));
  ros::Subscriber click_sub = nh.subscribe("/move_base_simple/goal", 1, clickCallback);
  ros::spin();
  return 0;
}