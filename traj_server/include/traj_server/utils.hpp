/**
 * @file utils.hpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2022-08-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _TRAJ_SRV_UTILS_HPP
#define _TRAJ_SRV_UTILS_HPP
#include <Eigen/Eigen>
#include <ros/ros.h>
/** Trajectory queue */
struct TrajPoint {
  ros::Time       t;
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
  Eigen::Vector3d acc;
  Eigen::Vector3d jrk;
  double          yaw;
  double          yaw_dot;
};

#endif // _TRAJ_SRV_UTILS_HPP