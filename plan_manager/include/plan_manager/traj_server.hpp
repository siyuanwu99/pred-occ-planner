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

#ifndef _TRAJ_SERVER_HPP
#define _TRAJ_SERVER_HPP

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <deque>
#include "quadrotor_msgs/PositionCommand.h"
#include "traj_utils/BezierTraj.h"
#include "traj_utils/bernstein.hpp"
#include "trajectory_msgs/JointTrajectoryPoint.h"

const double DELTA_T = 0.01;  // 100 Hz
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

class TrajServer {
 public:
  TrajServer() {}
  ~TrajServer() {}
  void init();

  TrajPoint popFront();
  TrajPoint popBack();

 private:
  void getYaw(const Eigen::Vector3d &v, double &yaw, double &yaw_dot);

 private:
  double                last_yaw_;
  double                last_yaw_dot_;
  Bernstein::Bezier     traj_;
  std::deque<TrajPoint> traj_queue_;
};

void TrajServer::getYaw(const Eigen::Vector3d &v, double &yaw, double &yaw_dot) {
  constexpr double PI                  = 3.1415926;
  constexpr double YAW_DOT_MAX_PER_SEC = PI;
  constexpr double MAX_YAW_CHANGE      = YAW_DOT_MAX_PER_SEC * 0.01;

  yaw                 = 0;
  yaw_dot             = 0;
  Eigen::Vector3d dir = v.normalized();
  // std::cout << "dir: " << dir.transpose() << std::endl;

  double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
  // std::cout << "yaw_temp: " << yaw_temp << std::endl;
  if (yaw_temp - last_yaw_ > PI) {
    if (yaw_temp - last_yaw_ - 2 * PI < -MAX_YAW_CHANGE) {
      yaw = last_yaw_ - MAX_YAW_CHANGE;
      if (yaw < -PI) yaw += 2 * PI;

      yaw_dot = -YAW_DOT_MAX_PER_SEC;
    } else {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yaw_dot = -YAW_DOT_MAX_PER_SEC;
      else
        yaw_dot = (yaw_temp - last_yaw_) / 0.01;
    }
  } else if (yaw_temp - last_yaw_ < -PI) {
    if (yaw_temp - last_yaw_ + 2 * PI > MAX_YAW_CHANGE) {
      yaw = last_yaw_ + MAX_YAW_CHANGE;
      if (yaw > PI) yaw -= 2 * PI;

      yaw_dot = YAW_DOT_MAX_PER_SEC;
    } else {
      yaw = yaw_temp;
      if (yaw - last_yaw_ < -PI)
        yaw_dot = YAW_DOT_MAX_PER_SEC;
      else
        yaw_dot = (yaw_temp - last_yaw_) / 0.01;
    }
  } else {
    if (yaw_temp - last_yaw_ < -MAX_YAW_CHANGE) {
      yaw = last_yaw_ - MAX_YAW_CHANGE;
      if (yaw < -PI) yaw += 2 * PI;

      yaw_dot = -YAW_DOT_MAX_PER_SEC;
    } else if (yaw_temp - last_yaw_ > MAX_YAW_CHANGE) {
      yaw = last_yaw_ + MAX_YAW_CHANGE;
      if (yaw > PI) yaw -= 2 * PI;

      yaw_dot = YAW_DOT_MAX_PER_SEC;
    } else {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yaw_dot = -YAW_DOT_MAX_PER_SEC;
      else if (yaw - last_yaw_ < -PI)
        yaw_dot = YAW_DOT_MAX_PER_SEC;
      else
        yaw_dot = (yaw_temp - last_yaw_) / 0.01;
    }
  }
  if (fabs(yaw - last_yaw_) <= MAX_YAW_CHANGE) yaw = 0.5 * last_yaw_ + 0.5 * yaw;  // nieve LPF
  yaw_dot      = 0.5 * last_yawdot_ + 0.5 * yaw_dot;
  last_yaw_    = yaw;
  last_yawdot_ = yaw_dot;
  // std::cout << "yaw: " << yaw << " yaw_dot: " << yaw_dot << std::endl;
  // }

#endif  // _TRAJ_SERVER_HPP
