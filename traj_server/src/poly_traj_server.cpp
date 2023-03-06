/**
 * @file poly_traj_server.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief receive parametric polynomial trajectory, publish position command
 * @version 1.0
 * @date 2022-08-03
 * @copyright Copyright (c) 2022
 *
 */
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <Eigen/Eigen>
#include <queue>

#include "quadrotor_msgs/PositionCommand.h"
#include "traj_server/visualizer.hpp"
#include "traj_utils/PolyTraj.h"
#include "traj_utils/poly_traj.hpp"
#include "trajectory_msgs/JointTrajectoryPoint.h"

ros::Publisher _pos_cmd_pub, _pva_pub, _vis_pub;
ros::Publisher _error_pub;

bool _is_traj_received = false;
bool _is_triggered     = false;

int                    _traj_id;
polynomial::Trajectory _traj;
ros::Time              _t_str;     // start time
ros::Time              _t_end;     // end time
ros::Time              _t_cur;     // current time
double                 _duration;  // duration of the trajectory in seconds

double _last_yaw    = 0;
double _last_yawdot = 0;

Eigen::Vector3d _odom_pos;

std::queue<TrajPoint> _traj_queue;

TrajSrvVisualizer::Ptr _vis_ptr;

/**
 * @brief Get the yaw angle and yaw dot
 *
 * @param v velocity
 * @param yaw return yaw angle
 * @param yaw_dot return yaw dot
 */
void getYaw(const Eigen::Vector3d &v, double &yaw, double &yaw_dot) {
  constexpr double PI                  = 3.1415926;
  constexpr double YAW_DOT_MAX_PER_SEC = PI;
  constexpr double MAX_YAW_CHANGE      = YAW_DOT_MAX_PER_SEC * 0.01;

  yaw                 = 0;
  yaw_dot             = 0;
  Eigen::Vector3d dir = v.normalized();
  // std::cout << "dir: " << dir.transpose() << std::endl;

  double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : _last_yaw;
  // std::cout << "yaw_temp: " << yaw_temp << std::endl;
  if (yaw_temp - _last_yaw > PI) {
    if (yaw_temp - _last_yaw - 2 * PI < -MAX_YAW_CHANGE) {
      yaw = _last_yaw - MAX_YAW_CHANGE;
      if (yaw < -PI) yaw += 2 * PI;

      yaw_dot = -YAW_DOT_MAX_PER_SEC;
    } else {
      yaw = yaw_temp;
      if (yaw - _last_yaw > PI)
        yaw_dot = -YAW_DOT_MAX_PER_SEC;
      else
        yaw_dot = (yaw_temp - _last_yaw) / 0.01;
    }
  } else if (yaw_temp - _last_yaw < -PI) {
    if (yaw_temp - _last_yaw + 2 * PI > MAX_YAW_CHANGE) {
      yaw = _last_yaw + MAX_YAW_CHANGE;
      if (yaw > PI) yaw -= 2 * PI;

      yaw_dot = YAW_DOT_MAX_PER_SEC;
    } else {
      yaw = yaw_temp;
      if (yaw - _last_yaw < -PI)
        yaw_dot = YAW_DOT_MAX_PER_SEC;
      else
        yaw_dot = (yaw_temp - _last_yaw) / 0.01;
    }
  } else {
    if (yaw_temp - _last_yaw < -MAX_YAW_CHANGE) {
      yaw = _last_yaw - MAX_YAW_CHANGE;
      if (yaw < -PI) yaw += 2 * PI;

      yaw_dot = -YAW_DOT_MAX_PER_SEC;
    } else if (yaw_temp - _last_yaw > MAX_YAW_CHANGE) {
      yaw = _last_yaw + MAX_YAW_CHANGE;
      if (yaw > PI) yaw -= 2 * PI;

      yaw_dot = YAW_DOT_MAX_PER_SEC;
    } else {
      yaw = yaw_temp;
      if (yaw - _last_yaw > PI)
        yaw_dot = -YAW_DOT_MAX_PER_SEC;
      else if (yaw - _last_yaw < -PI)
        yaw_dot = YAW_DOT_MAX_PER_SEC;
      else
        yaw_dot = (yaw_temp - _last_yaw) / 0.01;
    }
  }
  if (fabs(yaw - _last_yaw) <= MAX_YAW_CHANGE) yaw = 0.5 * _last_yaw + 0.5 * yaw;  // nieve LPF
  yaw_dot      = 0.5 * _last_yawdot + 0.5 * yaw_dot;
  _last_yaw    = yaw;
  _last_yawdot = yaw_dot;
  // std::cout << "yaw: " << yaw << " yaw_dot: " << yaw_dot << std::endl;
}

/** publish position command for gazebo simulation and real world test */
void publishPVA(const Eigen::Vector3d &pos,
                const Eigen::Vector3d &vel,
                const Eigen::Vector3d &acc,
                const double &         yaw,
                const double &         yaw_dot) {
  trajectory_msgs::JointTrajectoryPoint pva_msg;
  pva_msg.positions.push_back(pos(0));
  pva_msg.positions.push_back(pos(1));
  pva_msg.positions.push_back(pos(2));
  pva_msg.positions.push_back(yaw);
  pva_msg.velocities.push_back(vel(0));
  pva_msg.velocities.push_back(vel(1));
  pva_msg.velocities.push_back(vel(2));
  pva_msg.accelerations.push_back(acc(0));
  pva_msg.accelerations.push_back(acc(1));
  pva_msg.accelerations.push_back(acc(2));
  _pva_pub.publish(pva_msg);
}

/** Publish position command for fake drone simulation */
void publishCmd(const Eigen::Vector3d &pos,
                const Eigen::Vector3d &vel,
                const Eigen::Vector3d &acc,
                const double &         yaw,
                const double &         yaw_dot) {
  quadrotor_msgs::PositionCommand cmd_msg;
  cmd_msg.header.stamp    = _t_cur;
  cmd_msg.header.frame_id = "world";
  cmd_msg.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd_msg.trajectory_id   = _traj_id;

  cmd_msg.position.x     = pos(0);
  cmd_msg.position.y     = pos(1);
  cmd_msg.position.z     = pos(2);
  cmd_msg.yaw            = yaw;
  cmd_msg.velocity.x     = vel(0);
  cmd_msg.velocity.y     = vel(1);
  cmd_msg.velocity.z     = vel(2);
  cmd_msg.yaw_dot        = yaw_dot;
  cmd_msg.acceleration.x = acc(0);
  cmd_msg.acceleration.y = acc(1);
  cmd_msg.acceleration.z = acc(2);
  _pos_cmd_pub.publish(cmd_msg);
}

/**
 * @brief discretize trajectory, push points to the queue
 *
 * @param traj
 */
void fillTrajQueue(const polynomial::Trajectory &traj) {
  double T = traj.getDuration();

  double dt = 0.01;  // frequency 100Hz
  for (double t = 0; t < T; t += dt) {
    double          yaw, yaw_dot;
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d acc;
    Eigen::Vector3d jrk;

    pos = traj.getPos(t);
    vel = traj.getVel(t);
    acc = traj.getAcc(t);
    jrk = traj.getJrk(t);
    getYaw(vel, yaw, yaw_dot);

    TrajPoint p = {_t_str + ros::Duration(t), pos, vel, acc, jrk, yaw, yaw_dot};
    _traj_queue.push(p);
  }
}

/**
 * @brief receive trigger message, start the trajectory
 * @param msg
 */
void triggerCallback(const geometry_msgs::PoseStampedPtr &msg) {
  if (_is_triggered) {
    return;
  }
  ROS_WARN("[TrajSrv] trigger received");
  _is_triggered = true;

  _t_str = ros::Time::now();
  _t_end = _t_str + ros::Duration(_duration);
}

// void odomCallback(const nav_msgs::OdometryPtr &msg) {
//   _odom_pos = Eigen::Vector3d(msg->pose.pose.position.x,
//                               msg->pose.pose.position.y,
//                               msg->pose.pose.position.z);
// }

void odomCallback(const geometry_msgs::PoseStampedPtr &msg) {
  _odom_pos = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  _vis_ptr->visualizeOdom(msg);
}

/**
 * @brief recieve parametric polynomial trajectory
 *
 * @param msg
 */
void polyCallback(traj_utils::PolyTrajConstPtr msg) {
  _traj_id    = msg->traj_id;
  int order   = msg->order;
  int N       = order + 1;
  int n_piece = msg->duration.size();  // number of pieces

  _t_str = msg->start_time;

  /* get duration and time allocation */
  _duration = 0.0;
  std::vector<double> time_alloc;
  for (auto it = msg->duration.begin(); it != msg->duration.end(); ++it) {
    _duration += (*it);
    time_alloc.push_back(*it);
  }

  /* get coefficients */
  Eigen::VectorXd coeff;
  coeff.resize(N * 3 * n_piece);
  for (int i = 0; i < n_piece; i++) {
    for (int j = 0; j < order + 1; j++) {
      coeff[i * N * 3 + j]         = msg->coef_x[i * N + j];
      coeff[i * N * 3 + j + N]     = msg->coef_y[i * N + j];
      coeff[i * N * 3 + j + 2 * N] = msg->coef_z[i * N + j];
    }
  }

  if (time_alloc.size() <= 0) {
    _is_traj_received = false;
  } else { /** only when traj is valid */
    _is_traj_received = true;
    _traj.setDuration(time_alloc);
    _traj.setCoeffs(coeff);
    if (_t_str >= _t_end && _is_triggered) { /* look ahead */
      // ROS_INFO("[TrajSrv] %f < %f | adding to the end", _t_str.toSec(), _t_end.toSec());
      fillTrajQueue(_traj);
    } else { /* emergency and before trigger */
      // ROS_INFO("[TrajSrv] %f < %f | clearing and reloading", _t_str.toSec(), _t_end.toSec());
      std::queue<TrajPoint> empty;
      std::swap(_traj_queue, empty);
      fillTrajQueue(_traj);
    }
    _t_str = ros::Time::now();
    _t_end = _t_str + ros::Duration(_duration);
    _vis_ptr->visualizeTraj(_traj_queue);
    ROS_INFO("[TrajSrv] queue size %li", _traj_queue.size());
  }
}

/**
 * @brief publish tracking error
 *
 * @param pos_cmd
 * @param pos_real
 */
void pubTrackingError(const Eigen::Vector3d &pos_cmd, const Eigen::Vector3d &pos_real) {
  geometry_msgs::PointStamped error_msg;
  error_msg.header.frame_id = "world";
  error_msg.header.stamp    = ros::Time::now();
  error_msg.point.x         = pos_cmd(0) - pos_real(0);
  // std::cout << pos_cmd(0) << " " << pos_real(0) << std::endl;
  error_msg.point.y = pos_cmd(1) - pos_real(1);
  error_msg.point.z = pos_cmd(2) - pos_real(2);
  _error_pub.publish(error_msg);
}

/**
 * @brief publish quadrotor_msgs::PositionCommand for fake simulation
 *
 * @param e
 */
void PubCallback(const ros::TimerEvent &e) {
  if (!_is_traj_received) {
    ROS_INFO_ONCE("[TrajSrv] waiting for trajectory");
    return;
  } else if (!_is_triggered) {
    ROS_INFO_ONCE("[TrajSrv] waiting for trigger");
    return;
  }
  _t_cur = ros::Time::now();

  TrajPoint p;
  if (_traj_queue.size() == 1) {
    p         = _traj_queue.front();
    p.vel     = Eigen::Vector3d::Zero();
    p.acc     = Eigen::Vector3d::Zero();
    p.yaw_dot = 0.0;
  } else {
    p = _traj_queue.front();
    _traj_queue.pop();
  }
  publishCmd(p.pos, p.vel, p.acc, p.yaw, p.yaw_dot);
  publishPVA(p.pos, p.vel, p.acc, p.yaw, p.yaw_dot);

  /** publish tracking error */
  pubTrackingError(p.pos, _odom_pos);
  return;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "poly_traj_server");
  ros::NodeHandle nh("~");
  ros::Timer      cmd_timer   = nh.createTimer(ros::Duration(0.01), PubCallback);
  ros::Subscriber traj_sub    = nh.subscribe("trajectory", 1, polyCallback);
  ros::Subscriber trigger_sub = nh.subscribe("/traj_start_trigger", 10, triggerCallback);
  ros::Subscriber odom_sub    = nh.subscribe("odom", 10, odomCallback);
  _pva_pub     = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/pva_setpoint", 1);
  _pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 1);

  _error_pub = nh.advertise<geometry_msgs::PointStamped>("error", 1);

  _vis_ptr.reset(new TrajSrvVisualizer(nh));

  ros::Duration(3.0).sleep();
  ROS_INFO("[TrajSrv]: ready to receive trajectory");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
