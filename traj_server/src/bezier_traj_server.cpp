/**
 * @file bezier_traj_server.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-09-16
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <Eigen/Eigen>
#include <cmath>
#include <deque>
#include <queue>
#include <string>

#include "quadrotor_msgs/PositionCommand.h"
#include "traj_server/visualizer.hpp"
#include "traj_utils/BezierTraj.h"
#include "traj_utils/bernstein.hpp"
#include "trajectory_msgs/JointTrajectoryPoint.h"

/* ----- GLOBAL VARIABLES ----- */
const bool if_vis_cmd = true;

ros::Publisher pos_cmd_pub_, pva_pub_, vis_pub_;
ros::Publisher error_pub_;

const double     DELTA_T             = 0.01;  // 100 Hz
constexpr double PI                  = 3.14159265358979323846;
constexpr double YAW_DOT_MAX_PER_SEC = PI;  // max yaw rate
constexpr double MAX_YAW_CHANGE      = YAW_DOT_MAX_PER_SEC * 0.01;

bool   is_traj_received_   = false;
bool   is_triggered_       = false;
bool   is_init_yaw_needed_ = false;
bool   is_yaw_initilized_  = false;
bool   is_odom_received_   = false;
double replan_thres_ = 1.2;  // seconds of trajectory loaded into the queue

int               traj_id_;
Bernstein::Bezier traj_;
Bernstein::Bezier next_traj_;

double ts_;        // start time
double te_;        // end time
double ti_;        // initial time, when planner is triggered
double tns_;       // start time of the next trajectory
double tne_;       // end time of the next trajectory
double duration_;  // duration of the trajectory in seconds
double offset_;  // offset of the trajectory in seconds (to account for the time
                 // it takes to load the trajectory)

double init_yaw_    = 0;  // initial yaw angle
double last_yaw_    = 0;  // previous yaw value
double last_yawdot_ = 0;  // previous yawdot value

Eigen::Vector3d    init_pos_;  // initial velocity
Eigen::Vector3d    odom_pos_;  // current position
Eigen::Quaterniond odom_q_;    // current orientation

std::deque<TrajPoint> traj_queue_;

TrajSrvVisualizer::Ptr vis_ptr_;

/* ----- CALLBACKS ----- */
/**
 * @brief Get the yaw angle and yaw dot
 *
 * @param v velocity
 * @param yaw return yaw angle
 * @param yaw_dot return yaw dot
 */
void getYaw(const Eigen::Vector3d& v, double& yaw, double& yaw_dot) {
  yaw                 = 0;
  yaw_dot             = 0;
  Eigen::Vector3d dir = v.normalized(); /* velocity direction */
  // std::cout << "dir: " << dir.transpose() << std::endl;

  double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;  //??
  // std::cout << "[dbg] yaw_temp: " << yaw_temp << std::endl;
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
  if (fabs(yaw - last_yaw_) <= MAX_YAW_CHANGE)
    yaw = 0.5 * last_yaw_ + 0.5 * yaw;  // nieve LPF
  yaw_dot      = 0.5 * last_yawdot_ + 0.5 * yaw_dot;
  last_yaw_    = yaw;
  last_yawdot_ = yaw_dot;
  // std::cout << "[dbg] yaw: " << yaw << " yaw_dot: " << yaw_dot << std::endl;
}

/** publish position command for gazebo simulation and real world test */
void publishPVA(const Eigen::Vector3d& pos,
                const Eigen::Vector3d& vel,
                const Eigen::Vector3d& acc,
                const double&          yaw,
                const double&          yaw_dot) {
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
  pva_pub_.publish(pva_msg);
}

/** Publish position command for fake drone simulation */
void publishCmd(const Eigen::Vector3d& pos,
                const Eigen::Vector3d& vel,
                const Eigen::Vector3d& acc,
                const double&          yaw,
                const double&          yaw_dot) {
  quadrotor_msgs::PositionCommand cmd_msg;
  cmd_msg.header.stamp    = ros::Time::now();
  cmd_msg.header.frame_id = "world";
  cmd_msg.trajectory_flag =
      quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd_msg.trajectory_id = traj_id_;

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

  pos_cmd_pub_.publish(cmd_msg);

  if (if_vis_cmd) {
    vis_ptr_->visualizeCmd(cmd_msg);
  }
}

void pushToTrajQueue(double                 t,
                     const Eigen::Vector3d& pos,
                     const Eigen::Vector3d& vel,
                     const Eigen::Vector3d& acc,
                     const Eigen::Vector3d& jrk,
                     double                 yaw,
                     double                 yaw_dot) {
  // getYaw(vel, yaw, yaw_dot);
  ros::Time t0 = ros::Time().fromSec(ts_ + t);
  TrajPoint p  = {t0, pos, vel, acc, jrk, yaw, yaw_dot};
  traj_queue_.push_back(p);
}

/**
 * @brief fill the traj queue with a given bezier trajectory
 * @param traj
 */
void fillTrajQueue() {
  double tc = ros::Time::now().toSec();
  double tq = tc + replan_thres_;
  if (tq < ts_) {
    ROS_WARN("[TrajSrv] Invalid start time, too late!");
    ROS_INFO("[TrajSrv] t_str - tn = %f", ts_ - tc);
    return;
  }

  double t = tq - ts_;
  if (t > duration_) return;
  pushToTrajQueue(t, traj_.getPos(t), traj_.getVel(t), traj_.getAcc(t),
                  Eigen::Vector3d::Zero(), 0, 0);
  // ROS_INFO("[dbg] tq: %.2f id:%d sz:%d | t: %.2f | x: %.2f y: %.2f z: %.2f",
  // tq - ti_, traj_id_,
  //          int(traj_queue_.size()), t, traj_.getPos(t)(0),
  //          traj_.getPos(t)(1), traj_.getPos(t)(2));
}

/**
 * @brief reset the traj queue with a given bezier trajectory
 */
void resetTrajQueue(const Bernstein::Bezier& traj) {
  std::deque<TrajPoint>().swap(traj_queue_);
  double t = 0;
  for (; t < replan_thres_; t += DELTA_T) {
    pushToTrajQueue(t, traj.getPos(t), traj.getVel(t), traj.getAcc(t),
                    Eigen::Vector3d::Zero(), 0, 0);
    // ROS_INFO("[dbg] tq: %.2f id:%d sz:%d | t: %.2f | x: %.2f y: %.2f z:
    // %.2f", tns_ - ti_ + t,
    //          traj_id_, int(traj_queue_.size()), t, traj.getPos(t)(0),
    //          traj.getPos(t)(1), traj.getPos(t)(2));
  }
}

void mergeTrajQueue(const Bernstein::Bezier& traj) {
  int k = (ros::Time::now().toSec() + replan_thres_ - tns_) / DELTA_T;
  if (k > traj_queue_.size()) {
    // ROS_INFO("[TrajSrv] t_str - tn = %f,  k = %d", ts_ -
    // ros::Time::now().toSec(), k);
    ROS_WARN("[TrajSrv] Next traj starts earlier than current buffer!");
    double dt = ros::Time::now().toSec() - tns_;
    for (int i = 0; i < replan_thres_ / DELTA_T; i++) {
      double t = i * DELTA_T + dt;
      pushToTrajQueue(t, traj.getPos(t), traj.getVel(t), traj.getAcc(t),
                      Eigen::Vector3d::Zero(), 0, 0);
      // ROS_INFO("[dbg] tq: %.2f id:%d sz:%d | t: %.2f | x: %.2f y: %.2f z:
      // %.2f", tns_ - ti_ + t,
      //          traj_id_, int(traj_queue_.size()), t, traj.getPos(t)(0),
      //          traj.getPos(t)(1), traj.getPos(t)(2));
    }
    return;
  }
  traj_queue_.erase(traj_queue_.end() - k, traj_queue_.end());
  // ROS_INFO("[dbg] tc: %.2f, tns: %.2f k:%d sz:%d", ros::Time::now().toSec() -
  // ti_, tns_ - ti_, k,
  //          int(traj_queue_.size()));
  for (int i = 0; i < k; i++) {
    double t = i * DELTA_T;
    pushToTrajQueue(t, traj.getPos(t), traj.getVel(t), traj.getAcc(t),
                    Eigen::Vector3d::Zero(), 0, 0);
    // ROS_INFO("[dbg] tq: %.2f id:%d sz:%d | t: %.2f | x: %.2f y: %.2f z:
    // %.2f", tns_ - ti_ + t,
    //          traj_id_ + 1, int(traj_queue_.size()), t, traj.getPos(t)(0),
    //          traj.getPos(t)(1), traj.getPos(t)(2));
  }
}

/**
 * @brief receive parametric Bezier trajectory
 * @param msg
 */
void bezierCallback(traj_utils::BezierTrajConstPtr msg) {
  int N       = msg->order;
  int n_piece = msg->duration.size();  // number of pieces
  int R       = n_piece * (N + 1);     // number of control points

  tns_            = msg->start_time.toSec();
  ros::Time t_pub = msg->pub_time;
  ros::Time t_rcv = ros::Time::now();

  /* ----- get duration and time allocation ----- */
  double              duration = 0.0;
  std::vector<double> time_alloc;
  for (auto it = msg->duration.begin(); it != msg->duration.end(); ++it) {
    duration += (*it);
    time_alloc.push_back(*it);
  }
  duration_ = duration;

  if (time_alloc.size() != n_piece) {
    ROS_ERROR("[TrajSrv] time allocation size mismatch!");
    is_traj_received_ = false;
    return;
  }

  if (time_alloc.size() <= 0) {
    ROS_ERROR("[TrajSrv] time allocation size is zero!");
    is_traj_received_ = false;
    return;
  }

  ROS_INFO(
      "[TrajSrv] received traj(%d) at %f, | %f -> %f | pos: (%.2f, %.2f, %.2f)",
      msg->traj_id, ros::Time::now().toSec() - ti_, tns_ - ti_,
      duration + tns_ - ti_, msg->cpts.front().x, msg->cpts.front().y,
      msg->cpts.front().z);
  ROS_INFO("[TrajSrv] publish delay %f, received delay %f",
           t_pub.toSec() - tns_, (t_rcv - t_pub).toSec());

  /* ----- get control points ----- */
  Eigen::MatrixX3d cpts;
  cpts.resize(R, 3);
  for (int i = 0; i < R; ++i) {
    cpts(i, 0) = msg->cpts[i].x;
    cpts(i, 1) = msg->cpts[i].y;
    cpts(i, 2) = msg->cpts[i].z;
  }

  is_traj_received_ = true;

  next_traj_.clear();
  next_traj_.setOrder(N);
  next_traj_.setTime(time_alloc);
  next_traj_.setControlPoints(cpts);

  if (!is_triggered_) { /* if not triggered */
    ROS_INFO("[TrajSrv] not triggered yet, reset traj queue");
    tns_         = ti_;
    last_yaw_    = init_yaw_;
    last_yawdot_ = 0;
    resetTrajQueue(next_traj_);
  } else if (te_ <= tns_) { /* if the current trajectory is finished */
    ROS_INFO("[TrajSrv] current traj finished, reset traj queue");
    resetTrajQueue(next_traj_);
  } else { /* if the current trajectory is not finished */
    ROS_INFO("[TrajSrv] current traj not finished, merge traj queue");
    mergeTrajQueue(next_traj_);
  }

  traj_ = next_traj_;
  ts_   = tns_;
  te_   = ts_ + duration_;
  traj_id_++;
}

/**
 * @brief receive trigger message, start the trajectory
 * @param msg
 */
void triggerCallback(const geometry_msgs::PoseStampedPtr& msg) {
  if (is_triggered_) {
    return;
  }
  ROS_WARN("[TrajSrv] trigger received");
  is_triggered_ = true;

  ti_ = ros::Time::now().toSec();
  ts_ = ros::Time::now().toSec();
  te_ = ts_ + duration_;
}

void odomCallback(const nav_msgs::OdometryPtr& msg) {
  is_odom_received_ = true;
  odom_pos_ =
      Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                      msg->pose.pose.position.z);
  odom_q_ = Eigen::Quaterniond(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
}

void poseCallback(const geometry_msgs::PoseStampedPtr& msg) {
  is_odom_received_ = true;
  odom_pos_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y,
                              msg->pose.position.z);
  odom_q_ =
      Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,
                         msg->pose.orientation.y, msg->pose.orientation.z);
  vis_ptr_->visualizeOdom(msg);
}

/**
 * @brief publish tracking error
 *
 * @param pos_cmd
 * @param pos_real
 */
void pubTrackingError(const Eigen::Vector3d& pos_cmd,
                      const Eigen::Vector3d& pos_real) {
  geometry_msgs::PointStamped error_msg;
  error_msg.header.frame_id = "world";
  error_msg.header.stamp    = ros::Time::now();
  error_msg.point.x         = pos_cmd(0) - pos_real(0);
  error_msg.point.y         = pos_cmd(1) - pos_real(1);
  error_msg.point.z         = pos_cmd(2) - pos_real(2);
  error_pub_.publish(error_msg);
}

/**
 * @brief publish quadrotor_msgs::PositionCommand for fake simulation
 *
 * @param e
 */
void PubCallback(const ros::TimerEvent& e) {
  if (!is_odom_received_) {
    ROS_INFO("[dbg] waiting for odom");
    return;
  }
  double current_yaw =
      atan2(2 * (odom_q_.w() * odom_q_.z() + odom_q_.x() * odom_q_.y()),
            1 - 2 * (odom_q_.y() * odom_q_.y() + odom_q_.z() * odom_q_.z()));

  TrajPoint p;
  p.pos = init_pos_;
  p.vel = Eigen::Vector3d::Zero();
  p.acc = Eigen::Vector3d::Zero();

  if (!is_yaw_initilized_) { /* if yaw is not initialized, turn to desired yaw
                              */
    if (abs(init_yaw_ - current_yaw) < 0.1) {
      // ROS_INFO("[dbg] yaw: %.2f -| %.2f, no need to move yaw", current_yaw,
      //          init_yaw_);
      is_yaw_initilized_ = true;
      return;
    }
    // ROS_INFO("[dbg] t: %.2f yaw: %.2f --> %.2f", ros::Time::now().toSec(),
    //          current_yaw, init_yaw_);
    if (init_yaw_ < 0 && init_yaw_ > -3.12) {
      p.yaw     = current_yaw - MAX_YAW_CHANGE;
      p.yaw_dot = -YAW_DOT_MAX_PER_SEC;

    } else {
      p.yaw     = current_yaw + MAX_YAW_CHANGE;
      p.yaw_dot = YAW_DOT_MAX_PER_SEC;
    }
  } else {
    p.yaw     = current_yaw;
    p.yaw_dot = 0.0;
  }

  if (!is_traj_received_) {
    ROS_INFO_ONCE("[TrajSrv] waiting for trajectory");
    return;
  } else if (!is_triggered_) {
    ROS_INFO_ONCE("[TrajSrv] waiting for trigger");
    return;
  } else {
    if (traj_queue_.empty()) {
      ROS_INFO("[TrajSrv] traj queue empty");
      return;
    }
    if (traj_queue_.size() == 1) {
      p         = traj_queue_.front();
      p.vel     = Eigen::Vector3d::Zero();
      p.acc     = Eigen::Vector3d::Zero();
      p.yaw     = last_yaw_;
      p.yaw_dot = 0.0;
    } else {
      p = traj_queue_.front();
      traj_queue_.pop_front();
      fillTrajQueue();
    }
  }

  // ROS_INFO("[dbg] t: %.2f | pos: %.2f %.2f %.2f | vel: %.2f %.2f %.2f",
  //          p.t.toSec() - ti_, p.pos.x(), p.pos.y(), p.pos.z(), p.vel.x(),
  //          p.vel.y(), p.vel.z());

  publishCmd(p.pos, p.vel, p.acc, p.yaw, p.yaw_dot);
  publishPVA(p.pos, p.vel, p.acc, p.yaw, p.yaw_dot);
  /** publish tracking error */
  pubTrackingError(p.pos, odom_pos_);
  return;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "poly_traj_server");
  ros::NodeHandle nh("~");

  double init_qx, init_qy, init_qz, init_qw;
  nh.param("init_x", init_pos_(0), 0.0);
  nh.param("init_y", init_pos_(1), 0.0);
  nh.param("init_z", init_pos_(2), 0.0);
  nh.param("init_qw", init_qw, 1.0);
  nh.param("init_qx", init_qx, 0.0);
  nh.param("init_qy", init_qy, 0.0);
  nh.param("init_qz", init_qz, 0.0);
  nh.param("offset", offset_, 0.00);
  nh.param("replan_threshold", replan_thres_, 0.5);

  nh.param("is_init_yaw", is_init_yaw_needed_,
           false);  // correct yaw angle while initializing

  ros::Timer      cmd_timer = nh.createTimer(ros::Duration(0.01), PubCallback);
  ros::Subscriber traj_sub  = nh.subscribe("trajectory", 1, bezierCallback);
  ros::Subscriber trigger_sub =
      nh.subscribe("/traj_start_trigger", 10, triggerCallback);
  ros::Subscriber pose_sub = nh.subscribe("pose", 10, poseCallback);
  ros::Subscriber odom_sub = nh.subscribe("odom", 10, odomCallback);
  pva_pub_ =
      nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/pva_setpoint", 1);
  pos_cmd_pub_ =
      nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 1);

  error_pub_ = nh.advertise<geometry_msgs::PointStamped>("error", 1);

  vis_ptr_.reset(new TrajSrvVisualizer(nh));

  init_yaw_ = atan2(2.0 * (init_qw * init_qz + init_qx * init_qy),
                    1.0 - 2.0 * (init_qy * init_qy + init_qz * init_qz));

  ros::Duration(3.0).sleep();

  ROS_INFO("[TrajSrv]: ready to receive trajectory");

  ti_ = ros::Time::now().toSec();
  ts_ = ros::Time::now().toSec();
  te_ = ros::Time::now().toSec();

  traj_id_ = 0;

  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
