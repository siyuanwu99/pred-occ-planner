/**
 * @file plan_manager.h
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-11-01
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef PLAN_MANAGER_H
#define PLAN_MANAGER_H
#include <plan_manager/baseline.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <queue>
#include <string>
#include <termcolor.hpp>  // for colored output
#include <vector>

typedef Bernstein::Bezier      Trajectory;
typedef traj_utils::BezierTraj TrajMsg;

/**
 * @brief finite state machine for multi planning
 *
 */
enum class FSM_STATUS {
  INIT,
  WAIT_TARGET,
  NEW_PLAN,
  REPLAN,
  EXEC_TRAJ,
  EMERGENCY_REPLAN,
  GOAL_REACHED,
  EXIT
};

enum PLAN_TYPE {
  NEW,      /* plan a new trajectory from current position */
  CONTINUE, /* continue the current trajectory from the final position */
  EMERGENCY /* emergency replan from current position */
};

struct FSMParameters {
  double goal_tolerance    = 1.0;
  double replan_tolerance  = 1.0;
  double replan_duration   = 0.1;
  double replan_start_time = 0.4;

  int    replan_max_failures  = 3;
  double colli_check_duration = 2.0;
};

class FiniteStateMachine {
 public:
  FiniteStateMachine(ros::NodeHandle &nh1,
                     ros::NodeHandle &nh2,
                     ros::NodeHandle &nh3,
                     ros::NodeHandle &nh4)
      : nh1_(nh1), nh2_(nh2), nh3_(nh3), nh4_(nh4) {}
  ~FiniteStateMachine() = default;

  void run();

  void FSMCallback(const ros::TimerEvent &event);
  void TriggerCallback(const geometry_msgs::PoseStampedPtr &msg);
  void PoseCallback(const geometry_msgs::PoseStampedPtr &msg);
  void OdomCallback(const nav_msgs::OdometryPtr &msg);
  void clickCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void visCallback(const ros::TimerEvent &event);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  void FSMPrintState(FSM_STATUS new_state);
  void FSMChangeState(FSM_STATUS new_state);

  void publishTrajectory();
  void publishEmptyTrajectory();

  bool setLocalGoal();
  bool executeTrajectory();
  bool isTrajectorySafe();

  /********** HELPER FUNCTIONS **********/
  inline int  getDroneID();
  inline bool isGoalReached(const Eigen::Vector3d &p);
  inline bool isInputLost();
  inline bool checkTimeLapse(double time);

  FSM_STATUS    status_;
  FSMParameters cfgs_;

  int drone_id_;
  int traj_idx_;
  int num_replan_failures_;

  /********** BOOLEANS **********/
  bool is_map_updated_;
  bool is_future_risk_locked_;
  bool is_safety_mode_enabled_;
  bool is_local_frame_;
  bool is_state_locked_;
  bool is_exec_triggered_;
  bool is_odom_received_;
  bool is_velocity_received_;
  bool is_goal_preset_;
  bool is_goal_received_;
  bool is_success_;
  bool is_pose_subscribed_;

  /* ROS */
  ros::NodeHandle nh1_, nh2_, nh3_, nh4_;
  ros::Subscriber trigger_sub_, click_sub_, pose_sub_, odom_sub_, swarm_traj_sub_;
  ros::Timer      fsm_timer_, vis_timer_;
  ros::Publisher  traj_pub_, broadcast_traj_pub_;

  ros::Time traj_start_time_;

  /* planner */
  BaselinePlanner::Ptr planner_;

  /* odometry */
  Eigen::Vector3d    odom_pos_;       /** quadrotor's current position */
  Eigen::Vector3d    odom_vel_;       /** quadrotor's current velocity */
  Eigen::Vector3d    odom_acc_;       /** quadrotor's current acceleration */
  Eigen::Quaterniond odom_att_;       /** quadrotor's current attitude as a quaternion */
  Eigen::Vector3d    goal_pos_;       /** quadrotor's goal position */
  Eigen::Vector3d    prev_odom_pos_;  /** quadrotor's previous position */
  Eigen::Vector3d    prev_odom_vel_;  /** quadrotor's previous velocity */
  double             prev_odom_time_; /** quadrotor's previous time */

  /* trajectory */
  std::queue<Eigen::Vector3d> waypoints_;
};

/********** INLINE FUNCTIONS **********/
/**
 * @brief check if the given position close to the goal by checking euclidean distance
 *
 * @param p Eigen::Vector3d given position
 * @param g Eigen::Vector3d goal position
 * @return true input position is close to the goal
 * @return false
 */
inline bool FiniteStateMachine ::isGoalReached(const Eigen::Vector3d &p) {
  return ((p - goal_pos_).norm() < cfgs_.goal_tolerance) ? true : false;
}

/**
 * @brief if future risk map or odometry is not updating, return true
 * @return true   no update
 * @return false
 */
inline bool FiniteStateMachine::isInputLost() { return !is_odom_received_; }

/**
 * @brief read drone ID from the ros node handle
 * @return int drone ID
 */
inline int FiniteStateMachine::getDroneID() {
  std::string name = ros::this_node::getNamespace();
  int         id   = name.substr(name.size() - 1, 1).c_str()[0] - '0';
  // std::cout << "|" << name << "| " << id << std::endl;
  return id;
}

inline bool FiniteStateMachine::checkTimeLapse(double time) {
  double elapsed = ros::Time::now().toSec() - traj_start_time_.toSec();
  return (elapsed > time);
}

#endif  // PLAN_MANAGER_H
