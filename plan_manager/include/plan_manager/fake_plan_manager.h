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
#include <plan_manager/baseline_fake.h>
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
  double goal_tolerance   = 1.0;
  double replan_tolerance = 1.0;
  double replan_duration  = 0.1;
};

class FiniteStateMachineFake {
 public:
  FiniteStateMachineFake(ros::NodeHandle &nh) : _nh(nh) {}
  ~FiniteStateMachineFake() = default;

  void run();

  void FSMCallback(const ros::TimerEvent &event);
  void TriggerCallback(const geometry_msgs::PoseStampedPtr &msg);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  void FSMPrintState(FSM_STATUS new_state);
  void FSMChangeState(FSM_STATUS new_state);

  void publishTrajectory();

  // bool localReplan(PLAN_TYPE type);
  // bool globalPlan();
  bool setLocalGoal();
  bool executeTrajectory();
  bool isTrajectorySafe();

  /********** HELPER FUNCTIONS **********/
  inline int  getDroneID();
  inline bool isGoalReached(const Eigen::Vector3d &p);
  inline bool isInputLost();
  inline bool checkTimeLapse(double time);

  FSM_STATUS    _status;
  FSMParameters _cfgs;

  int    _drone_id;
  int    _traj_idx;
  double _time;

  /********** BOOLEANS **********/
  bool _is_map_updated;
  bool _is_future_risk_locked;
  bool _is_safety_mode_enabled;
  bool _is_local_frame;
  bool _is_state_locked;
  bool _is_exec_triggered;
  bool _is_odom_received;
  bool _is_velocity_received;
  bool _is_goal_received;

  /* ROS */
  ros::NodeHandle _nh;
  ros::Subscriber _trigger_sub;
  ros::Timer      _fsm_timer;
  ros::Publisher  _traj_pub, _broadcast_traj_pub;

  ros::Time _traj_start_time;
  ros::Time _prev_plan_time;

  /* planner */
  FakeBaselinePlanner::Ptr _planner;

  /* trajectory */
  Eigen::Vector3d             _goal;
  Eigen::Vector3d             _odom_pos;
  std::queue<Eigen::Vector3d> _waypoints;
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
inline bool FiniteStateMachineFake ::isGoalReached(const Eigen::Vector3d &p) {
  return ((p - _goal).norm() < _cfgs.goal_tolerance) ? true : false;
}

/**
 * @brief if future risk map or odometry is not updating, return true
 * @return true   no update
 * @return false
 */
inline bool FiniteStateMachineFake::isInputLost() {
  _is_map_updated   = _planner->getMapStatus();
  _is_odom_received = _planner->getOdomStatus();
  return !_is_map_updated || !_is_odom_received;
}

/**
 * @brief read drone ID from the ros node handle
 * @return int drone ID
 */
inline int FiniteStateMachineFake::getDroneID() {
  std::string name = ros::this_node::getNamespace();
  int         id   = name.substr(name.size() - 1, 1).c_str()[0] - '0';
  // std::cout << "|" << name << "| " << id << std::endl;
  return id;
}

inline bool FiniteStateMachineFake::checkTimeLapse(double time) {
  double elapsed = ros::Time::now().toSec() - _prev_plan_time.toSec();
  return (elapsed > time);
}

#endif  // PLAN_MANAGER_H
