/**
 * @file multi_baseline.h
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-07-29
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef _MULTI_BASELINE_H_
#define _MULTI_BASELINE_H_

#include <geometry_msgs/TwistStamped.h>
#include <polynomial/mini_snap.h>
#include <traj_utils/PolyTraj.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <decomp_ros_msgs/DynPolyhedronArray.h>

#include <chrono>
#include <ctime>
#include <memory>
#include <queue>
#include <termcolor.hpp>  // for colored output
#include <traj_utils/poly_traj.hpp>
#include <vector>

#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "risk_aware_kinodynamic_a_star.h"
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64.h"
#include "std_msgs/UInt8.h"
#include "traj_utils/visualizer.hpp"

namespace planner {

struct PlannerConfig {
  /* data */
  double max_vel              = 3.0;
  double max_acc              = 4.0;
  double max_vel_optimization = 3.0;
  double max_acc_optimization = 4.0;
  double delta_corridor       = 0.3;

  bool  use_height_limit = true;
  float height_limit_max = 2.2f;
  float height_limit_min = 0.f;
  bool  sample_z_acc     = true;

  float a_star_acc_sample_step  = 2.f;
  float a_star_search_time_step = 0.4f;
  float expand_safety_distance  = 0.2f;

  float risk_threshold_motion_primitive = 0.15;
  float risk_threshold_single_voxel     = 0.15;
  float risk_threshold_corridor         = 2.5;
  float risk_threshold_replan           = 20;

  int trajectory_piece_max_size = 12;
  int nmpc_receive_points_num   = 20;

  double planning_time_step           = 0.05;
  double max_differentiated_current_a = 4.0;
  bool   is_rviz_map_center_locked    = false;

  double goal_x                = 60.0;
  double goal_y                = 0.0;
  double goal_z                = 1.5;
  double waypoint_distance     = 4.0;
  double goal_reach_threshold  = 1.0;
  double replan_time_threshold = 0.5;

  PlannerConfig(const ros::NodeHandle &nh) {
    nh.getParam("planner/p_goal_x", goal_x);
    nh.getParam("planner/p_goal_y", goal_y);
    nh.getParam("planner/p_goal_z", goal_z);
    nh.getParam("planner/max_vel", max_vel);
    nh.getParam("planner/max_acc", max_acc);
    nh.getParam("planner/goal_reach_threshold", goal_reach_threshold);
    nh.getParam("planner/risk_threshold_replan", risk_threshold_replan);
    nh.getParam("planner/replan_time_threshold", replan_time_threshold);
    nh.getParam("planner/max_differentiated_current_a", max_differentiated_current_a);
    nh.getParam("planner/planning_time_step", planning_time_step);
    nh.getParam("planner/trajectory_piece_max_size", trajectory_piece_max_size);

    nh.getParam("optimizer/max_vel_optimization", max_vel_optimization);
    nh.getParam("optimizer/max_acc_optimization", max_acc_optimization);
    nh.getParam("optimizer/delta_corridor", delta_corridor);

    nh.getParam("astar/use_height_limit", use_height_limit);
    nh.getParam("astar/height_limit_max", height_limit_max);
    nh.getParam("astar/height_limit_min", height_limit_min);
    nh.getParam("astar/sample_z_acc", sample_z_acc);
    nh.getParam("astar/a_star_acc_sample_step", a_star_acc_sample_step);
    nh.getParam("astar/a_star_search_time_step", a_star_search_time_step);
    nh.getParam("astar/risk_threshold_motion_primitive", risk_threshold_motion_primitive);
    nh.getParam("astar/expand_safety_distance", expand_safety_distance);
    nh.getParam("astar/nmpc_receive_points_num", nmpc_receive_points_num);

    nh.getParam("corridor/risk_threshold_single_voxel", risk_threshold_single_voxel);
    nh.getParam("corridor/risk_threshold_corridor", risk_threshold_corridor);

    nh.getParam("waypoint_distance", waypoint_distance);
    nh.getParam("rviz_map_center_locked", is_rviz_map_center_locked);
  }
};

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

class Planner {
 private:
  /********** FINITE STATE MACHINE **********/
  FSM_STATUS _status;

  /********** TRAJECTORY PLANNING **********/
  polynomial::CorridorMiniSnap::Ptr _traj_optimizer; /** Trajectory optimizer */
  polynomial::Trajectory            _traj;           /** Trajectory */
  int                               _traj_idx;       /** Trajectory index */
  ros::Time                         _last_plan_time;
  ros::Time                         _traj_start_time;
  ros::Time                         _traj_end_time;
  double                            _traj_duration;

  Astar _astar_planner; /** A* path finding */

  Eigen::Vector3d    _goal;     /** Goal position */
  Eigen::Vector3d    _odom_pos; /** quadrotor's current position */
  Eigen::Vector3d    _odom_vel; /** quadrotor's current velocity */
  Eigen::Vector3d    _odom_acc; /** quadrotor's current acceleration */
  Eigen::Quaternionf _odom_att; /** quadrotor's current attitude as a quaternion */

  double _prev_pt, _prev_px, _prev_py, _prev_pz; /** previous point */
  double _prev_vt, _prev_vx, _prev_vy, _prev_vz; /** previous velocity */
  double _prev_opt_end_time;                     /// previous trajectory end time

  /********** MAP **********/
  int             _map_x_limit, _map_y_limit, _map_z_limit; /** Map limits */
  int             _map_size;
  float           _map_half_length, _map_half_width, _map_half_height;
  Eigen::Vector3d _map_center; /** Map center */

  /********** ROS UTILS **********/
  ros::NodeHandle _nh;
  ros::Timer      _traj_timer;
  ros::Subscriber _future_risk_sub, _pose_sub, _vel_sub, _trigger_sub;
  ros::Publisher  _traj_pub, _corridor_pub;

  /********** CONFIG **********/
  PlannerConfig _config;
  float         _ref_direction_angle;

  /********** DATA **********/
  double _traj_planning_start_time;
  int    _drone_id;

  /** @brief Risk map in map frame. usage: [spatial_index][temporal_index] */
  float                       _future_risk[VOXEL_NUM][RISK_MAP_NUMBER];
  std::queue<Eigen::Vector3d> _waypoints; /** Waypoint queue */

  Eigen::Vector3d _p_store_for_em, _v_store_for_em, _a_store_for_em;

  /********** BOOLEANS **********/
  bool _is_future_risk_updated;
  bool _is_future_risk_locked;
  bool _is_safety_mode_enabled;
  bool _is_local_frame;
  bool _is_state_locked;
  bool _is_exec_triggered;
  bool _is_odom_received;
  bool _is_velocity_received;
  bool _is_goal_received;

  /********** VISUALIZATIONS **********/
  visualizer::Visualizer::Ptr _vis;

 public:
  Planner(ros::NodeHandle &nh, const PlannerConfig &conf) : _nh(nh), _config(conf) {}
  ~Planner() {}

  void init();

  int    getPointSpatialIndexInMap(const Eigen::Vector3d &p, const Eigen::Vector3d &c);
  bool   OptimizationInCorridors(const std::vector<Eigen::Matrix<double, 6, -1>> &c,
                                 const std::vector<double> &                      t,
                                 const Eigen::Matrix3d &                          init,
                                 const Eigen::Matrix3d &                          final);
  bool   checkTrajectoryRisk(const polynomial::Trajectory &traj);
  double getMaxRisk(const polynomial::Trajectory &traj);
  double getTotalRisk(const polynomial::Trajectory &traj);

  void publishTrajectory(const polynomial::Trajectory &traj);
  void publishCorridor(const vector<Corridor *> &c);

  void FutureRiskCallback(const std_msgs::Float32MultiArrayConstPtr &risk_msg);
  void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void TriggerCallback(const geometry_msgs::PoseStampedPtr &msg);
  void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void VelCallback(const geometry_msgs::TwistStamped &msg);

  /********** STATE MACHINE **********/
  void FSMPrintState(FSM_STATUS new_state);
  void FSMChangeState(FSM_STATUS new_state);
  void FSMCallback(const ros::TimerEvent &event);

  bool localReplan(PLAN_TYPE type);
  bool globalPlan();
  bool setLocalGoal();
  bool executeTrajectory();
  bool checkTimeLapse(double time);

  /********** HELPER FUNCTIONS **********/
  inline bool isGoalReached(const Eigen::Vector3d &p, const Eigen::Vector3d &g);
  inline bool isInputLost();
  inline int  getDroneID();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<Planner> Ptr;
};

}  // namespace planner

#endif  // _MULTI_BASELINE_H_