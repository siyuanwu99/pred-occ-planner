/**
 * @file fake_rmader.h
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-10-24
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef _BASELINE_H_
#define _BASELINE_H_

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <path_searching/fake_risk_hybrid_a_star.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <plan_env/fake_dsp_map.h>
#include <plan_env/risk_voxel.h>
#include <sensor_msgs/PointCloud2.h>
#include <traj_utils/BezierTraj.h>
#include <bernstein/bezier_optimizer.hpp>
// #include <plan_manager/mader_deconfliction.hpp>
#include <traj_utils/bernstein.hpp>
#include <traj_utils/corridor.hpp>
#include <traj_utils/visualizer.hpp>

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <string>
#include <termcolor.hpp>  // for colored output
#include <vector>
// #include <plan_manager/baseline.h>  // include RmaderParameters
struct RmaderParameters {
  /* data */
  double max_vel        = 3.0;
  double max_acc        = 4.0;
  double opt_max_vel    = 3.0;
  double opt_max_acc    = 4.0;
  double delta_corridor = 0.3;
  double init_range     = 1.0;

  bool  use_height_limit = true;
  float height_limit_max = 2.2f;
  float height_limit_min = 0.f;
  bool  sample_z_acc     = true;

  float  a_star_acc_sample_step = 2.f;
  double corridor_tau           = 0.4f; /* time span for corridor generation */
  float  expand_safety_distance = 0.2f;

  float risk_threshold_motion_primitive = 0.15;
  float risk_threshold_single_voxel     = 0.15;
  float risk_threshold_corridor         = 2.5;
  float risk_threshold_replan           = 20;

  int trajectory_piece_max_size = 12;
  int nmpc_receive_points_num   = 20;

  double planning_time_step           = 0.05;
  double max_differentiated_current_a = 4.0;
  bool   is_rviz_map_center_locked    = false;

  double waypoint_distance     = 4.0;
  double goal_reach_threshold  = 1.0;
  double replan_time_threshold = 0.5;

  /* New */

  RmaderParameters(const ros::NodeHandle &nh) {
    nh.getParam("planner/max_vel", max_vel);
    nh.getParam("planner/max_acc", max_acc);
    nh.getParam("planner/corridor_tau", corridor_tau);
    nh.getParam("planner/goal_reach_threshold", goal_reach_threshold);
    nh.getParam("planner/risk_threshold_replan", risk_threshold_replan);
    nh.getParam("planner/replan_time_threshold", replan_time_threshold);
    nh.getParam("planner/max_differentiated_current_a", max_differentiated_current_a);
    nh.getParam("planner/planning_time_step", planning_time_step);
    nh.getParam("planner/trajectory_piece_max_size", trajectory_piece_max_size);
    nh.getParam("corridor/init_range", init_range);

    nh.getParam("optimizer/max_vel_optimization", opt_max_vel);
    nh.getParam("optimizer/max_acc_optimization", opt_max_acc);
    nh.getParam("optimizer/delta_corridor", delta_corridor);

    nh.getParam("astar/use_height_limit", use_height_limit);
    nh.getParam("astar/height_limit_max", height_limit_max);
    nh.getParam("astar/height_limit_min", height_limit_min);
    nh.getParam("astar/sample_z_acc", sample_z_acc);
    nh.getParam("astar/a_star_acc_sample_step", a_star_acc_sample_step);
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
 * @brief baseline planner with ground truth map for testing and debugging
 *
 */
class FakeRmaderPlanner {
 public:
  FakeRmaderPlanner(ros::NodeHandle &nh, const RmaderParameters &params) : nh_(nh), cfg_(params) {}
  ~FakeRmaderPlanner() {}

  void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void clickCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

  void init();
  bool plan();

  void showAstarPath();

  void setGoal(const Eigen::Vector3d &goal) { goal_pos_ = goal; }

  bool              getMapStatus() { return is_map_updated_; }
  bool              getOdomStatus() { return is_odom_received_; }
  bool              delayCheck(const Bernstein::Bezier &traj);
  Eigen::Vector3d   getPos() const { return odom_pos_; }
  Bernstein::Bezier getTrajectory() const { return traj_; }

  void getTrajStartTime(ros::Time &start_time) const { start_time = t_start_; }

  typedef std::shared_ptr<FakeRmaderPlanner> Ptr;

 private:
  /* Helper function */
  void showObstaclePoints(const std::vector<Eigen::Vector3d> &points);
  void addAgentsTrajectoryToMap();
  void setEmptyTrajectory();

 private:
  /* ROS */
  ros::NodeHandle  nh_;
  ros::Subscriber  click_sub_, pose_sub_, swarm_traj_sub_;
  ros::Publisher   obstacle_pub_;
  ros::Time        t_start_;
  RmaderParameters cfg_;

  Eigen::Vector3d    odom_pos_; /** quadrotor's current position */
  Eigen::Vector3d    odom_vel_; /** quadrotor's current velocity */
  Eigen::Vector3d    odom_acc_; /** quadrotor's current acceleration */
  Eigen::Vector3d    goal_pos_; /** quadrotor's goal position */
  Eigen::Quaterniond odom_att_; /** quadrotor's current attitude as a quaternion */

  double prev_pt_, prev_px_, prev_py_, prev_pz_; /** previous point */
  double prev_vt_, prev_vx_, prev_vy_, prev_vz_; /** previous velocity */
  double prev_opt_end_time_;                     /** previous trajectory end time */

  /* Shared Pointers */
  FakeRiskVoxel::Ptr       map_;
  FakeRiskHybridAstar::Ptr a_star_;
  traj_opt::BezierOpt::Ptr traj_optimizer_;    /** Trajectory optimizer */
  RMADER::Ptr              collision_avoider_; /* multi-agent collision avoidance policy*/
  /* Trajectory */
  Bernstein::Bezier traj_;     /** Trajectory */
  int               traj_idx_; /** Trajectory index */

  /* Waypoints */

  /* Booleans */
  bool is_state_locked_;      /** State lock */
  bool is_velocity_received_; /** Velocity received */
  bool is_odom_received_;     /** Odom received */
  bool is_map_updated_;       /** Map updated */

  /* Visualization */
  visualizer::Visualizer::Ptr visualizer_;
};

#endif  // _BASELINE_H_
