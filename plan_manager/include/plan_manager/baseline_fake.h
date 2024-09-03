/**
 * @file baseline_fake.h
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-10-24
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef _BASELINE_FAKE_H_
#define _BASELINE_FAKE_H_

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <path_searching/fake_risk_hybrid_a_star.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <plan_env/fake_dsp_map.h>
// #include <plan_env/risk_voxel.h>
#include <plan_env/fake_particle_risk_voxel.h>
#include <sensor_msgs/PointCloud2.h>
#include <traj_utils/BezierTraj.h>
#include <bernstein/bezier_optimizer.hpp>
// #include <plan_manager/mader_deconfliction.hpp>
#include <sfc_gen/firi.hpp>
// #include <traj_coordinator/mader.hpp>
#include <sfc_gen/sdlp.hpp>
#include <traj_coordinator/particle.hpp>
#include <traj_utils/bernstein.hpp>
#include <traj_utils/corridor.hpp>
#include <traj_utils/visualizer.hpp>

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <string>
#include <termcolor.hpp>  // for colored output
#include <vector>
// #include <plan_manager/baseline.h>  // include BaselineParameters
struct FakeBaselineParameters {
  /* data */
  double max_vel        = 3.0;
  double max_acc        = 6.0;
  double opt_max_vel    = 3.0;
  double opt_max_acc    = 4.0;
  double delta_corridor = 0.3;
  double init_range     = 1.0;
  double min_volumn     = 0.5;
  double shrink_size    = 0.2;

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

  FakeBaselineParameters(const ros::NodeHandle &nh) {
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
    nh.getParam("corridor/min_volumn", min_volumn);
    nh.getParam("corridor/shrink_size", shrink_size);

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
class FakeBaselinePlanner {
 public:
  FakeBaselinePlanner(ros::NodeHandle              &nh2,
                      ros::NodeHandle              &nh3,
                      ros::NodeHandle              &nh4,
                      const FakeBaselineParameters &params)
      : nh_planner_(nh2), nh_coordinator_(nh3), nh_map_(nh4), cfg_(params) {}
  ~FakeBaselinePlanner() {}

  void init();
  bool replan(double                 t,
              const Eigen::Vector3d &start_pos,
              const Eigen::Vector3d &start_vel,
              const Eigen::Vector3d &start_acc,
              const Eigen::Vector3d &goal_pos);
  void setStartTime(double t) { prev_traj_start_time_ = t; }

  void showAstarPath();

  Bernstein::Bezier getTrajectory() const { return traj_; }

  bool isTrajSafe(double);

  inline bool            isPrevTrajFinished(double t) const;
  inline double          getPrevTrajStartTime() const;
  inline double          getPrevTrajEndTime() const;
  inline Eigen::Vector3d getPos(double t) const;
  inline Eigen::Vector3d getVel(double t) const;
  inline Eigen::Vector3d getAcc(double t) const;

  /* Visualization */
  visualizer::Visualizer::Ptr visualizer_;

  typedef std::shared_ptr<FakeBaselinePlanner> Ptr;

 private:
  /* Helper function */
  Eigen::Matrix<double, 6, 4> getInitCorridor(const Eigen::Vector3d &left_higher_corner,
                                              const Eigen::Vector3d &right_lower_corner);

  bool checkCorridorValidity(const Eigen::MatrixX4d &corridor);
  bool checkCorridorIntersect(const Eigen::MatrixX4d &corridor1, const Eigen::MatrixX4d &corridor2);
  bool checkGoalReachability(const Eigen::MatrixX4d &corridor,
                             const Eigen::Vector3d  &start_pos,
                             Eigen::Vector3d        &goal_pos);
  void ShrinkCorridor(Eigen::MatrixX4d &corridor);
  void ShrinkCorridor(Eigen::MatrixX4d &corridor, const Eigen::Vector3d &path);
  void showObstaclePoints(const std::vector<Eigen::Vector3d> &points);
  void showObstaclePoints(const std::vector<Eigen::Vector4d> &points, ros::Publisher &pub);
  void addAgentsTrajectoryToMap();
  void setEmptyTrajectory(const Eigen::Vector3d &pos);

 private:
  /* ROS */
  ros::NodeHandle nh_map_, nh_coordinator_, nh_planner_;
  ros::Publisher  obstacle_pub_;

  FakeBaselineParameters cfg_;

  /* Shared Pointers */
  FakeParticleRiskVoxel::Ptr map_;
  FakeRiskHybridAstar::Ptr   a_star_;
  traj_opt::BezierOpt::Ptr   traj_optimizer_;    /** Trajectory optimizer */
  ParticleATC::Ptr           collision_avoider_; /* multi-agent collision avoidance policy*/

  /* Trajectory */
  int    traj_idx_;        /** Trajectory index */
  double traj_start_time_; /** current trajectory start time */
  double prev_traj_start_time_;

  Bernstein::Bezier traj_; /** Trajectory */
};

inline Eigen::Vector3d FakeBaselinePlanner::getPos(double t) const {
  return traj_.getPos(t - prev_traj_start_time_);
}

inline Eigen::Vector3d FakeBaselinePlanner::getVel(double t) const {
  return traj_.getVel(t - prev_traj_start_time_);
}

inline Eigen::Vector3d FakeBaselinePlanner::getAcc(double t) const {
  return traj_.getAcc(t - prev_traj_start_time_);
}

inline bool FakeBaselinePlanner::isPrevTrajFinished(double t) const {
  return (t - prev_traj_start_time_ > traj_.getDuration());
}

inline double FakeBaselinePlanner::getPrevTrajStartTime() const { return prev_traj_start_time_; }

inline double FakeBaselinePlanner::getPrevTrajEndTime() const {
  return prev_traj_start_time_ + traj_.getDuration();
}

#endif  // _BASELINE_FAKE_H_
