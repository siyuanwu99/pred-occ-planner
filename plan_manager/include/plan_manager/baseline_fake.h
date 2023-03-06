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
#include <plan_env/fake_dsp_map.h>
#include <sensor_msgs/PointCloud2.h>
#include <traj_utils/BezierTraj.h>
#include <bernstein/bezier_optimizer.hpp>
// #include <plan_manager/mader_deconfliction.hpp>
#include <sfc_gen/firi.hpp>
#include <traj_coordinator/mader.hpp>
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
  double opt_max_vel    = 3.0;
  double opt_max_acc    = 4.0;
  double delta_corridor = 0.3;
  double init_range     = 1.0;
  double min_volumn     = 0.5;
  double corridor_tau   = 0.4f; /* time span for corridor generation */
  /* New */
  FakeBaselineParameters(const ros::NodeHandle &nh) {
    nh.getParam("planner/corridor_tau", corridor_tau);
    nh.getParam("corridor/init_range", init_range);
    nh.getParam("corridor/min_volumn", min_volumn);

    nh.getParam("optimizer/max_vel_optimization", opt_max_vel);
    nh.getParam("optimizer/max_acc_optimization", opt_max_acc);
    nh.getParam("optimizer/delta_corridor", delta_corridor);
  }
};

/**
 * @brief baseline planner with ground truth map for testing and debugging
 *
 */
class FakeBaselinePlanner {
 public:
  FakeBaselinePlanner(ros::NodeHandle &nh, const FakeBaselineParameters &params)
      : nh_(nh), cfg_(params) {}
  ~FakeBaselinePlanner() {}

  void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void clickCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

  void init();
  bool plan();

  void showAstarPath();

  void setGoal(const Eigen::Vector3d &goal) { goal_pos_ = goal; }

  bool              getMapStatus() { return is_map_updated_; }
  bool              getOdomStatus() { return is_odom_received_; }
  Eigen::Vector3d   getPos() const { return odom_pos_; }
  Bernstein::Bezier getTrajectory() const { return traj_; }

  void getTrajStartTime(ros::Time &start_time) const { start_time = t_start_; }

  typedef std::shared_ptr<FakeBaselinePlanner> Ptr;

 private:
  /* Helper function */
  void showObstaclePoints(const std::vector<Eigen::Vector3d> &points);
  void addAgentsTrajectoryToMap();
  void setEmptyTrajectory();

 private:
  /* ROS */
  ros::NodeHandle        nh_;
  ros::Subscriber        click_sub_, pose_sub_, swarm_traj_sub_;
  ros::Publisher         obstacle_pub_;
  ros::Time              t_start_;
  FakeBaselineParameters cfg_;

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
  MADER::Ptr               collision_avoider_; /* multi-agent collision avoidance policy*/
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

#endif  // _BASELINE_FAKE_H_
