/**
 * @file risk_base.h
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-10-23
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef RISK_BASE_H
#define RISK_BASE_H

#include <plan_env/map.h>
// #include <traj_coordinator/mader.hpp>
#include <traj_coordinator/particle.hpp>

#include <ros/ros.h>
#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Float32MultiArray.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

class RiskBase : public MapBase {
 private:
  /* Data */

  /* Parameters */
  float observation_stddev_;
  float localization_stddev_;
  float num_newborn_particles_;
  float risk_threshold_astar_;
  float risk_thres_reg_decay_;
  float risk_thres_vox_decay_;

  /* Map */
  float risk_maps_[VOXEL_NUM][PREDICTION_TIMES];

  /* Multi Agents */
  bool             is_multi_agents_ = false;
  ParticleATC::Ptr coordinator_;

  /* ROS */
  ros::Subscriber future_risk_sub_, pose_sub_, odom_sub_, time_sub_;
  bool            is_updating_map_ = false;

  /* Utilities */

 public:
  RiskBase() {}
  ~RiskBase() {}

  void init(ros::NodeHandle &nh);
  void setCoordinator(ParticleATC::Ptr ptr) {
    is_multi_agents_ = true;
    coordinator_     = ptr;
  }
  void addOtherAgents();

  inline void            setMapCenter(const Eigen::Vector3f &center) { pose_ = center; }
  inline Eigen::Vector3f getMapCenter() const { return pose_; }

  inline void               setQuaternion(const Eigen::Quaternionf &q) { q_ = q; }
  inline Eigen::Quaternionf getQuaternion() const { return q_; }
  inline void               getQuaternion(Eigen::Quaternionf &q) const { q = q_; }

  ros::Time getMapTime() const { return last_update_time_; }

  void publishMap();
  void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
    MapBase::odomCallback(odom_msg);
  }
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {
    MapBase::poseCallback(pose_msg);
  }
  void futureRiskCallback(const std_msgs::Float32MultiArrayConstPtr &future_risk);
  void updateTimeCallback(const std_msgs::Header &header) { last_update_time_ = header.stamp; }

  int getClearOcccupancy(const Eigen::Vector3d &pos) const;
  int getClearOcccupancy(const Eigen::Vector3d &pos, int t) const;
  int getClearOcccupancy(const Eigen::Vector3d &pos, double t) const;

  void addObstaclesToRiskMap(const std::vector<Eigen::Vector3d> &centers, int t_index);
  void addParticlesToRiskMap(const std::vector<Eigen::Vector3d> &centers,
                             const std::vector<float>           &risks,
                             int                                 t_index);
  void addParticlesToRiskMap(const std::vector<Eigen::Vector3f> &centers,
                             const std::vector<float>           &risks,
                             int                                 t_index);
  void getObstaclePoints(std::vector<Eigen::Vector3d> &points);
  void getObstaclePoints(std::vector<Eigen::Vector3d> &points, double t_start, double t_end);
  void getObstaclePoints(std::vector<Eigen::Vector3d> &points,
                         double                        t_start,
                         double                        t_end,
                         const Eigen::Vector3d        &lc,
                         const Eigen::Vector3d        &hc);

  typedef std::shared_ptr<RiskBase> Ptr;
};
#endif /* RISK_BASE_H */
