/**
 * @file risk_voxel.h
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-10-23
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef RISK_VOXEL_H
#define RISK_VOXEL_H

#include <plan_env/dsp_dynamic.h>
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

#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>

class RiskVoxel : public MapBase {
 private:
  /* Data */
  dsp_map::DSPMap::Ptr                dsp_map_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;

  /* Parameters */
  float observation_stddev_;
  float localization_stddev_;
  float num_newborn_particles_;
  float risk_threshold_astar_;

  /* Multi Agents */
  bool             is_multi_agents_ = false;
  ParticleATC::Ptr coordinator_;

  /* Message filters */
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
                                                          nav_msgs::Odometry>
      SyncPolicyCloudOdom;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
                                                          geometry_msgs::PoseStamped>
                                                                              SyncPolicyCloudPose;
  typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyCloudPose>> SynchronizerCloudPose;
  typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyCloudOdom>> SynchronizerCloudOdom;

  std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>>         odom_sub_;
  std::shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>>   cloud_sub_;

  SynchronizerCloudOdom sync_cloud_odom_;
  SynchronizerCloudPose sync_cloud_pose_;

  /* Utilities */

 public:
  RiskVoxel() {}
  ~RiskVoxel() {}

  void init(ros::NodeHandle &nh);
  void setCoordinator(ParticleATC::Ptr ptr) {
    is_multi_agents_ = true;
    coordinator_     = ptr;
  }
  void publishMap();
  void inflateMap();
  void addOtherAgents();

  void updateMap(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg);

  inline void            setMapCenter(const Eigen::Vector3f &center) { pose_ = center; }
  inline Eigen::Vector3f getMapCenter() const { return pose_; }

  inline void               setQuaternion(const Eigen::Quaternionf &q) { q_ = q; }
  inline Eigen::Quaternionf getQuaternion() const { return q_; }
  inline void               getQuaternion(Eigen::Quaternionf &q) const { q = q_; }

  ros::Time getMapTime() const { return last_update_time_; }

  void pubCallback(const ros::TimerEvent &event);
  void cloudPoseCallback(const sensor_msgs::PointCloud2::ConstPtr   &cloud_msg,
                         const geometry_msgs::PoseStamped::ConstPtr &pose_msg);
  void cloudOdomCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg,
                         const nav_msgs::Odometry::ConstPtr       &odom_msg);

  int getInflateOccupancy(const Eigen::Vector3d &pos) const;
  int getInflateOccupancy(const Eigen::Vector3d &pos, int t) const;
  int getInflateOccupancy(const Eigen::Vector3d &pos, double t) const;

  int getClearOcccupancy(const Eigen::Vector3d &pos) const;
  int getClearOcccupancy(const Eigen::Vector3d &pos, int t) const;
  int getClearOcccupancy(const Eigen::Vector3d &pos, double t) const;

  // void createEgoParticlesVoxel();
  void addObstaclesToRiskMap(const std::vector<Eigen::Vector3d> &centers, int t_index);
  void addParticlesToRiskMap(const std::vector<Eigen::Vector3d> &centers,
                             const std::vector<float>           &risks,
                             int                                 t_index);
  void addParticlesToRiskMap(const std::vector<Eigen::Vector3f> &centers,
                             const std::vector<float>           &risks,
                             int                                 t_index);

  typedef std::shared_ptr<RiskVoxel> Ptr;
};
#endif /* RISK_VOXEL_H */
