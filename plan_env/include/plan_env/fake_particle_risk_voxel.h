/**
 * @file fake_particle_risk_voxel.h
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief subscribe ground truth map and velocity,
 *       generate fake map prediction
 * @version 1.0
 * @date 2023-07-28
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef FAKE_RISK_VOXEL_H
#define FAKE_RISK_VOXEL_H

#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <plan_env/map.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <memory>
#include <traj_coordinator/particle.hpp>
#include <vector>

#undef PREDICTION_TIMES
#define PREDICTION_TIMES 9

struct Cylinder {
  int    type;
  double x;   // x coordinate
  double y;   // y coordinate
  double z;   // y coordinate
  double w;   // width
  double h;   // height
  double vx;  // velocity x
  double vy;  // velocity y
  double qw;
  double qx;  // quaternion x
  double qy;  // quaternion y
  double qz;  // quaternion z
};

/**
 * @brief Class for generating fake map prediction, which is inherited from ParticleRiskVoxel
 */
class FakeParticleRiskVoxel : public MapBase {
 public:
  FakeParticleRiskVoxel() {}
  ~FakeParticleRiskVoxel() {}
  void init(ros::NodeHandle &nh);
  void setCoordinator(ParticleATC::Ptr ptr) {
    is_multi_agents_ = true;
    coordinator_     = ptr;
  }
  void groundTruthStateCallback(const visualization_msgs::MarkerArray::ConstPtr &state_msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
    MapBase::odomCallback(odom_msg);
  }
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {
    MapBase::poseCallback(pose_msg);
  }
  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg) {
    MapBase::cloudCallback(cloud_msg);
  }
  void updateMap(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg);
  void pubCallback(const ros::TimerEvent &event) { MapBase::pubCallback(event); }
  void addObstaclesToRiskMap(const std::vector<Eigen::Vector3d> &centers, int t_index);
  void addParticlesToRiskMap(const std::vector<Eigen::Vector3d> &centers,
                             const std::vector<float>           &risks,
                             int                                 t_index);
  void addParticlesToRiskMap(const std::vector<Eigen::Vector3f> &centers,
                             const std::vector<float>           &risks,
                             int                                 t_index);

  int getClearOcccupancy(const Eigen::Vector3d &pos) const;
  int getClearOcccupancy(const Eigen::Vector3d &pos, int t) const;
  int getClearOcccupancy(const Eigen::Vector3d &pos, double t) const;

  // void getObstaclePoints(std::vector<Eigen::Vector3d> &points,
  //                        double                        t_start,
  //                        double                        t_end,
  //                        const Eigen::Vector3d        &lower_corner,
  //                        const Eigen::Vector3d        &higher_corner);
  //
  void createEgoParticlesVoxel();

  typedef std::shared_ptr<FakeParticleRiskVoxel> Ptr;

 private:
  bool            is_multi_agents_ = false;
  ros::Subscriber gt_map_sub_;    // ground truth local map
  ros::Subscriber gt_state_sub_;  // ground truth velocity

  /* Data Variables */
  std::vector<Cylinder> gt_cylinders_;
  ParticleATC::Ptr      coordinator_;  // TODO(01.10): move this to risk_voxel
};

#endif  // FAKE_RISK_VOXEL_H
