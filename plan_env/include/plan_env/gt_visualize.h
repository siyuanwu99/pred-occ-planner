/**
 * @file gt_visalize.h
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief a better visualization with ground truth data
 * @version 1.0
 * @date 2022-10-23
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef GT_VIS_H
#define GT_VIS_H

#include <plan_env/map.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <memory>
#include <string>
#include <traj_coordinator/particle.hpp>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float32MultiArray.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

class GTVisualize : public MapBase {
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
  ros::NodeHandle nh2_;
  ros::Subscriber future_risk_sub_, pose_sub_, odom_sub_, time_sub_;
  ros::Publisher  cloud_pub_, ground_pub_;
  ros::Publisher  cloud_pub_1_, cloud_pub_2_, cloud_pub_3_, cloud_pub_4_, cloud_pub_5_;
  ros::Time       time_now_;
  bool            is_updating_map_ = false;

  /* Utilities */

 public:
  GTVisualize() {}
  ~GTVisualize() {}

  void init(ros::NodeHandle &nh1, ros::NodeHandle &nh2);
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

  void publishMapI(int i, const ros::Publisher &pub);
  void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
    time_now_ = odom_msg->header.stamp;
    MapBase::odomCallback(odom_msg);
  }
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {
    time_now_ = pose_msg->header.stamp;
    MapBase::poseCallback(pose_msg);
  }
  void futureRiskCallback(const std_msgs::Float32MultiArrayConstPtr &future_risk);
  void updateTimeCallback(const std_msgs::Header &header) { last_update_time_ = header.stamp; }

  // int getClearOcccupancy(const Eigen::Vector3d &pos) const;
  // int getClearOcccupancy(const Eigen::Vector3d &pos, int t) const;
  // int getClearOcccupancy(const Eigen::Vector3d &pos, double t) const;

  void addObstaclesToRiskMap(const std::vector<Eigen::Vector3d> &centers, int t_index);
  void addParticlesToRiskMap(const std::vector<Eigen::Vector3d> &centers,
                             const std::vector<float>           &risks,
                             int                                 t_index);
  void addParticlesToRiskMap(const std::vector<Eigen::Vector3f> &centers,
                             const std::vector<float>           &risks,
                             int                                 t_index);
  // void getObstaclePoints(std::vector<Eigen::Vector3d> &points);
  // void getObstaclePoints(std::vector<Eigen::Vector3d> &points, double t_start, double t_end);
  // void getObstaclePoints(std::vector<Eigen::Vector3d> &points,
  //                        double                        t_start,
  //                        double                        t_end,
  //                        const Eigen::Vector3d        &lc,
  //                        const Eigen::Vector3d        &hc);
  //
  std_msgs::ColorRGBA getColorJet(double v, double vmin, double vmax);
  std_msgs::ColorRGBA getColorG2B(double v, double vmin, double vmax);

  typedef std::shared_ptr<GTVisualize> Ptr;
};
#endif /* GT_VIS_H */
