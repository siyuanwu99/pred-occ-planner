/**
 * @file map.h
 * @author Siyuan Wu
 * @brief
 * @version 1.0
 * @date 2023-02-14
 *
 * @copyright Copyright (c) 2020
 *
 */
#ifndef MAP_H
#define MAP_H
#include <ros/ros.h>
#include <Eigen/Dense>
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
#include "map_parameters.h"  // MAP PARAMETRES

class MapBase {
 protected:
  /* ROS Utilities */
  ros::NodeHandle nh_;
  ros::Subscriber click_sub_;
  ros::Publisher  cloud_pub_;
  ros::Publisher  risk_pub_;
  ros::Publisher  obstacle_pub_; /* Debug */
  ros::Timer      pub_timer_;
  ros::Time       last_update_time_;

  /* Data */
  Eigen::Vector3f                     pose_;
  Eigen::Quaternionf                  q_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;

  float risk_maps_[VOXEL_NUM][PREDICTION_TIMES];
  float valid_clouds_[5000 * 3];

  /* Parameters */
  bool  if_pub_spatio_temporal_map_;
  bool  if_pub_in_world_frame_;
  bool  is_odom_local_;
  float resolution_;
  float time_resolution_;
  float local_update_range_x_;
  float local_update_range_y_;
  float local_update_range_z_;
  float init_x_  = 0.0;
  float init_y_  = 0.0;
  float init_z_  = 0.0;
  float init_qx_ = 0.0;
  float init_qy_ = 0.0;
  float init_qz_ = 0.0;
  float init_qw_ = 1.0;

  float filter_res_;
  float risk_threshold_;
  float clearance_;
  int   inf_step_;

  std::vector<Eigen::Vector3i> inflate_kernel_;

  /* Message filters */
  bool            is_pose_sub_ = false;
  ros::Subscriber odom_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber cloud_sub_;

  /* Utilities */
  inline bool            isInRange(const Eigen::Vector3f &p) const;
  inline bool            isInRange(const Eigen::Vector3i &p) const;
  inline int             getVoxelIndex(const Eigen::Vector3f &pos) const;
  inline int             getVoxelIndex(const Eigen::Vector3i &pos) const;
  inline Eigen::Vector3f getVoxelPosition(int index) const;
  inline Eigen::Vector3f getVoxelRelPosition(int index) const;
  inline Eigen::Vector3i getVoxelRelIndex(const Eigen::Vector3f &pos) const;

 public:
  MapBase() {}
  ~MapBase() {}

  virtual void init(ros::NodeHandle &nh);
  virtual void loadParameters();
  virtual void publishMap();
  virtual void updateMap(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg);

  void inflateMap();
  void filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr       &cloud_out,
                        float                                     *valid_clouds,
                        int                                       &valid_clouds_num);

  void pubCallback(const ros::TimerEvent &event);
  void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg);
  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg);

  ros::Time              getMapTime() const { return last_update_time_; }
  inline void            setMapCenter(const Eigen::Vector3f &center) { pose_ = center; }
  inline Eigen::Vector3f getMapCenter() const { return pose_; }

  inline void               setQuaternion(const Eigen::Quaternionf &q) { q_ = q; }
  inline Eigen::Quaternionf getQuaternion() const { return q_; }
  inline void               getQuaternion(Eigen::Quaternionf &q) const { q = q_; }

  void getObstaclePoints(std::vector<Eigen::Vector3d> &points);
  void getObstaclePoints(std::vector<Eigen::Vector3d> &points, double t_start, double t_end);
  void getObstaclePoints(std::vector<Eigen::Vector3d> &points,
                         double                        t_start,
                         double                        t_end,
                         const Eigen::Vector3d        &lc,
                         const Eigen::Vector3d        &hc);
  int  getInflateOccupancy(const Eigen::Vector3d &pos) const;
  int  getInflateOccupancy(const Eigen::Vector3d &pos, int t) const;
  int  getInflateOccupancy(const Eigen::Vector3d &pos, double t) const;

  int getClearOcccupancy(const Eigen::Vector3d &pos) const;
  int getClearOcccupancy(const Eigen::Vector3d &pos, int t) const;
  int getClearOcccupancy(const Eigen::Vector3d &pos, double t) const;

  void addObstacles(const std::vector<Eigen::Vector3d> &centers,
                    const Eigen::Vector3d              &size,
                    int                                 t_index);
  void addObstacles(const std::vector<Eigen::Vector3d> &centers,
                    const Eigen::Vector3d              &size,
                    const ros::Time                    &t);

  typedef std::shared_ptr<MapBase> Ptr;
};

/* ====================== definition of inline function ====================== */

inline bool MapBase::isInRange(const Eigen::Vector3f &p) const {
  return p.x() > -local_update_range_x_ && p.x() < local_update_range_x_ &&
         p.y() > -local_update_range_y_ && p.y() < local_update_range_y_ &&
         p.z() > -local_update_range_z_ && p.z() < local_update_range_z_;
}

inline bool MapBase::isInRange(const Eigen::Vector3i &p) const {
  return p.x() >= 0 && p.x() < MAP_LENGTH_VOXEL_NUM && p.y() >= 0 && p.y() < MAP_WIDTH_VOXEL_NUM &&
         p.z() >= 0 && p.z() < MAP_HEIGHT_VOXEL_NUM;
}

/**
 * @brief get the index of the voxel in the world frame
 * @param pos point in map frame
 * @return index
 */
inline int MapBase::getVoxelIndex(const Eigen::Vector3f &pos) const {
  int x = (pos[0] + local_update_range_x_) / resolution_;
  int y = (pos[1] + local_update_range_y_) / resolution_;
  int z = (pos[2] + local_update_range_z_) / resolution_;
  return z * MAP_LENGTH_VOXEL_NUM * MAP_WIDTH_VOXEL_NUM + y * MAP_LENGTH_VOXEL_NUM + x;
}

inline int MapBase::getVoxelIndex(const Eigen::Vector3i &pos) const {
  return pos.z() * MAP_LENGTH_VOXEL_NUM * MAP_WIDTH_VOXEL_NUM + pos.y() * MAP_LENGTH_VOXEL_NUM +
         pos.x();
}

/**
 * @brief
 * @param index
 * @return position of the voxel in the world frame
 */
inline Eigen::Vector3f MapBase::getVoxelPosition(int index) const {
  int x = index % MAP_LENGTH_VOXEL_NUM;
  int y = (index / MAP_LENGTH_VOXEL_NUM) % MAP_WIDTH_VOXEL_NUM;
  int z = index / (MAP_LENGTH_VOXEL_NUM * MAP_WIDTH_VOXEL_NUM);
  return Eigen::Vector3f(x * resolution_ - local_update_range_x_,
                         y * resolution_ - local_update_range_y_,
                         z * resolution_ - local_update_range_z_) +
         pose_;
}

/**
 * @brief get voxel index in local frame
 *
 * @param pos index in global frame
 */
inline Eigen::Vector3i MapBase::getVoxelRelIndex(const Eigen::Vector3f &pos) const {
  int x = (pos[0] + local_update_range_x_) / resolution_;
  int y = (pos[1] + local_update_range_y_) / resolution_;
  int z = (pos[2] + local_update_range_z_) / resolution_;
  return Eigen::Vector3i(x, y, z);
}

inline Eigen::Vector3f MapBase::getVoxelRelPosition(int index) const {
  int x = index % MAP_LENGTH_VOXEL_NUM;
  int y = (index / MAP_LENGTH_VOXEL_NUM) % MAP_WIDTH_VOXEL_NUM;
  int z = index / (MAP_LENGTH_VOXEL_NUM * MAP_WIDTH_VOXEL_NUM);
  return Eigen::Vector3f(x * resolution_ - local_update_range_x_,
                         y * resolution_ - local_update_range_y_,
                         z * resolution_ - local_update_range_z_);
}

#endif  // MAP_H
