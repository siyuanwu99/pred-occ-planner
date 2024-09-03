/**
 * @file risk_base.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-10-23
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <plan_env/map_parameters.h>
#include <plan_env/risk_base.h>

void RiskBase::init(ros::NodeHandle &nh) {
  nh_ = nh;

  /* Parameters */
  loadParameters();
  nh_.param("map/risk_threshold_region", risk_threshold_astar_, 1.2F);
  nh_.param("map/risk_threshold_region_decay", risk_thres_reg_decay_, 0.2F);
  nh_.param("map/risk_threshold_voxel_decay", risk_thres_vox_decay_, 0.2F);

  resolution_           = VOXEL_RESOLUTION;
  local_update_range_x_ = MAP_LENGTH_VOXEL_NUM / 2 * resolution_;
  local_update_range_y_ = MAP_WIDTH_VOXEL_NUM / 2 * resolution_;
  local_update_range_z_ = MAP_HEIGHT_VOXEL_NUM / 2 * resolution_;

  /* initialize inflated kernel */
  inf_step_ = clearance_ / resolution_;
  inflate_kernel_.reserve((2 * inf_step_ + 1) * (2 * inf_step_ + 1) * (2 * inf_step_ + 1));
  for (int x = -inf_step_; x <= inf_step_; x++) {
    for (int y = -inf_step_; y <= inf_step_; y++) {
      for (int z = -inf_step_; z <= inf_step_; z++) {
        inflate_kernel_.push_back(Eigen::Vector3i(x, y, z));
      }
    }
  }
  ROS_INFO("[RiskMap] Inflated kernel size: %d", (int)inflate_kernel_.size());

  future_risk_sub_ = nh_.subscribe("map/future_risk", 1, &RiskBase::futureRiskCallback, this);
  time_sub_        = nh.subscribe("map/time", 1, &RiskBase::updateTimeCallback, this);

  if (is_pose_sub_) {
    pose_sub_ = nh_.subscribe("map/pose", 1, &RiskBase::poseCallback, this);
  } else {
    odom_sub_ = nh_.subscribe("map/odom", 1, &RiskBase::odomCallback, this);
  }

  cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("map/occupancy_inflated", 1, true);

  /* initialize odometry */
  pose_ = Eigen::Vector3f::Zero();
  q_    = Eigen::Quaternionf::Identity();

  last_update_time_ = ros::Time::now();
}

void RiskBase::futureRiskCallback(const std_msgs::Float32MultiArrayConstPtr &future_risk) {
  ros::Time t1     = ros::Time::now();
  is_updating_map_ = true;
  int s            = future_risk->layout.dim[0].stride;
  for (int i = 0; i < VOXEL_NUM; i++) {
    for (int j = 0; j < PREDICTION_TIMES; j++) {
      risk_maps_[i][j] = future_risk->data[i * s + j];
    }
  }

  if (is_multi_agents_) {
    addOtherAgents();
  }
  is_updating_map_  = false;
  float last_update = future_risk->data[VOXEL_NUM * PREDICTION_TIMES + 3];
  // last_update_time_ = ros::Time::now();

  ROS_INFO("[RiskMap] Updating map takes %f ms at %f", (ros::Time::now() - t1).toSec() * 1000,
           last_update_time_.toSec());
  publishMap();
}

void RiskBase::publishMap() {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  cloud->points.reserve(VOXEL_NUM);
  if (if_pub_spatio_temporal_map_) {
    for (int i = 0; i < VOXEL_NUM; i++) {
      pcl::PointXYZI  pt;
      Eigen::Vector3f pos = getVoxelPosition(i);
      if (abs(pos.z() - pose_.z()) > 0.5) continue;  // filter out necessary points
      for (int j = 0; j < PREDICTION_TIMES; j++) {
        if (risk_maps_[i][j] > risk_threshold_ - j * risk_thres_vox_decay_) {
          pt.x         = pos.x();
          pt.y         = pos.y();
          pt.z         = pos.z() + j * 1.5f;
          pt.intensity = risk_maps_[i][j];
          cloud->points.push_back(pt);
        }
      }
    }
  } else {
    for (int i = 0; i < VOXEL_NUM; i++) {
      pcl::PointXYZI  pt;
      Eigen::Vector3f pos = getVoxelPosition(i);
      for (int j = 0; j < PREDICTION_TIMES; j++) {
        if (risk_maps_[i][j] > risk_threshold_) {
          pt.x = pos.x();
          pt.y = pos.y();
          pt.z = pos.z();
          cloud->points.push_back(pt);
        }
      }
    }
  }
  cloud->width  = cloud->points.size();
  cloud->height = 1;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.stamp    = ros::Time::now();
  cloud_msg.header.frame_id = "world";
  cloud_pub_.publish(cloud_msg);
}

/**
 * @brief add occupancies of other agents to the map
 */
void RiskBase::addOtherAgents() {
  std::vector<bool> is_swarm_traj_valid;
  is_swarm_traj_valid.resize(coordinator_->getNumAgents(), true);
  int ego_id          = coordinator_->getEgoID();
  int n_rbts          = coordinator_->getNumAgents();
  int n_ego_particles = coordinator_->getEgoParticlesNum();

  for (int t_idx = 0; t_idx < PREDICTION_TIMES; t_idx++) {
    std::vector<Eigen::Vector3d> particles;
    particles.clear();
    particles.reserve(n_rbts * n_ego_particles * 20);
    std::vector<float> risks;
    risks.clear();
    risks.reserve(n_rbts * n_ego_particles * 20);

    double t = last_update_time_.toSec() + time_resolution_ * t_idx;
    for (int i = 0; i < n_rbts; i++) {
      if (i == ego_id || !is_swarm_traj_valid[i]) {
        continue;
      }
      bool is_traj_valid     = coordinator_->getWaypoints(particles, i, t);
      is_swarm_traj_valid[i] = is_traj_valid;
    }

    /* filter */
    for (auto &pt : particles) {
      pt = pt - pose_.cast<double>();
    }
    addObstaclesToRiskMap(particles, t_idx);
  }
}

/**
 * @brief create ego particles voxel in the map coordinate, to reduce time when collision check
 */
// void RiskBase::createEgoParticlesVoxel() {
//   /* add ego particles to the map */
//   std::vector<Eigen::Vector3d> particles = coordinator_->getEgoParticles();
//   if (particles.size() != inflate_kernel_.size() && particles.size() != 0) {
//     inflate_kernel_.clear();
//     for (auto &pt : particles) {
//       Eigen::Vector3f ptf = pt.cast<float>();
//       inflate_kernel_.push_back(getVoxelRelIndex(ptf));
//     }
//     ROS_INFO("[MAMapUpdate]: inflate kernel size: %lu", inflate_kernel_.size());
//   }
// }

/**
 * @brief add particles to map
 * @param pts  a list of particles
 * @param t_index time index
 */
void RiskBase::addObstaclesToRiskMap(const std::vector<Eigen::Vector3d> &pts, int t_index) {
  for (auto &pt : pts) {
    Eigen::Vector3f ptf = pt.cast<float>();
    if (!isInRange(ptf)) continue;
    int index                  = getVoxelIndex(ptf);
    risk_maps_[index][t_index] = 1.0;
  }
}
void RiskBase::addParticlesToRiskMap(const std::vector<Eigen::Vector3d> &pts,
                                     const std::vector<float>           &risks,
                                     int                                 t_index) {
  for (int i = 0; i < pts.size(); i++) {
    Eigen::Vector3f ptf = pts[i].cast<float>();
    if (!isInRange(ptf)) continue;
    int index = getVoxelIndex(ptf);
    risk_maps_[index][t_index] += risks[i];
  }
}

void RiskBase::addParticlesToRiskMap(const std::vector<Eigen::Vector3f> &pts,
                                     const std::vector<float>           &risks,
                                     int                                 t_index) {
  for (int i = 0; i < pts.size(); i++) {
    Eigen::Vector3f ptf = pts[i];
    if (!isInRange(ptf)) continue;
    int index = getVoxelIndex(ptf);
    risk_maps_[index][t_index] += risks[i];
  }
}

/**
 * @brief collision check on voxel map with clearance (none-inflated map)
 *
 * @param pos position in world frame
 * @param t index of the time frame
 * @return 0: free, 1: occupied, -1: out of bound
 */
int RiskBase::getClearOcccupancy(const Eigen::Vector3d &pos, int t) const {
  if (pos.z() < ground_height_) return 1;
  if (pos.z() > ceiling_height_) return 1;

  Eigen::Vector3f pos_f = pos.cast<float>() - pose_;
  Eigen::Vector3i pos_i = getVoxelRelIndex(pos_f);
  if (!isInRange(pos_i)) return -1;
  // if (risk_maps_[getVoxelIndex(pos_i)][t] > risk_threshold_) return 1;
  // return 0;

  float sum_risk = 0.0;
  // std::cout << "inflate kernel size: " << inflate_kernel_.size() << std::endl;
  for (auto &ego_i : inflate_kernel_) {
    Eigen::Vector3i p = pos_i + ego_i;
    // std::cout << "pos_i: " << pos_i.transpose() << ", ego_i: " << ego_i.transpose()
    //           << ", p: " << p.transpose() << std::endl;
    if (!isInRange(p)) continue;
    int idx = getVoxelIndex(p);
    sum_risk += risk_maps_[idx][t];
    if (sum_risk > risk_threshold_astar_ - t * risk_thres_reg_decay_) return 1; /* collision */
  }
  return 0;
}

int RiskBase::getClearOcccupancy(const Eigen::Vector3d &pos) const {
  return getClearOcccupancy(pos, 0);
}

int RiskBase::getClearOcccupancy(const Eigen::Vector3d &pos, double dt) const {
  int tf = floor(dt / time_resolution_);
  tf     = tf > (PREDICTION_TIMES - 1) ? PREDICTION_TIMES - 1 : tf;
  return getClearOcccupancy(pos, tf);
}

/**
 * @brief get the obstacle points in the time interval and the cube
 * @param points : output
 * @param t_start : start time
 * @param t_end: end time
 */
void RiskBase::getObstaclePoints(std::vector<Eigen::Vector3d> &points) {
  points.clear();
  for (int i = 0; i < VOXEL_NUM; i++) {
    if (risk_maps_[i][0] > risk_threshold_) {
      Eigen::Vector3f pt = getVoxelPosition(i);
      points.push_back(pt.cast<double>());
    }
  }
}

void RiskBase::getObstaclePoints(std::vector<Eigen::Vector3d> &points,
                                 double                        t_start,
                                 double                        t_end) {
  points.clear();
  int idx_start = floor(t_start / time_resolution_);
  int idx_end   = ceil(t_end / time_resolution_);
  for (int i = 0; i < VOXEL_NUM; i++) {
    for (int j = idx_start; j < idx_end; j++) {
      if (risk_maps_[i][j] > risk_threshold_) {
        Eigen::Vector3f pt = getVoxelPosition(i);
        points.push_back(pt.cast<double>());
        break;
      }
    }
  }
}

void RiskBase::getObstaclePoints(std::vector<Eigen::Vector3d> &points,
                                 double                        t_start,
                                 double                        t_end,
                                 const Eigen::Vector3d        &lower_corner,
                                 const Eigen::Vector3d        &higher_corner) {
  double t0 = last_update_time_.toSec();

  int idx_start = floor((t_start - t0) / time_resolution_);
  int idx_end   = ceil((t_end - t0) / time_resolution_);
  idx_start     = idx_start < 0 ? 0 : idx_start;
  idx_start     = idx_start > PREDICTION_TIMES ? PREDICTION_TIMES : idx_start;
  idx_end       = idx_end > PREDICTION_TIMES ? PREDICTION_TIMES : idx_end;
  idx_end       = idx_end < 0 ? 0 : idx_end;

  // std::cout << "idx_start" << idx_start << "idx_end" << idx_end << std::endl;
  int lx = (lower_corner[0] - pose_.x() + local_update_range_x_) / resolution_;
  int ly = (lower_corner[1] - pose_.y() + local_update_range_y_) / resolution_;
  int lz = (lower_corner[2] - pose_.z() + local_update_range_z_) / resolution_;
  int hx = (higher_corner[0] - pose_.x() + local_update_range_x_) / resolution_;
  int hy = (higher_corner[1] - pose_.y() + local_update_range_y_) / resolution_;
  int hz = (higher_corner[2] - pose_.z() + local_update_range_z_) / resolution_;

  hx = std::min(hx, MAP_LENGTH_VOXEL_NUM - 1);
  hy = std::min(hy, MAP_WIDTH_VOXEL_NUM - 1);
  hz = std::min(hz, MAP_HEIGHT_VOXEL_NUM - 1);
  lx = std::max(lx, 0);
  ly = std::max(ly, 0);
  lz = std::max(lz, 0);

  for (int z = lz; z <= hz; z++) {
    for (int y = ly; y <= hy; y++) {
      for (int x = lx; x <= hx; x++) {
        int i = x + y * MAP_LENGTH_VOXEL_NUM + z * MAP_LENGTH_VOXEL_NUM * MAP_WIDTH_VOXEL_NUM;
        for (int j = idx_start; j <= idx_end; j++) {
          if (risk_maps_[i][j] > risk_threshold_ - risk_thres_vox_decay_ * j) {
            Eigen::Vector3d pt = getVoxelPosition(i).cast<double>();
            points.emplace_back(pt);
          }
        }
      }
    }
  }
}
