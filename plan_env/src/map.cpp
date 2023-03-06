/**
 * @file fake_dsp_map.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-11-22
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <plan_env/map.h>

void MapBase::loadParameters() {
  /* parameters */
  nh_.param("init_x", init_x_, 0.0F);
  nh_.param("init_y", init_y_, 0.0F);
  nh_.param("init_z", init_z_, 0.0F);
  nh_.param("init_qx", init_qx_, 0.0F);
  nh_.param("init_qy", init_qy_, 0.0F);
  nh_.param("init_qz", init_qz_, 0.0F);
  nh_.param("init_qw", init_qw_, 1.0F);
  nh_.param("is_odom_local", is_odom_local_, false);

  nh_.param("map/booleans/sub_pose", is_pose_sub_, false);
  nh_.param("map/booleans/pub_world_frame", if_pub_in_world_frame_, true);
  nh_.param("map/booleans/pub_spatio_temporal", if_pub_spatio_temporal_map_, false);
  nh_.param("map/resolution", filter_res_, 0.15F); /* resolution of the voxel filter */
  nh_.param("map/time_resolution", time_resolution_, 0.2F);
  nh_.param("map/local_update_range_x", local_update_range_x_, 5.0F);
  nh_.param("map/local_update_range_y", local_update_range_y_, 5.0F);
  nh_.param("map/local_update_range_z", local_update_range_z_, 4.0F);
  nh_.param("map/risk_threshold", risk_threshold_, 0.2F);
  nh_.param("map/clearance", clearance_, 0.3F);
}

void MapBase::init(ros::NodeHandle &nh) {
  nh_ = nh;
  loadParameters();
  resolution_           = VOXEL_RESOLUTION;
  local_update_range_x_ = MAP_LENGTH_VOXEL_NUM / 2 * resolution_;
  local_update_range_y_ = MAP_WIDTH_VOXEL_NUM / 2 * resolution_;
  local_update_range_z_ = MAP_HEIGHT_VOXEL_NUM / 2 * resolution_;
  ROS_INFO("[MAP_BASE] Local update range: %f, %f, %f", local_update_range_x_,
           local_update_range_y_, local_update_range_z_);
  ROS_INFO("[MAP_BASE] Init fake risk voxel map");

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
  ROS_INFO("[MAP_BASE] Inflated kernel size: %d", (int)inflate_kernel_.size());

  // cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  // cloud_->points.reserve(80000);

  /* subscribers */
  cloud_sub_ = nh_.subscribe("map/cloud", 1, &MapBase::cloudCallback, this);

  if (is_pose_sub_) {
    pose_sub_ = nh_.subscribe("map/pose", 1, &MapBase::poseCallback, this);
  } else {
    odom_sub_ = nh_.subscribe("map/odom", 1, &MapBase::odomCallback, this);
  }

  /* publishers */
  cloud_pub_    = nh.advertise<sensor_msgs::PointCloud2>("map/occupancy_inflated", 1, true);
  obstacle_pub_ = nh.advertise<sensor_msgs::PointCloud2>("vis_obstacle", 1, true);
  /* publish point clouds in 20 Hz */
  pub_timer_ = nh.createTimer(ros::Duration(0.10), &MapBase::pubCallback, this);

  last_update_time_ = ros::Time::now();

  /* initialize odometry */
  pose_ = Eigen::Vector3f::Zero();
  q_    = Eigen::Quaternionf::Identity();
}

/**
 * @brief filter point clouds, keep points in range
 *
 * @param cloud_in input point cloud in camera frame
 * @param cloud_out output point cloud in local frame
 * @param valid_clouds array of valid points in local frame
 * @param valid_clouds_num number of valid points
 */
void MapBase::filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr       &cloud_out,
                               float                                     *valid_clouds,
                               int                                       &valid_clouds_num) {
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud_in);
  sor.setLeafSize(filter_res_, filter_res_, filter_res_);
  sor.filter(*cloud_out);

  /* filter points which are too far from the drone */
  for (int i = 0; i < (int)cloud_out->points.size(); i++) {
    float           x = cloud_out->points[i].z;
    float           y = -cloud_out->points[i].x;
    float           z = -cloud_out->points[i].y;
    Eigen::Vector3f p(x, y, z);
    if (isInRange(p)) {
      valid_clouds[valid_clouds_num * 3 + 0] = x;
      valid_clouds[valid_clouds_num * 3 + 1] = y;
      valid_clouds[valid_clouds_num * 3 + 2] = z;
      valid_clouds_num++;
      if (valid_clouds_num >= 5000) {
        break;
      }
    }
  }
}

/**
 * @brief
 *
 * @param pose_msg
 */
void MapBase::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {
  pose_ = Eigen::Vector3f(pose_msg->pose.position.x, pose_msg->pose.position.y,
                          pose_msg->pose.position.z);
  q_    = Eigen::Quaternionf(pose_msg->pose.orientation.w, pose_msg->pose.orientation.x,
                             pose_msg->pose.orientation.y, pose_msg->pose.orientation.z);
  if (is_odom_local_) {
    Eigen::Vector3f offset(init_x_, init_y_, init_z_);
    pose_ += offset;
    Eigen::Quaternionf rotation(init_qw_, init_qx_, init_qy_, init_qz_);
    q_ = rotation * q_;
  }
}

/**
 * @brief
 *
 * @param odom_msg
 */
void MapBase::odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
  pose_ = Eigen::Vector3f(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y,
                          odom_msg->pose.pose.position.z);
  q_    = Eigen::Quaternionf(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
                             odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);
  if (is_odom_local_) {
    Eigen::Vector3f offset(init_x_, init_y_, init_z_);
    pose_ += offset;
    Eigen::Quaternionf rotation(init_qw_, init_qx_, init_qy_, init_qz_);
    q_ = rotation * q_;
  }
}

void MapBase::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg) {
  updateMap(cloud_msg);
}

/**
 * @brief publish point clouds in a fixed frequency
 *
 * @param event
 */
void MapBase::pubCallback(const ros::TimerEvent &event) { publishMap(); }

void MapBase::publishMap() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->points.reserve(VOXEL_NUM);
  int num_occupied = 0;

  std::string st_msg  = (if_pub_spatio_temporal_map_) ? "true" : "false";
  std::string wrd_msg = (if_pub_in_world_frame_) ? "true" : "false";

  if (if_pub_spatio_temporal_map_) {
    for (int i = 0; i < VOXEL_NUM; i++) {
      pcl::PointXYZ   pt;
      Eigen::Vector3f pos = getVoxelPosition(i);
      if (pos.z() < -1) continue;  // filter out the ground
      for (int j = 0; j < PREDICTION_TIMES; j++) {
        if (risk_maps_[i][j] > risk_threshold_) {
          pt.x = pos.x();
          pt.y = pos.y();
          pt.z = j * time_resolution_;
          cloud->points.push_back(pt);
        }
      }
    }
  } else {
    for (int i = 0; i < VOXEL_NUM; i++) {
      Eigen::Vector3f pt = getVoxelPosition(i);
      pcl::PointXYZ   p;
      if (risk_maps_[i][0] > risk_threshold_) {
        p.x = pt[0];
        p.y = pt[1];
        p.z = pt[2];
        cloud->points.push_back(p);
      }
    }
  }

  if (if_pub_in_world_frame_ && !if_pub_spatio_temporal_map_) {
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translate(pose_);
    // transform.rotate(q_);
    pcl::transformPointCloud(*cloud, *cloud, transform);
  }

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.stamp    = ros::Time::now();
  cloud_msg.header.frame_id = "world";
  cloud_pub_.publish(cloud_msg);

  // clock_t t2 = clock();
  // std::cout << "num_occupied: " << num_occupied << "\t" << std::endl;
  //           << "publish time (ms): " << (t2 - t1) * 1000 / (double)CLOCKS_PER_SEC << std::endl;
}

/**
 * @brief obstacle inflation
 *
 */
void MapBase::inflateMap() {
  ros::Time t1 = ros::Time::now();

  int                          inf_steps = (2 * clearance_) / VOXEL_RESOLUTION + 1;
  std::vector<Eigen::Vector3f> inf_pts(std::pow(inf_steps, 3));

  /* create convolution kernel */
  int n = 0;

  for (float xx = -clearance_; xx <= clearance_; xx += VOXEL_RESOLUTION) {
    for (float yy = -clearance_; yy <= clearance_; yy += VOXEL_RESOLUTION) {
      for (float zz = -clearance_; zz <= clearance_; zz += VOXEL_RESOLUTION) {
        inf_pts[n++] = Eigen::Vector3f(xx, yy, zz);
      }
    }
  }

  /* select points */
  std::vector<std::pair<int, int>> obs_idx_list(VOXEL_NUM * PREDICTION_TIMES);
  for (int i = 0; i < VOXEL_NUM; i++) {
    for (int j = 0; j < PREDICTION_TIMES; j++) {
      if (risk_maps_[i][j] > risk_threshold_) {
        std::pair<int, int> obs_idx = std::make_pair(i, j);
        obs_idx_list.push_back(obs_idx);
      }
    }
  }

  for (auto &idx : obs_idx_list) {
    Eigen::Vector3f pos = getVoxelRelPosition(idx.first);
    int             i   = idx.first;
    int             j   = idx.second;
    for (auto &inf_pt : inf_pts) {
      Eigen::Vector3f pos_inf = pos + inf_pt;
      if (!isInRange(pos_inf)) continue;
      int idx = getVoxelIndex(pos_inf);
      if (idx < 0 || idx >= VOXEL_NUM) continue;
      if (risk_maps_[i][j] > risk_threshold_) continue;
      risk_maps_[i][j] = 0.5F;
    }
  }

  ros::Time t2 = ros::Time::now();
  ROS_INFO("convolution time: %f ms", (t2 - t1).toSec() * 1000);
}

void MapBase::updateMap(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg) {
  ROS_INFO("[MAP_BASE] update map");
  last_update_time_ = ros::Time::now();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*cloud_msg, *cloud_in);

  int                                 n_valid = 0;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
  filterPointCloud(cloud_in, cloud_filtered, valid_clouds_, n_valid);

  /* UPDATE */
}

/**
 * @brief check if the point is in obstacle
 * @param pos: point position in world frame
 * @return -1: out of range, 0: not in obstacle, 1: in obstacle
 */
int MapBase::getInflateOccupancy(const Eigen::Vector3d &pos) const {
  Eigen::Vector3f pf  = pos.cast<float>() - pose_;  // point in the local frame
  int             idx = getVoxelIndex(pf);
  // std::cout << "pf: " << pf.transpose() << "\t idx: " << idx % MAP_LENGTH_VOXEL_NUM <<
  // "\trange"
  //           << this->isInRange(pf) << "\trisk:" << risk_maps_[idx][0] << std::endl;
  if (!this->isInRange(pf)) return -1;
  if (risk_maps_[idx][0] > risk_threshold_) return 1;
  return 0;
}

/**
 * @brief collision check on inflated map
 * @param pos in world frame
 * @param t : int
 * @return
 */
int MapBase::getInflateOccupancy(const Eigen::Vector3d &pos, int t) const {
  Eigen::Vector3f pf  = pos.cast<float>() - pose_;  // point in the local frame
  int             idx = getVoxelIndex(pf);
  // std::cout << "pf: " << pf.transpose() << "\t idx: " << idx % MAP_LENGTH_VOXEL_NUM <<
  // "\trange"
  // //           << this->isInRange(pf) << "\trisk:" << risk_maps_[idx][0] << std::endl;
  if (!this->isInRange(pf)) return -1;
  if (t > PREDICTION_TIMES) return -1;
  if (risk_maps_[idx][t] > risk_threshold_) return 1;
  return 0;
}
/**
 * @brief collision check on inflated map
 * @param pos in world frame
 * @param t : double FIXME: TIME MISMATCH
 * @return
 */
int MapBase::getInflateOccupancy(const Eigen::Vector3d &pos, double t) const {
  int tc = ceil(t / time_resolution_);
  tc     = tc > PREDICTION_TIMES ? PREDICTION_TIMES : tc;
  int tf = floor(t / time_resolution_);
  tf     = tf > PREDICTION_TIMES ? PREDICTION_TIMES : tf;

  Eigen::Vector3f pf  = pos.cast<float>() - pose_;  // point in the local frame
  int             idx = getVoxelIndex(pf);
  // std::cout << "pf: " << pf.transpose() << " " << pos.transpose() << "\t ti: " << t << " " << ti
  //           << "\trange" << this->isInRange(pf) << "\trisk:" << risk_maps_[idx][ti] << std::endl;
  if (!this->isInRange(pf)) return -1;
  if (risk_maps_[idx][tc] > risk_threshold_) return 1;
  if (risk_maps_[idx][tf] > risk_threshold_) return 1;
  return 0;
}

/**
 * @brief collision check on voxel map with clearance (none-inflated map)
 *
 * @param pos position in world frame
 * @param t index of the time frame
 * @return 0: free, 1: occupied, -1: out of bound
 */
int MapBase::getClearOcccupancy(const Eigen::Vector3d &pos, int t) const {
  Eigen::Vector3f pf = pos.cast<float>() - pose_;
  Eigen::Vector3i pi = getVoxelRelIndex(pf);
  if (!this->isInRange(pi)) return -1; /* query position out of range */
  for (auto &inf_pts : inflate_kernel_) {
    Eigen::Vector3i p = pi + inf_pts;
    if (!isInRange(p)) continue;
    int idx = getVoxelIndex(p);
    if (risk_maps_[idx][t] > risk_threshold_) return 1; /* collision */
  }
  return 0;
}

int MapBase::getClearOcccupancy(const Eigen::Vector3d &pos) const {
  return getClearOcccupancy(pos, 0);
}

int MapBase::getClearOcccupancy(const Eigen::Vector3d &pos, double dt) const {
  int tc = ceil(dt / time_resolution_);
  tc     = tc > PREDICTION_TIMES ? PREDICTION_TIMES : tc;
  int tf = floor(dt / time_resolution_);
  tf     = tf > PREDICTION_TIMES ? PREDICTION_TIMES : tf;

  int ret0 = getClearOcccupancy(pos, tc);
  int ret1 = getClearOcccupancy(pos, tf);
  if (ret0 == -1 || ret1 == -1) return -1;
  if (ret0 == 0 && ret1 == 0) {
    return 0;
  } else {
    return 1;
  }
}

/**
 * @brief get the obstacle points in the time interval and the cube
 * @param points : output
 * @param t_start : start time
 * @param t_end: end time
 */
void MapBase::getObstaclePoints(std::vector<Eigen::Vector3d> &points) {
  points.clear();
  for (int i = 0; i < VOXEL_NUM; i++) {
    if (risk_maps_[i][0] > risk_threshold_) {
      Eigen::Vector3f pt = getVoxelPosition(i);
      points.push_back(pt.cast<double>());
    }
  }
}

void MapBase::getObstaclePoints(std::vector<Eigen::Vector3d> &points,
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

void MapBase::getObstaclePoints(std::vector<Eigen::Vector3d> &points,
                                double                        t_start,
                                double                        t_end,
                                const Eigen::Vector3d        &lower_corner,
                                const Eigen::Vector3d        &higher_corner) {
  double t0 = last_update_time_.toSec();

  int idx_start = floor((t_start - t0) / time_resolution_);
  int idx_end   = ceil((t_end - t0) / time_resolution_);
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

  // std::cout << "z" << hz - lz << std::endl;
  // std::cout << "y" << hy - ly << std::endl;
  // std::cout << "x" << hx - lx << std::endl;
  for (int z = lz; z <= hz; z++) {
    for (int y = ly; y <= hy; y++) {
      for (int x = lx; x <= hx; x++) {
        int i = x + y * MAP_LENGTH_VOXEL_NUM + z * MAP_LENGTH_VOXEL_NUM * MAP_WIDTH_VOXEL_NUM;
        for (int j = idx_start; j <= idx_end; j++) {
          if (risk_maps_[i][j] > risk_threshold_) {
            points.emplace_back(getVoxelPosition(i).cast<double>());
          }
        }
      }
    }
  }
}

/**
 * @brief add obstacles to the risk map at the initial time
 * @param points: given obstacle points in the map frame
 * @param size: the size of the obstacle (axis-aligned bounding box)
 * @param t_index: the time index (indicating the number of predicted map)
 */
void MapBase::addObstacles(const std::vector<Eigen::Vector3d> &centers,
                           const Eigen::Vector3d              &size,
                           int                                 t_index) {
  for (auto &pt : centers) {
    float pt_x   = static_cast<float>(pt.x());
    float pt_y   = static_cast<float>(pt.y());
    float pt_z   = static_cast<float>(pt.z());
    float size_x = static_cast<float>(size.x());
    float size_y = static_cast<float>(size.y());
    float size_z = static_cast<float>(size.z());

    for (float z = pt_z - size_z; z <= pt_z + size_z; z += resolution_) {
      for (float y = pt_y - size_y; y <= pt_y + size_y; y += resolution_) {
        for (float x = pt_x - size_x; x <= pt_x + size_x; x += resolution_) {
          Eigen::Vector3f ptf = Eigen::Vector3f(x, y, z);
          if (isInRange(ptf)) {
            int index = getVoxelIndex(ptf);

            risk_maps_[index][t_index] = 1.0;
          }
        }
      }
    }
  }
}

/**
 * @brief add obstacles to the risk map at the given time
 * @param points: given obstacle points in the map frame
 * @param size: the size of the obstacle (axis-aligned bounding box)
 */
void MapBase::addObstacles(const std::vector<Eigen::Vector3d> &centers,
                           const Eigen::Vector3d              &size,
                           const ros::Time                    &t) {
  double dt = t.toSec() - last_update_time_.toSec();
  // double dt  = t.toSec() - ros::Time::now().toSec(); /* approximation: use current time */
  int idx = floor(dt / time_resolution_);
  if (idx >= PREDICTION_TIMES) {
    ROS_WARN("[MAP] The time %.2f is too large, the risk map will not be updated", dt);
    return;
  }
  if (idx < 0) {
    ROS_WARN("[MAP] The time %.2f is too small, the risk map will not be updated", dt);
    return;
  }
  addObstacles(centers, size, idx);
}
