/**
 * @file risk_voxel.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-10-23
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <plan_env/map_parameters.h>
#include <plan_env/risk_voxel.h>

void RiskVoxel::init(ros::NodeHandle &nh) {
  nh_ = nh;

  /* Parameters */
  loadParameters();
  nh_.param("map/sigma_observation", observation_stddev_, 0.05F);
  nh_.param("map/sigma_localization", localization_stddev_, 0.05F);
  nh_.param("map/num_newborn_particles", num_newborn_particles_, 0.05F);
  nh_.param("map/risk_threshold_region", risk_threshold_astar_, 0.2F);

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

  dsp_map_.reset(new dsp_map::DSPMap());
  dsp_map_->setPredictionVariance(0.05, 0.05);
  // StdDev for prediction. velocity StdDev, position StdDev, respectively.
  dsp_map_->setObservationStdDev(observation_stddev_);  // StdDev for update. position StdDev.
  dsp_map_->setLocalizationStdDev(localization_stddev_);
  dsp_map_->setNewBornParticleNumberofEachPoint(20);
  // Number of new particles generated from one measurement point.
  dsp_map_->setNewBornParticleWeight(0.0001);  // Initial weight of particles.
  dsp_map::DSPMap::setOriginalVoxelFilterResolution(filter_res_);
  // Resolution of the voxel filter used for point cloud pre-process.
  // dsp_map_->setParticleRecordFlag(0, 19.0);
  // Set the first parameter to 1 to save particles at a time: e.g. 19.0s. Saving
  // will take a long time. Don't use it in realtime applications.
  ROS_INFO("[RiskMap] Init risk voxel map");

  cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_->points.reserve(80000);

  /* subscribers */
  cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "map/cloud", 30));
  if (!is_pose_sub_) {
    /* use odometry */
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(
        nh, "map/odom", 100, ros::TransportHints().tcpNoDelay()));
    sync_cloud_odom_.reset(new message_filters::Synchronizer<SyncPolicyCloudOdom>(
        SyncPolicyCloudOdom(100), *cloud_sub_, *odom_sub_));
    sync_cloud_odom_->registerCallback(boost::bind(&RiskVoxel::cloudOdomCallback, this, _1, _2));
    ROS_INFO("[RiskMap] subscribe to odom and depth points");
  } else {
    /* use pose */
    pose_sub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(
        nh, "map/pose", 100, ros::TransportHints().tcpNoDelay()));
    sync_cloud_pose_.reset(new message_filters::Synchronizer<SyncPolicyCloudPose>(
        SyncPolicyCloudPose(100), *cloud_sub_, *pose_sub_));
    sync_cloud_pose_->registerCallback(boost::bind(&RiskVoxel::cloudPoseCallback, this, _1, _2));
    ROS_INFO("[RiskMap] subscribe to pose and depth points");
  }

  /* publishers */
  cloud_pub_    = nh.advertise<sensor_msgs::PointCloud2>("map/occupancy_inflated", 1, true);
  obstacle_pub_ = nh.advertise<sensor_msgs::PointCloud2>("vis_obstacle", 1, true);

  /* publish point clouds in 10 Hz */
  pub_timer_ = nh.createTimer(ros::Duration(0.10), &RiskVoxel::pubCallback, this);

  /* initialize odometry */
  pose_ = Eigen::Vector3f::Zero();
  q_    = Eigen::Quaternionf::Identity();

  last_update_time_ = ros::Time::now();
}

/**
 * @brief subscribe point cloud and odometry
 *
 * @param cloud_msg point cloud in camera frame
 * @param pose_msg geometry_msgs::PoseStamped
 */
void RiskVoxel::cloudPoseCallback(const sensor_msgs::PointCloud2::ConstPtr   &cloud_msg,
                                  const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {
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
  // std::cout << "odom: " << pose_.transpose() << "  pose: " << q_.w() << q_.x() << q_.y() <<
  // q_.z()
  //           << std::endl;
  updateMap(cloud_msg);
}

/**
 * @brief subscribe point cloud and odometry
 *
 * @param cloud_msg point cloud in camera frame
 * @param odom_msg nav_msgs::Odometry
 */
void RiskVoxel::cloudOdomCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg,
                                  const nav_msgs::Odometry::ConstPtr       &odom_msg) {
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
  // std::cout << "odom: " << pose_.transpose() << "  pose: " << q_.w() << q_.x() << q_.y() <<
  // q_.z()
  //           << std::endl;
  updateMap(cloud_msg);
}

/**
 * @brief publish point clouds in a fixed frequency
 *
 * @param event
 */
void RiskVoxel::pubCallback(const ros::TimerEvent &event) { publishMap(); }

/**
 * @brief publish occupancy map
 *
 */
void RiskVoxel::publishMap() {
  // clock_t t1           = clock();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->points.reserve(VOXEL_NUM);
  int   num_occupied = 0;
  float temp_risk_map[VOXEL_NUM][PREDICTION_TIMES];
  dsp_map_->getOccupancyMapWithFutureStatus(num_occupied, *cloud, &temp_risk_map[0][0],
                                            risk_threshold_);
  for (auto &ego_i : inflate_kernel_) {
    int idx               = getVoxelIndex(ego_i);
    temp_risk_map[idx][0] = 0.0;
    temp_risk_map[idx][1] = 0.0;
    temp_risk_map[idx][2] = 0.0;
  }

  for (int i = 0; i < VOXEL_NUM; i++) {
    for (int j = 0; j < PREDICTION_TIMES; j++) {
      risk_maps_[i][j] = temp_risk_map[i][j];
    }
  }
  /* set current to zero to prevent deadlock */

  clock_t t0 = clock();
  /* project other agents to the map */
  if (is_multi_agents_) {
    addOtherAgents();
  }
  clock_t t1 = clock();
  std::cout << "[RiskMap] agent particle update time (ms): " << (t1 - t0) * 1000 / CLOCKS_PER_SEC
            << std::endl;

  std::string st_msg  = (if_pub_spatio_temporal_map_) ? "true" : "false";
  std::string wrd_msg = (if_pub_in_world_frame_) ? "true" : "false";

  if (if_pub_spatio_temporal_map_) {
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < VOXEL_NUM; i++) {
      pcl::PointXYZ   pt;
      Eigen::Vector3f pos = getVoxelPosition(i);
      if (abs(pos.z() - pose_.z()) > 0.5) continue;  // filter out necessary points
      for (int j = 0; j < PREDICTION_TIMES; j++) {
        if (risk_maps_[i][j] > risk_threshold_) {
          pt.x = pos.x();
          pt.y = pos.y();
          pt.z = j * time_resolution_;
          cloud->points.push_back(pt);
        }
      }
    }
    cloud->width  = cloud->points.size();
    cloud->height = 1;
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
 * @brief update DSP map
 *
 * @param cloud_in pointer to input cloud
 */
void RiskVoxel::updateMap(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg) {
  last_update_time_ = ros::Time::now();
  double t          = cloud_msg->header.stamp.toSec();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*cloud_msg, *cloud_in);

  int                                 n_valid = 0;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
  filterPointCloud(cloud_in, cloud_filtered, valid_clouds_, n_valid);

  clock_t t_update_0 = clock();

  /* update DSP map */
  // std::cout << "num_valid " << n_valid << std::endl;
  if (!dsp_map_->update(n_valid, 3, valid_clouds_, pose_.x(), pose_.y(), pose_.z(), t, q_.w(),
                        q_.x(), q_.y(), q_.z())) {
    return;
  }

  clock_t t_update_1 = clock();
  std::cout << "[RiskMap] map update time (ms): "
            << (t_update_1 - t_update_0) * 1000 / CLOCKS_PER_SEC << std::endl;
}

/**
 * @brief add occupancies of other agents to the map
 */
void RiskVoxel::addOtherAgents() {
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
// void RiskVoxel::createEgoParticlesVoxel() {
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
void RiskVoxel::addObstaclesToRiskMap(const std::vector<Eigen::Vector3d> &pts, int t_index) {
  for (auto &pt : pts) {
    Eigen::Vector3f ptf = pt.cast<float>();
    if (!isInRange(ptf)) continue;
    int index                  = getVoxelIndex(ptf);
    risk_maps_[index][t_index] = 1.0;
  }
}
void RiskVoxel::addParticlesToRiskMap(const std::vector<Eigen::Vector3d> &pts,
                                      const std::vector<float>           &risks,
                                      int                                 t_index) {
  for (int i = 0; i < pts.size(); i++) {
    Eigen::Vector3f ptf = pts[i].cast<float>();
    if (!isInRange(ptf)) continue;
    int index = getVoxelIndex(ptf);
    risk_maps_[index][t_index] += risks[i];
  }
}

void RiskVoxel::addParticlesToRiskMap(const std::vector<Eigen::Vector3f> &pts,
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
 * @brief check if the point is in obstacle
 * @param pos: point position in world frame
 * @return -1: out of range, 0: not in obstacle, 1: in obstacle
 */
int RiskVoxel::getInflateOccupancy(const Eigen::Vector3d &pos) const {
  int   index;
  float px = static_cast<float>(pos(0));
  float py = static_cast<float>(pos(1));
  float pz = static_cast<float>(pos(2));

  if (!dsp_map_->getPointVoxelsIndexPublic(px, py, pz, index)) {
    return -1;
  }
  float risk = risk_maps_[index][0];
  if (risk > risk_threshold_) {
    return 1;
  }
  return 0;
}

/**
 * @brief
 * @param pos
 * @param t : int
 * @return
 */
int RiskVoxel::getInflateOccupancy(const Eigen::Vector3d &pos, int t) const {
  int   index;
  float px = static_cast<float>(pos(0));
  float py = static_cast<float>(pos(1));
  float pz = static_cast<float>(pos(2));

  // std::cout << "pf: " << pf.transpose() << "\t idx: " << idx % MAP_LENGTH_VOXEL_NUM << "\trange"
  // //           << this->isInRange(pf) << "\trisk:" << risk_maps_[idx][0] << std::endl;
  if (!dsp_map_->getPointVoxelsIndexPublic(px, py, pz, index)) return -1;
  if (t > PREDICTION_TIMES) return -1;
  if (risk_maps_[index][t] > risk_threshold_) return 1;
  return 0;
}

/**
 * @brief
 * @param pos
 * @param t : double
 * @return
 */
int RiskVoxel::getInflateOccupancy(const Eigen::Vector3d &pos, double t) const {
  int   ti = t / time_resolution_;
  int   index;
  float px = static_cast<float>(pos(0));
  float py = static_cast<float>(pos(1));
  float pz = static_cast<float>(pos(2));
  if (!dsp_map_->getPointVoxelsIndexPublic(px, py, pz, index)) return -1;
  if (t > PREDICTION_TIMES) return -1;
  if (risk_maps_[index][ti] > risk_threshold_) return 1;
  return 0;
}

/**
 * @brief collision check on voxel map with clearance (none-inflated map)
 *
 * @param pos position in world frame
 * @param t index of the time frame
 * @return 0: free, 1: occupied, -1: out of bound
 */
int RiskVoxel::getClearOcccupancy(const Eigen::Vector3d &pos, int t) const {
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
    if (sum_risk > risk_threshold_astar_) return 1; /* collision */
  }
  return 0;
}

int RiskVoxel::getClearOcccupancy(const Eigen::Vector3d &pos) const {
  return getClearOcccupancy(pos, 0);
}

int RiskVoxel::getClearOcccupancy(const Eigen::Vector3d &pos, double dt) const {
  int tf = floor(dt / time_resolution_);
  tf     = tf > (PREDICTION_TIMES - 1) ? PREDICTION_TIMES - 1 : tf;
  return getClearOcccupancy(pos, tf);
}
