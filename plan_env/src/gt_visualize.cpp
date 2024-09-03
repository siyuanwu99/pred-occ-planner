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

#include <plan_env/gt_visualize.h>
#include <plan_env/map_parameters.h>

/**
 * @brief get color from jet colormap
 *
 * @param v current value
 * @param vmin min value
 * @param vmax max value
 * @return std_msgs::ColorRGBA
 */
std_msgs::ColorRGBA GTVisualize::getColorJet(double v, double vmin, double vmax) {
  std_msgs::ColorRGBA c;
  c.r = 1;
  c.g = 1;
  c.b = 1;
  c.a = 1;
  // white
  double dv;

  if (v < vmin) v = vmin;
  if (v > vmax) v = vmax;
  dv = vmax - vmin;

  if (v < (vmin + 0.25 * dv)) {
    c.r = 0;
    c.g = 4 * (v - vmin) / dv;
  } else if (v < (vmin + 0.5 * dv)) {
    c.r = 0;
    c.b = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
  } else if (v < (vmin + 0.75 * dv)) {
    c.r = 4 * (v - vmin - 0.5 * dv) / dv;
    c.b = 0;
  } else {
    c.g = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
    c.b = 0;
  }

  return (c);
}

std_msgs::ColorRGBA GTVisualize::getColorG2B(double v, double vmin, double vmax) {
  std_msgs::ColorRGBA c;
  c.r = 0;  // No red component for green-blue transition
  c.a = 1;  // Assuming you always want full opacity

  double dv;

  if (v < vmin) v = vmin;
  if (v > vmax) v = vmax;
  dv = vmax - vmin;

  // If you strictly want to transition from green to blue, the color calculation is simpler
  if (v < (vmin + 0.5 * dv)) {
    c.g = 1;
    c.b = 2 * (v - vmin) / dv;
  } else {
    c.g = 1 - 2 * (v - vmin - 0.5 * dv) / dv;
    c.b = 1;
  }

  return c;
}
void GTVisualize::init(ros::NodeHandle &nh1, ros::NodeHandle &nh2) {
  nh_ = nh1;

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

  future_risk_sub_ = nh1.subscribe("map/future_risk", 1, &GTVisualize::futureRiskCallback, this);
  time_sub_        = nh1.subscribe("map/time", 1, &GTVisualize::updateTimeCallback, this);

  if (is_pose_sub_) {
    pose_sub_ = nh2.subscribe("map/pose", 1, &GTVisualize::poseCallback, this);
  } else {
    odom_sub_ = nh2.subscribe("map/odom", 1, &GTVisualize::odomCallback, this);
  }

  ground_pub_  = nh1.advertise<sensor_msgs::PointCloud2>("map/ground", 1, true);
  cloud_pub_   = nh1.advertise<sensor_msgs::PointCloud2>("map/occupancy_inflated", 1, true);
  cloud_pub_1_ = nh1.advertise<sensor_msgs::PointCloud2>("map/occupancy_inflated_1", 1, true);
  cloud_pub_2_ = nh1.advertise<sensor_msgs::PointCloud2>("map/occupancy_inflated_2", 1, true);
  cloud_pub_3_ = nh1.advertise<sensor_msgs::PointCloud2>("map/occupancy_inflated_3", 1, true);
  cloud_pub_4_ = nh1.advertise<sensor_msgs::PointCloud2>("map/occupancy_inflated_4", 1, true);
  cloud_pub_5_ = nh1.advertise<sensor_msgs::PointCloud2>("map/occupancy_inflated_5", 1, true);

  /* initialize odometry */
  pose_ = Eigen::Vector3f::Zero();
  q_    = Eigen::Quaternionf::Identity();

  last_update_time_ = time_now_;
}

void GTVisualize::futureRiskCallback(const std_msgs::Float32MultiArrayConstPtr &future_risk) {
  last_update_time_ = time_now_;
  ros::Time t1      = time_now_;
  is_updating_map_  = true;
  int s             = future_risk->layout.dim[0].stride;
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

  ROS_INFO("[RiskMap] Updating map takes %f ms at %f", (time_now_ - t1).toSec() * 1000,
           last_update_time_.toSec());
  publishMapI(-1, ground_pub_);
  publishMapI(0, cloud_pub_);
  publishMapI(1, cloud_pub_1_);
  publishMapI(2, cloud_pub_2_);
  publishMapI(3, cloud_pub_3_);
  publishMapI(4, cloud_pub_4_);
  publishMapI(5, cloud_pub_5_);
}

void GTVisualize::publishMapI(int j, const ros::Publisher &pub) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud->points.reserve(VOXEL_NUM);
  for (int i = 0; i < VOXEL_NUM; i++) {
    pcl::PointXYZRGB pt;
    Eigen::Vector3f  pos = getVoxelPosition(i);
    if (j == -1) {
      if (pos.z() > -0.15 && pos.z() < 0.00) {
        pt.x = pos.x();
        pt.y = pos.y();
        pt.z = pos.z();
        pt.r = 255;
        pt.g = 155;
        pt.b = 0;
        cloud->points.push_back(pt);
      }
    } else {
      if (risk_maps_[i][j] > risk_threshold_) {
        pt.x = pos.x();
        pt.y = pos.y();
        pt.z = pos.z();
        if (pt.z < 0.00) {
          continue;
        }
        std_msgs::ColorRGBA c = getColorG2B(j * 0.3, 0, 2);
        pt.r                  = c.r * 255;
        pt.g                  = c.g * 255;
        pt.b                  = c.b * 255;
        cloud->points.push_back(pt);
      }
    }
  }
  cloud->width  = cloud->points.size();
  cloud->height = 1;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.stamp    = time_now_;
  cloud_msg.header.frame_id = "world";
  pub.publish(cloud_msg);
}

/**
 * @brief add occupancies of other agents to the map
 */
void GTVisualize::addOtherAgents() {
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
      std::vector<Eigen::Vector3d> pts;
      bool is_traj_valid = coordinator_->getParticlesWithRisk(pts, risks, i, t);
      particles.insert(particles.end(), pts.begin(), pts.end());
      is_swarm_traj_valid[i] = is_traj_valid;
    }

    /* filter */
    for (auto &pt : particles) {
      pt = pt - pose_.cast<double>();
    }
    addParticlesToRiskMap(particles, risks, t_idx);
  }
}

void GTVisualize::addObstaclesToRiskMap(const std::vector<Eigen::Vector3d> &pts, int t_index) {
  for (auto &pt : pts) {
    Eigen::Vector3f ptf = pt.cast<float>();
    if (!isInRange(ptf)) continue;
    int index                  = getVoxelIndex(ptf);
    risk_maps_[index][t_index] = 1.0;
  }
}
void GTVisualize::addParticlesToRiskMap(const std::vector<Eigen::Vector3d> &pts,
                                        const std::vector<float>           &risks,
                                        int                                 t_index) {
  for (int i = 0; i < pts.size(); i++) {
    Eigen::Vector3f ptf = pts[i].cast<float>();
    if (!isInRange(ptf)) continue;
    int index = getVoxelIndex(ptf);
    risk_maps_[index][t_index] += risks[i];
  }
}

void GTVisualize::addParticlesToRiskMap(const std::vector<Eigen::Vector3f> &pts,
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
// int gtvisualize::getclearocccupancy(const eigen::vector3d &pos, int t) const {
//   if (pos.z() < ground_height_) return 1;
//   if (pos.z() > ceiling_height_) return 1;
//
//   eigen::vector3f pos_f = pos.cast<float>() - pose_;
//   eigen::vector3i pos_i = getvoxelrelindex(pos_f);
//   if (!isinrange(pos_i)) return -1;
//   // if (risk_maps_[getvoxelindex(pos_i)][t] > risk_threshold_) return 1;
//   // return 0;
//
//   float sum_risk = 0.0;
//   // std::cout << "inflate kernel size: " << inflate_kernel_.size() << std::endl;
//   for (auto &ego_i : inflate_kernel_) {
//     eigen::vector3i p = pos_i + ego_i;
//     // std::cout << "pos_i: " << pos_i.transpose() << ", ego_i: " << ego_i.transpose()
//     //           << ", p: " << p.transpose() << std::endl;
//     if (!isinrange(p)) continue;
//     int idx = getvoxelindex(p);
//     sum_risk += risk_maps_[idx][t];
//     if (sum_risk > risk_threshold_astar_ - t * risk_thres_reg_decay_) return 1; /* collision */
//   }
//   return 0;
// }
//
// int gtvisualize::getclearocccupancy(const eigen::vector3d &pos) const {
//   return getclearocccupancy(pos, 0);
// }
//
// int gtvisualize::getclearocccupancy(const eigen::vector3d &pos, double dt) const {
//   int tf = floor(dt / time_resolution_);
//   tf     = tf > (prediction_times - 1) ? prediction_times - 1 : tf;
//   return getclearocccupancy(pos, tf);
// }
//
// /**
//  * @brief get the obstacle points in the time interval and the cube
//  * @param points : output
//  * @param t_start : start time
//  * @param t_end: end time
//  */
// void gtvisualize::getobstaclepoints(std::vector<eigen::vector3d> &points) {
//   points.clear();
//   for (int i = 0; i < voxel_num; i++) {
//     if (risk_maps_[i][0] > risk_threshold_) {
//       eigen::vector3f pt = getvoxelposition(i);
//       points.push_back(pt.cast<double>());
//     }
//   }
// }
//
// void gtvisualize::getobstaclepoints(std::vector<eigen::vector3d> &points,
//                                  double                        t_start,
//                                  double                        t_end) {
//   points.clear();
//   int idx_start = floor(t_start / time_resolution_);
//   int idx_end   = ceil(t_end / time_resolution_);
//   for (int i = 0; i < voxel_num; i++) {
//     for (int j = idx_start; j < idx_end; j++) {
//       if (risk_maps_[i][j] > risk_threshold_) {
//         eigen::vector3f pt = getvoxelposition(i);
//         points.push_back(pt.cast<double>());
//         break;
//       }
//     }
//   }
// }
//
// void gtvisualize::getobstaclepoints(std::vector<eigen::vector3d> &points,
//                                  double                        t_start,
//                                  double                        t_end,
//                                  const eigen::vector3d        &lower_corner,
//                                  const eigen::vector3d        &higher_corner) {
//   double t0 = last_update_time_.tosec();
//
//   int idx_start = floor((t_start - t0) / time_resolution_);
//   int idx_end   = ceil((t_end - t0) / time_resolution_);
//   idx_start     = idx_start < 0 ? 0 : idx_start;
//   idx_start     = idx_start > prediction_times ? prediction_times : idx_start;
//   idx_end       = idx_end > prediction_times ? prediction_times : idx_end;
//   idx_end       = idx_end < 0 ? 0 : idx_end;
//
//   // std::cout << "idx_start" << idx_start << "idx_end" << idx_end << std::endl;
//   int lx = (lower_corner[0] - pose_.x() + local_update_range_x_) / resolution_;
//   int ly = (lower_corner[1] - pose_.y() + local_update_range_y_) / resolution_;
//   int lz = (lower_corner[2] - pose_.z() + local_update_range_z_) / resolution_;
//   int hx = (higher_corner[0] - pose_.x() + local_update_range_x_) / resolution_;
//   int hy = (higher_corner[1] - pose_.y() + local_update_range_y_) / resolution_;
//   int hz = (higher_corner[2] - pose_.z() + local_update_range_z_) / resolution_;
//
//   hx = std::min(hx, map_length_voxel_num - 1);
//   hy = std::min(hy, map_width_voxel_num - 1);
//   hz = std::min(hz, map_height_voxel_num - 1);
//   lx = std::max(lx, 0);
//   ly = std::max(ly, 0);
//   lz = std::max(lz, 0);
//
//   for (int z = lz; z <= hz; z++) {
//     for (int y = ly; y <= hy; y++) {
//       for (int x = lx; x <= hx; x++) {
//         int i = x + y * map_length_voxel_num + z * map_length_voxel_num * map_width_voxel_num;
//         for (int j = idx_start; j <= idx_end; j++) {
//           if (risk_maps_[i][j] > risk_threshold_ - risk_thres_vox_decay_ * j) {
//             eigen::vector3d pt = getvoxelposition(i).cast<double>();
//             points.emplace_back(pt);
//           }
//         }
//       }
//     }
//   }
// }
