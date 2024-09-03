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

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <plan_env/dsp_dynamic.h>
#include <plan_env/map_parameters.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <queue>
#include "gazebo_msgs/ModelStates.h"
#include "std_msgs/Float32MultiArray.h"

#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>

constexpr double LOCAL_RANGE_X = MAP_LENGTH_VOXEL_NUM / 2.f * VOXEL_RESOLUTION;
constexpr double LOCAL_RANGE_Y = MAP_WIDTH_VOXEL_NUM / 2.f * VOXEL_RESOLUTION;
constexpr double LOCAL_RANGE_Z = MAP_HEIGHT_VOXEL_NUM / 2.f * VOXEL_RESOLUTION;

dsp_map::DSPMap dsp_map_;

/** ROS */
bool            is_pose_sub_ = false; /** if subscribe pose or odom */
ros::Publisher  cloud_pub_, future_risk_pub_, risk_2dmap_pub_, time_pub_;
ros::Subscriber cloud_sub_, pose_sub_, odom_sub_;
ros::Time       last_update_time_;

/** map */
const unsigned int MAX_POINT_NUM = 5000;
float              valid_clouds_[MAX_POINT_NUM * 3];
float              risk_maps_[VOXEL_NUM][PREDICTION_TIMES];

Eigen::Vector3f    pose_ = Eigen::Vector3f::Zero();
Eigen::Vector3f    vel_  = Eigen::Vector3f::Zero();
Eigen::Quaternionf q_    = Eigen::Quaternionf::Identity();

/** parameters */
float filter_res_;
float risk_threshold_;
float observation_stddev_;
float localization_stddev_;

void getParam(const ros::NodeHandle &nh) {
  nh.param("map/booleans/sub_pose", is_pose_sub_, false);
  nh.param("map/resolution", filter_res_, 0.15F); /* resolution of the voxel filter */
  nh.param("map/risk_threshold_voxel", risk_threshold_, 0.2F);
  nh.param("map/sigma_observation", observation_stddev_, 0.05F);
  nh.param("map/sigma_localization", localization_stddev_, 0.05F);
}

bool isInRange(const Eigen::Vector3f &p) {
  return p.x() > -LOCAL_RANGE_X && p.x() < LOCAL_RANGE_X && p.y() > -LOCAL_RANGE_Y &&
         p.y() < LOCAL_RANGE_Y && p.z() > -LOCAL_RANGE_Z && p.z() < LOCAL_RANGE_Z;
}

void filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
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
 * @brief publish point cloud
 *
 */
void publishMap() {
  int                            num_occupied = 0;
  clock_t                        t1           = clock();
  pcl::PointCloud<pcl::PointXYZ> cloud;
  dsp_map_.getOccupancyMapWithFutureStatus(num_occupied, cloud, &risk_maps_[0][0], risk_threshold_);

  for (auto &p : cloud.points) {
    p.x += pose_.x();
    p.y += pose_.y();
    p.z += pose_.z();
  }
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp    = ros::Time::now();
  cloud_msg.header.frame_id = "world";
  cloud_pub_.publish(cloud_msg);
  std::cout << "num_occupied: " << num_occupied << std::endl;

  /** publish future risks */

  // /* HACK: the risk map sometimes gives zero value */
  // float sum_risk =
  //     std::accumulate(&risk_maps_[0][0], &risk_maps_[0][0] + VOXEL_NUM * PREDICTION_TIMES, 0.f);
  // if (sum_risk > 0.0f) {
  std_msgs::Float32MultiArray   future_risk_array_msg;
  std_msgs::MultiArrayDimension future_risk_array_dimension;
  future_risk_array_dimension.size   = VOXEL_NUM;
  future_risk_array_dimension.stride = PREDICTION_TIMES;
  future_risk_array_msg.layout.dim.push_back(future_risk_array_dimension);
  future_risk_array_msg.data.reserve(VOXEL_NUM * PREDICTION_TIMES + 4);
  for (int i = 0; i < VOXEL_NUM; ++i) {
    for (int j = 0; j < PREDICTION_TIMES; ++j) {
      future_risk_array_msg.data.push_back(risk_maps_[i][j]);
    }
  }
  future_risk_array_msg.data.push_back(pose_.x());
  future_risk_array_msg.data.push_back(pose_.y());
  future_risk_array_msg.data.push_back(pose_.z());
  // add current time
  future_risk_array_msg.data.push_back(ros::Time::now().toSec());
  future_risk_pub_.publish(future_risk_array_msg);

  clock_t t2 = clock();
  std::cout << "publish time (ms): " << (t2 - t1) * 1000 / (double)CLOCKS_PER_SEC << std::endl;
  // }
}
/**
 * @brief update DSP map
 *
 * @param cloud_in pointer to input cloud
 */
void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg) {
  double t          = cloud_msg->header.stamp.toSec();
  last_update_time_ = cloud_msg->header.stamp;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*cloud_msg, *cloud_in);

  int                                 n_valid = 0;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
  filterPointCloud(cloud_in, cloud_filtered, valid_clouds_, n_valid);

  clock_t t_update_0 = clock();

  /* update DSP map */
  // std::cout << "num_valid " << n_valid << std::endl;
  if (!dsp_map_.update(n_valid, 3, valid_clouds_, pose_.x(), pose_.y(), pose_.z(), t, q_.w(),
                       q_.x(), q_.y(), q_.z())) {
    return;
  }

  clock_t t_update_1 = clock();
  std::cout << "[RiskMap] map update time (ms): "
            << (t_update_1 - t_update_0) * 1000 / CLOCKS_PER_SEC << std::endl;

  /* publish header */
  std_msgs::Header time_msg;
  time_msg.stamp    = cloud_msg->header.stamp;
  time_msg.frame_id = "world";
  time_pub_.publish(time_msg);

  /* publish map */
  publishMap();
}

void clearMapCallback(const ros::TimerEvent &event) {
  if (vel_.norm() > 0.1) {
    return;
  }
  float dx = 0.15;
  for (float x = -0.9f; x < 0.9f; x += dx) {
    for (float y = -0.9f; y < 0.9f; y += dx) {
      for (float z = -0.9f; z < 0.9f; z += dx) {
        dsp_map_.removeParticlesAtMapPosition(x, y, z);
      }
    }
  }
}

void poseCallback(const geometry_msgs::PoseStamped &msg) {
  pose_ << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
  q_.x() = msg.pose.orientation.x;
  q_.y() = msg.pose.orientation.y;
  q_.z() = msg.pose.orientation.z;
  q_.w() = msg.pose.orientation.w;
}

void odomCallback(const nav_msgs::Odometry &msg) {
  pose_ << msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z;
  q_.x() = msg.pose.pose.orientation.x;
  q_.y() = msg.pose.pose.orientation.y;
  q_.z() = msg.pose.pose.orientation.z;
  q_.w() = msg.pose.pose.orientation.w;
  vel_ << msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "risk_voxel_node");
  ros::NodeHandle nh("~");
  getParam(nh);

  cloud_sub_ = nh.subscribe("map/cloud", 1, cloudCallback);
  if (!is_pose_sub_) { /* use odometry */
    odom_sub_ = nh.subscribe("map/odom", 100, odomCallback);

  } else { /* use pose */
    pose_sub_ = nh.subscribe("map/pose", 100, poseCallback);
  }
  future_risk_pub_ = nh.advertise<std_msgs::Float32MultiArray>("map/future_risk", 1, true);
  // risk_2dmap_pub             = n.advertise<nav_msgs::OccupancyGrid>("risk2d", 1, true);
  cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("map/occupancy_inflated", 1, true);
  time_pub_  = nh.advertise<std_msgs::Header>("map/time", 1, true);

  dsp_map_.setPredictionVariance(0.01, 0.01);
  // StdDev for prediction. velocity StdDev, position StdDev, respectively.
  dsp_map_.setObservationStdDev(observation_stddev_);  // StdDev for update. position StdDev.
  dsp_map_.setLocalizationStdDev(localization_stddev_);
  dsp_map_.setNewBornParticleNumberofEachPoint(20);
  // Number of new particles generated from one measurement point.
  dsp_map_.setNewBornParticleWeight(0.0001);  // Initial weight of particles.
  dsp_map::DSPMap::setOriginalVoxelFilterResolution(filter_res_);
  // Resolution of the voxel filter used for point cloud pre-process.
  // dsp_map_->setParticleRecordFlag(0, 19.0);
  // Set the first parameter to 1 to save particles at a time: e.g. 19.0s. Saving
  // will take a long time. Don't use it in realtime applications.
  ROS_INFO("[RiskMap] Init risk voxel map");

  ros::Timer clear_map_timer = nh.createTimer(ros::Duration(3.0), clearMapCallback);

  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();
  return 0;

  return 0;
}
