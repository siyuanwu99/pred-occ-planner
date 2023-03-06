/**
 * @file vis_risk_a_star_node.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-10-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
// #include <path_searching/risk_hybrid_a_star.h>
#include <path_searching/fake_risk_hybrid_a_star.h>
// #include <plan_env/risk_voxel.h>
#include <plan_env/fake_dsp_map.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Eigen>
#include <vector>

ros::Subscriber click_sub_;
ros::Publisher  path_pub_;
ros::Publisher  t_path_pub_;
ros::Publisher  voxel_pub_;
ros::Publisher  occupied_pub_;

FakeRiskVoxel::Ptr       grid_map_;
FakeRiskHybridAstar::Ptr a_star_;

Eigen::Vector3d              end_pos_, start_pos_, start_vel_;
Eigen::Vector3d              end_vel_   = Eigen::Vector3d::Zero();
Eigen::Vector3d              start_acc_ = Eigen::Vector3d::Zero();
Eigen::Vector3d              end_acc_   = Eigen::Vector3d::Zero();
std::vector<Eigen::Vector3d> path_;
double                       sample_duration_ = 0.2;

void visualizeObstacles(const std::vector<Eigen::Vector4d> &points, ros::Publisher &pub) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->points.reserve(points.size());
  for (auto &pt : points) {
    pcl::PointXYZ p;
    p.x = pt[0]; /* x */
    p.y = pt[1]; /* y */
    p.z = pt[3]; /* t */
    cloud->points.push_back(p);
  }
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.stamp    = ros::Time::now();
  cloud_msg.header.frame_id = "world";
  pub.publish(cloud_msg);
}

void visualizePath(const std::vector<Eigen::Vector3d> &path) {
  visualization_msgs::Marker marker;
  marker.header.frame_id    = "world";
  marker.header.stamp       = ros::Time::now();
  marker.ns                 = "path";
  marker.id                 = 0;
  marker.type               = visualization_msgs::Marker::LINE_STRIP;
  marker.action             = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x            = 0.02;
  marker.color.r            = 1.0;
  marker.color.g            = 0.0;
  marker.color.b            = 0.0;
  marker.color.a            = 1.0;

  Eigen::Vector3d last;
  bool            first = true;
  for (const auto &p : path) {
    geometry_msgs::Point pt;

    if (first) {
      first = false;
      last  = p;
    } else {
      pt.x = last.x();
      pt.y = last.y();
      pt.z = last.z();
      marker.points.push_back(pt);
      last = p;
      pt.x = last.x();
      pt.y = last.y();
      pt.z = last.z();
      marker.points.push_back(pt);
    }
  }
  path_pub_.publish(marker);
}

void visualizeTemporalPath(const std::vector<Eigen::Vector3d> &path, double dt) {
  visualization_msgs::Marker marker;
  marker.header.frame_id    = "world";
  marker.header.stamp       = ros::Time::now();
  marker.ns                 = "path";
  marker.id                 = 0;
  marker.type               = visualization_msgs::Marker::LINE_STRIP;
  marker.action             = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x            = 0.02;
  marker.color.r            = 0.0;
  marker.color.g            = 1.0;
  marker.color.b            = 0.0;
  marker.color.a            = 1.0;

  Eigen::Vector3d last;
  bool            first = true;
  double          t     = 0;
  for (const auto &p : path) {
    geometry_msgs::Point pt;

    if (first) {
      first = false;
      last  = p;
    } else {
      pt.x = last.x();
      pt.y = last.y();
      pt.z = t;
      marker.points.push_back(pt);
      std::cout << "x: " << pt.x << " y: " << pt.y << " t: " << pt.z << std::endl;
      t += dt;
      last = p;
      pt.x = last.x();
      pt.y = last.y();
      pt.z = t;
      marker.points.push_back(pt);
    }
  }
  t_path_pub_.publish(marker);
}

void clickCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  end_pos_(0) = msg->pose.position.x;
  end_pos_(1) = msg->pose.position.y;
  end_pos_(2) = 1;
  ROS_INFO("End position: (%f, %f, %f)", end_pos_(0), end_pos_(1), end_pos_(2));
  a_star_->reset();
  auto   t1      = ros::Time::now();
  auto   tm      = grid_map_->getMapTime();
  double delta_t = (t1 - tm).toSec();
  ROS_INFO("Time interval to last map updates: %f ms", delta_t * 1000);
  ASTAR_RET rst =
      a_star_->search(start_pos_, start_vel_, start_acc_, end_pos_, end_vel_, true, true, delta_t);
  auto t2 = ros::Time::now();
  ROS_INFO("Time used: %f ms", (t2 - t1).toSec() * 1000);

  if (rst == 0) {
    auto   t1      = ros::Time::now();
    auto   tm      = grid_map_->getMapTime();
    double delta_t = (t1 - tm).toSec();
    a_star_->reset();
    rst = a_star_->search(start_pos_, start_vel_, start_acc_, end_pos_, end_vel_, false, true,
                          delta_t);
  }

  std::vector<Eigen::Vector4d> visited_voxels  = a_star_->getTraversedObstacles();
  std::vector<Eigen::Vector4d> occupied_voxels = a_star_->getOccupiedObstacles();
  visualizeObstacles(visited_voxels, voxel_pub_);
  visualizeObstacles(occupied_voxels, occupied_pub_);

  if (rst > 2) {
    path_ = a_star_->getPath(sample_duration_);
    std::cout << "Path size: " << path_.size() << std::endl;
    int i = 0;
    for (const auto &p : path_) {
      std::cout << "t: " << (i++) * sample_duration_ << ": " << p.transpose() << std::endl;
    }
    visualizePath(path_);
    visualizeTemporalPath(path_, sample_duration_);
  } else {
    ROS_WARN("Failed to find path!");
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "vis_kinodyn_a_star_node");
  ros::NodeHandle nh("~");
  int             pool_size_x = 100, pool_size_y = 100, pool_size_z = 60;
  nh.getParam("pool_size_x", pool_size_x);
  nh.getParam("pool_size_y", pool_size_y);
  nh.getParam("pool_size_z", pool_size_z);

  start_pos_ = Eigen::Vector3d(0, 0, 0);
  nh.getParam("init_x", start_pos_(0));
  nh.getParam("init_y", start_pos_(1));
  nh.getParam("init_z", start_pos_(2));
  ROS_INFO("Start position: (%f, %f, %f)", start_pos_(0), start_pos_(1), start_pos_(2));

  start_vel_ = Eigen::Vector3d(0, 0, 0);
  nh.getParam("vel_x", start_vel_(0));
  nh.getParam("vel_y", start_vel_(1));
  nh.getParam("vel_z", start_vel_(2));
  ROS_INFO("Start velocity: (%f, %f, %f)", start_vel_(0), start_vel_(1), start_vel_(2));

  nh.getParam("sample_duration", sample_duration_);

  grid_map_.reset(new FakeRiskVoxel());
  grid_map_->init(nh);
  Eigen::Vector3f posf = start_pos_.cast<float>();
  grid_map_->setMapCenter(posf);
  ROS_INFO("Map initialized!");

  a_star_.reset(new FakeRiskHybridAstar());
  a_star_->setParam(nh);
  a_star_->setEnvironment(grid_map_);
  a_star_->init(start_pos_, Eigen::Vector3d(10, 10, 4));

  click_sub_    = nh.subscribe("/move_base_simple/goal", 1, clickCallback);
  path_pub_     = nh.advertise<visualization_msgs::Marker>("/path", 1);
  t_path_pub_   = nh.advertise<visualization_msgs::Marker>("/t_path", 1);
  voxel_pub_    = nh.advertise<sensor_msgs::PointCloud2>("/voxel", 1, true);
  occupied_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/occupied", 1, true);

  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
