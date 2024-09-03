/**
 * @file test_firi_node.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-12-10
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Eigen>
#include <chrono>
#include <sfc_gen/firi.hpp>
#include <traj_utils/visualizer.hpp>
#include <vector>

using namespace chrono;

visualizer::Visualizer::Ptr visualizer_;
ros::Subscriber             click_sub_;
ros::Subscriber             point_sub_;
ros::Subscriber             path_sub_;
ros::Subscriber             odom_sub_;
ros::Publisher              obstacle_pub_;

double          range_ = 1.0;
int             flag_  = 0;
Eigen::Vector3d pos_;

Eigen::Vector3d lower_corner_  = Eigen::Vector3d(-4, -4, -1);
Eigen::Vector3d higher_corner_ = Eigen::Vector3d(4, 4, 1);

std::vector<Eigen::MatrixX4d> hPolys_;
std::vector<Eigen::Vector3d>  pc;
std::vector<Eigen::Vector3d>  astar_path_;

Eigen::Vector3d pos_start(0, 0, 0);

void generateCorridor(const Eigen::Vector3d              &pos_start,
                      const Eigen::Vector3d              &pos_goal,
                      const std::vector<Eigen::Vector3d> &pc) {
  Eigen::Vector3d llc, lhc; /* local lower corner and higher corner */
  lhc(0) = std::min(std::max(pos_start(0), pos_goal(0)) + range_, pos_(0) + higher_corner_(0));
  lhc(1) = std::min(std::max(pos_start(1), pos_goal(1)) + range_, pos_(1) + higher_corner_(1));
  lhc(2) = std::min(std::max(pos_start(2), pos_goal(2)) + range_, pos_(2) + higher_corner_(2));
  llc(0) = std::max(std::min(pos_start(0), pos_goal(0)) - range_, pos_(0) + lower_corner_(0));
  llc(1) = std::max(std::min(pos_start(1), pos_goal(1)) - range_, pos_(1) + lower_corner_(1));
  llc(2) = std::max(std::min(pos_start(2), pos_goal(2)) - range_, pos_(2) + lower_corner_(2));
  std::cout << "pos: " << pos_.transpose() << std::endl;
  std::cout << "lhc: " << lhc.transpose() << std::endl;
  std::cout << "llc: " << llc.transpose() << std::endl;

  Eigen::Matrix<double, 6, 4> bd = Eigen::Matrix<double, 6, 4>::Zero();  // initial corridor
  bd(0, 0)                       = 1.0;
  bd(1, 1)                       = 1.0;
  bd(2, 2)                       = 1.0;
  bd(3, 0)                       = -1.0;
  bd(4, 1)                       = -1.0;
  bd(5, 2)                       = -1.0;
  bd.block<3, 1>(0, 3)           = -lhc;
  bd.block<3, 1>(3, 3)           = llc;
  Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> m_pc(pc[0].data(), 3, pc.size());

  Eigen::MatrixX4d hPoly;
  auto             t1 = system_clock::now();
  firi::firi(bd, m_pc, pos_start, pos_goal, hPoly, 1, 1e-6);
  auto t2 = system_clock::now();
  std::cout << "===== FIRI time =====" << std::endl;
  auto duration = duration_cast<microseconds>(t2 - t1);
  std::cout << double(duration.count()) * microseconds::period::num / microseconds::period::den *
                   1000
            << "ms" << std::endl;
  // std::cout << "===== hPoly  =====" << std::endl;
  // std::cout << hPoly << std::endl;
  hPolys_.push_back(hPoly);
}

/**
 * @brief
 * publish obstacle points
 * @param points
 */
void showObstaclePoints(const std::vector<Eigen::Vector3d> &points) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int i = 0; i < points.size(); i++) {
    pcl::PointXYZ p;
    p.x = points[i](0);
    p.y = points[i](1);
    p.z = points[i](2);
    cloud.push_back(p);
  }
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp    = ros::Time::now();
  cloud_msg.header.frame_id = "world";
  obstacle_pub_.publish(cloud_msg);
}

/**
 * @brief receive trajectory points, push back to astar_path_
 * @param msg
 */
void pointCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);
  ROS_INFO("Received point cloud size: %d", int(cloud.size()));

  pc.clear();
  for (int i = 0; i < cloud.size(); i++) {
    pc.push_back(Eigen::Vector3d(cloud[i].x, cloud[i].y, cloud[i].z));
  }
  if (astar_path_.size() > 0) {
    std::cout << "===== astar_path_ =====" << std::endl;
    std::cout << flag_ + 0 << "|:" << astar_path_[flag_ + 0].transpose() << std::endl;
    std::cout << flag_ + 3 << "|:" << astar_path_[flag_ + 3].transpose() << std::endl;
    generateCorridor(astar_path_[flag_ + 0], astar_path_[flag_ + 3], pc);
    visualizer_->visualizePolytope(hPolys_);
    visualizer_->visualizeStartGoal(astar_path_[flag_ + 3]);
  }
  flag_ += 3;
  if (flag_ >= int(astar_path_.size())) {
    flag_ = 0;
  }
}

void pathCallback(const visualization_msgs::Marker::ConstPtr &msg) {
  flag_ = 0;
  hPolys_.clear();
  astar_path_.clear();
  for (int i = 0; i < int(msg->points.size()); i++) {
    astar_path_.push_back(Eigen::Vector3d(msg->points[i].x, msg->points[i].y, msg->points[i].z));
  }
  ROS_INFO("Received path size: %i", (int)(astar_path_.size()));
}

void odomCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  pos_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_firi_node");
  ros::NodeHandle nh("~");
  std::string     ns = "world";
  visualizer_.reset(new visualizer::Visualizer(nh, ns));

  point_sub_ = nh.subscribe("/points", 1, &pointCallback);
  path_sub_  = nh.subscribe("/path", 1, &pathCallback);
  odom_sub_  = nh.subscribe("/odom", 1, &odomCallback);

  // obstacle_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/obstacle_points", 1);
  ros::spin();
}
