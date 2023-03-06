/**
 * @file vis_a_star_node.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-09-28
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <path_searching/dyn_a_star.h>
#include <plan_env/grid_map.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Eigen>
#include <vector>

ros::Subscriber click_sub_;
ros::Publisher  path_pub_;

GridMap::Ptr grid_map_;
AStar::Ptr   a_star_;

Eigen::Vector3d              end_pos_, start_pos_;
std::vector<Eigen::Vector3d> path_;

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

void clickCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  end_pos_(0) = msg->pose.position.x;
  end_pos_(1) = msg->pose.position.y;
  end_pos_(2) = 1;
  ROS_INFO("End position: (%f, %f, %f)", end_pos_(0), end_pos_(1), end_pos_(2));
  a_star_->reset();
  auto t1 = ros::Time::now();
  a_star_->search(start_pos_, end_pos_);
  auto t2 = ros::Time::now();
  ROS_INFO("Time used: %f ms", (t2 - t1).toSec() * 1000);
  path_ = a_star_->getPath();
  visualizePath(path_);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "vis_a_star_node");
  ros::NodeHandle nh("~");
  int             pool_size_x = 100, pool_size_y = 100, pool_size_z = 60;
  nh.getParam("pool_size_x", pool_size_x);
  nh.getParam("pool_size_y", pool_size_y);
  nh.getParam("pool_size_z", pool_size_z);

  grid_map_.reset(new GridMap);
  grid_map_->initMap(nh);

  a_star_.reset(new AStar);
  a_star_->setParam(nh);
  a_star_->setEnvironment(grid_map_);
  a_star_->init(Eigen::Vector3d(0, 0, 1),
                Eigen::Vector3i(pool_size_x, pool_size_y, pool_size_z));

  start_pos_ = Eigen::Vector3d(0, 0, 0);
  nh.getParam("init_x", start_pos_(0));
  nh.getParam("init_y", start_pos_(1));
  nh.getParam("init_z", start_pos_(2));
  ROS_INFO("Start position: (%f, %f, %f)", start_pos_(0), start_pos_(1), start_pos_(2));

  click_sub_ = nh.subscribe("/move_base_simple/goal", 1, clickCallback);
  path_pub_  = nh.advertise<visualization_msgs::Marker>("/path", 1);
  ros::spin();

  return 0;
}