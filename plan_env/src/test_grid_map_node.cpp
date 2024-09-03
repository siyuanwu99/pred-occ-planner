/**
 * @file test_grid_map_node.cpp
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
#include <plan_env/grid_map.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "quadrotor_msgs/PositionCommand.h"

#include <Eigen/Eigen>
#include <vector>

ros::Subscriber click_sub_;
ros::Publisher  cmd_pub_;

GridMap::Ptr grid_map_;

Eigen::Vector3d end_pos_, start_pos_, cur_pos_, dir_;
double          dist_;
double          vel_ = 1.0;  // m/s
double          dt_  = 0.1;  // s

void clickCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  end_pos_(0) = msg->pose.position.x;
  end_pos_(1) = msg->pose.position.y;
  end_pos_(2) = 1;

  dir_  = end_pos_ - cur_pos_;
  dir_  = dir_ / dir_.norm();
  dist_ = (end_pos_ - start_pos_).norm();
}

void timerCallback(const ros::TimerEvent &event) {
  if ((end_pos_ - cur_pos_).norm() < 0.1) {
    return;
  }
  quadrotor_msgs::PositionCommand cmd;
  cmd.header.stamp    = ros::Time::now();
  cmd.header.frame_id = "world";

  cur_pos_ = cur_pos_ + dir_ * vel_ * dt_;

  cmd.position.x = cur_pos_.x();
  cmd.position.y = cur_pos_.y();
  cmd.position.z = cur_pos_.z();

  cmd_pub_.publish(cmd);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "vis_mapping_node");
  ros::NodeHandle nh("~");
  int             pool_size_x = 100, pool_size_y = 100, pool_size_z = 60;

  nh.getParam("pool_size_x", pool_size_x);
  nh.getParam("pool_size_y", pool_size_y);
  nh.getParam("pool_size_z", pool_size_z);

  grid_map_.reset(new GridMap);
  grid_map_->initMap(nh);

  start_pos_ = Eigen::Vector3d(0, 0, 0);
  nh.getParam("init_x", start_pos_(0));
  nh.getParam("init_y", start_pos_(1));
  nh.getParam("init_z", start_pos_(2));
  nh.getParam("vel", vel_);
  nh.getParam("time_step", dt_);
  ROS_INFO("Start position: (%f, %f, %f)", start_pos_(0), start_pos_(1), start_pos_(2));
  cur_pos_ = start_pos_;
  click_sub_       = nh.subscribe("/move_base_simple/goal", 1, clickCallback);
  cmd_pub_         = nh.advertise<quadrotor_msgs::PositionCommand>("/pos_command", 1);
  ros::Timer timer = nh.createTimer(ros::Duration(dt_), timerCallback);
  ros::spin();

  return 0;
}