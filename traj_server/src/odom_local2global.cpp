/**
 * @file odom_local2global.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief generate localization noise
 * @version 1.0
 * @date 2022-02-15
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Eigen>
#include <iostream>
#include <random>
#include <string>

Eigen::Vector3d    position_;
Eigen::Quaterniond orientation_;

ros::Publisher global_odom_pub_, global_pose_pub_;
void           odomCallback(const nav_msgs::Odometry& msg) {
  nav_msgs::Odometry global_odom;
  global_odom.header.frame_id = "world";
  // std::string ns                      = ros::this_node::getNamespace();
  // global_odom.child_frame_id          = ns + "/base_link";
  // global_odom.child_frame_id          = "base_link";
  global_odom.header.seq              = msg.header.seq;
  global_odom.header.stamp            = msg.header.stamp;
  global_odom.pose.pose.position.x    = msg.pose.pose.position.x + position_.x();
  global_odom.pose.pose.position.y    = msg.pose.pose.position.y + position_.y();
  global_odom.pose.pose.position.z    = msg.pose.pose.position.z + position_.z();
  global_odom.pose.pose.orientation.w = msg.pose.pose.orientation.w;
  global_odom.pose.pose.orientation.x = msg.pose.pose.orientation.x;
  global_odom.pose.pose.orientation.y = msg.pose.pose.orientation.y;
  global_odom.pose.pose.orientation.z = msg.pose.pose.orientation.z;
  global_odom.twist.twist.linear.x    = msg.twist.twist.linear.x;
  global_odom.twist.twist.linear.y    = msg.twist.twist.linear.y;
  global_odom.twist.twist.linear.z    = msg.twist.twist.linear.z;
  global_odom.twist.twist.angular.x   = msg.twist.twist.angular.x;
  global_odom.twist.twist.angular.y   = msg.twist.twist.angular.y;
  global_odom.twist.twist.angular.z   = msg.twist.twist.angular.z;

  global_odom_pub_.publish(global_odom);
}

void poseCallback(const geometry_msgs::PoseStamped& msg) {
  geometry_msgs::PoseStamped global_pose;
  global_pose.header.seq         = msg.header.seq;
  global_pose.header.stamp       = msg.header.stamp;
  global_pose.header.frame_id    = "world";
  global_pose.pose.position.x    = msg.pose.position.x + position_.x();
  global_pose.pose.position.y    = msg.pose.position.y + position_.y();
  global_pose.pose.position.z    = msg.pose.position.z + position_.z();
  global_pose.pose.orientation.x = msg.pose.orientation.x;
  global_pose.pose.orientation.y = msg.pose.orientation.y;
  global_pose.pose.orientation.z = msg.pose.orientation.z;
  global_pose.pose.orientation.w = msg.pose.orientation.w;

  global_pose_pub_.publish(global_pose);
}

void staticTFBroadcast(const Eigen::Vector3d&    position,
                       const Eigen::Quaterniond& orientation,
                       std::string               ns) {
  /* transform broadcaster at robot's initial position */
  static tf2_ros::StaticTransformBroadcaster static_broadcaster_;
  geometry_msgs::TransformStamped            static_transformStamped;
  static_transformStamped.header.stamp            = ros::Time::now();
  static_transformStamped.header.frame_id         = "world";
  static_transformStamped.child_frame_id          = ns + "/local";
  static_transformStamped.transform.translation.x = position.x();
  static_transformStamped.transform.translation.y = position.y();
  static_transformStamped.transform.translation.z = position.z();
  static_transformStamped.transform.rotation.w    = 1.0;
  static_transformStamped.transform.rotation.x    = 0.0;
  static_transformStamped.transform.rotation.y    = 0.0;
  static_transformStamped.transform.rotation.z    = 0.0;
  static_broadcaster_.sendTransform(static_transformStamped);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "odom_local2global");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  std::string ns = ros::this_node::getNamespace();
  ROS_INFO("[GlbOdom] Node namespace: %s", ns.c_str());

  position_    = Eigen::Vector3d::Zero();
  orientation_ = Eigen::Quaterniond::Identity();
  /* load std dev from ROS parameter server */
  nh_priv.param("init_x", position_.x(), 0.0);
  nh_priv.param("init_y", position_.y(), 0.0);
  nh_priv.param("init_z", position_.z(), 0.0);
  nh_priv.param("init_qw", orientation_.w(), 0.0);
  nh_priv.param("init_qx", orientation_.x(), 0.0);
  nh_priv.param("init_qy", orientation_.y(), 0.0);
  nh_priv.param("init_qz", orientation_.z(), 0.0);

  ROS_INFO("[GlbOdom] Init pose: (%f, %f, %f) (%f, %f, %f, %f)", position_.x(), position_.y(),
           position_.z(), orientation_.w(), orientation_.x(), orientation_.y(), orientation_.z());

  ros::Subscriber odom_sub =
      nh_priv.subscribe("local_odom", 1, &odomCallback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber pose_sub =
      nh_priv.subscribe("local_pose", 1, &poseCallback, ros::TransportHints().tcpNoDelay());

  global_odom_pub_ = nh_priv.advertise<nav_msgs::Odometry>("global_odom", 1);
  global_pose_pub_ = nh_priv.advertise<geometry_msgs::PoseStamped>("global_pose", 1);

  staticTFBroadcast(position_, orientation_, ns);

  ros::spin();
}
