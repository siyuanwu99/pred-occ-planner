/**
 * @file risk_voxel_moving.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief Debug dsp map when drone is moving
 * @version 1.0
 * @date 2022-10-23
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <plan_env/risk_voxel.h>
#include <quadrotor_msgs/PositionCommand.h>
// #include <plan_env/fake_dsp_map.h>
#include <ros/ros.h>

ros::Publisher pose_pub_;

int             pose_count_ = 0;
double          dt_         = 0.01;
Eigen::Vector3d pos         = Eigen::Vector3d(0, 0, 0);
Eigen::Vector3d vel         = Eigen::Vector3d(0, 0, 0);
Eigen::Vector3d acc         = Eigen::Vector3d(0, 0, 0);
double          yaw         = 0;
double          yaw_dot     = 0;

void poseCallback(const ros::TimerEvent& e) {
  pose_count_++;
  /* ----- moving logic ----- */

  /* make our drone move along a line back and force */
  if (pose_count_ < 500) {
    vel     = Eigen::Vector3d(1, 0, 0);
    acc     = Eigen::Vector3d(0, 0, 0);
    yaw_dot = 0;
  } else if (pose_count_ < 600) {
    vel     = Eigen::Vector3d(0, 0, 0);
    acc     = Eigen::Vector3d(0, 0, 0);
    yaw_dot = 3.1415926;
  } else if (pose_count_ < 1600) {
    vel     = Eigen::Vector3d(-1, 0, 0);
    acc     = Eigen::Vector3d(0, 0, 0);
    yaw_dot = 0;
  } else if (pose_count_ < 1700) {
    vel     = Eigen::Vector3d(0, 0, 0);
    acc     = Eigen::Vector3d(0, 0, 0);
    yaw_dot = 3.1415926;
  } else if (pose_count_ < 2200) {
    vel     = Eigen::Vector3d(1, 0, 0);
    acc     = Eigen::Vector3d(0, 0, 0);
    yaw_dot = 0;
  } else {
    pose_count_ = 0;
    vel         = Eigen::Vector3d(0, 0, 0);
    acc         = Eigen::Vector3d(0, 0, 0);
    yaw_dot     = 0;
  }

  pos += vel * dt_;
  vel += acc * dt_;
  yaw += yaw_dot * dt_;

  /* ----- publish control commands ----- */
  quadrotor_msgs::PositionCommand cmd;
  cmd.header.stamp    = ros::Time::now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.position.x      = pos(0);
  cmd.position.y      = pos(1);
  cmd.position.z      = pos(2);
  cmd.yaw             = yaw;
  cmd.velocity.x      = vel(0);
  cmd.velocity.y      = vel(1);
  cmd.velocity.z      = vel(2);
  cmd.yaw_dot         = yaw_dot;
  cmd.acceleration.x  = acc(0);
  cmd.acceleration.y  = acc(1);
  cmd.acceleration.z  = acc(2);
  pose_pub_.publish(cmd);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "risk_voxel_node");
  ros::NodeHandle nh("~");
  nh.param("init_x", pos(0), 0.0);
  nh.param("init_y", pos(1), 0.0);
  nh.param("init_z", pos(2), 0.0);

  RiskVoxel::Ptr risk_voxel;
  risk_voxel.reset(new RiskVoxel());
  risk_voxel->init(nh);

  pose_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 1);

  ros::Timer pose_timer = nh.createTimer(ros::Duration(dt_), poseCallback);

  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
