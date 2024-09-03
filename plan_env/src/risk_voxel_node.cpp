/**
 * @file risk_voxel_node.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-10-23
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <plan_env/risk_voxel.h>
// #include <plan_env/fake_dsp_map.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "risk_voxel_node");
  ros::NodeHandle nh("~");

  RiskVoxel::Ptr risk_voxel;
  risk_voxel.reset(new RiskVoxel());
  risk_voxel->init(nh);
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
