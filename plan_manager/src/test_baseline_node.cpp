/**
 * @file test_baseline_node.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2022-10-24
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <ros/ros.h>
#include <plan_manager/baseline.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_baseline_node");
  ros::NodeHandle   nh("~");
  BaselinePlanner   baseline_planner(nh, BaselineParameters(nh));
  baseline_planner.init();
  // baseline_planner.plan();
  ros::AsyncSpinner spinner(8);  // use 3 threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
