/**
 * @file fake_rmader_node.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2023-01-09
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <ros/ros.h>
#include "plan_manager/rmader/fake_plan_manager.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "planner_node");
  ros::NodeHandle              nh("~");
  FakeRobustFiniteStateMachine plan_manager(nh);
  plan_manager.run();
  ros::AsyncSpinner spinner(3);  // use 3 threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
