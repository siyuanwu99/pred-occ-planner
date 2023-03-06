/**
 * @file planner_node.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-11-01
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <ros/ros.h>
#include "plan_manager/fake_plan_manager.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "planner_node");
  ros::NodeHandle        nh("~");
  FiniteStateMachineFake plan_manager(nh);
  plan_manager.run();
  ros::AsyncSpinner spinner(3);  // use 3 threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
