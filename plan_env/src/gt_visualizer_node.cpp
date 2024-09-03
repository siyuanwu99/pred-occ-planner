/**
 * @file gt_visualizer_node.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief visualize ground truth map for making a better figure
 *
 *  input: /pedestrian1/optitrack/pose
 *         /drone1/local_position/pose
 *  output: /map/occupancy_inflated
 *          /map/future_risk
 *          /map/time
 *
 * @version 1.0
 * @date 2022-10-23
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <plan_env/gt_visualize.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <traj_coordinator/particle.hpp>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "gt_visualizer_node");
  ros::NodeHandle nh1("~");

  GTVisualize gt_visualizer;
  gt_visualizer.init(nh1, nh1);

  ParticleATC::Ptr coordinator = std::make_shared<ParticleATC>(nh1);
  coordinator->init();
  gt_visualizer.setCoordinator(coordinator);

  ros::spin();
  ros::waitForShutdown();
  return 0;
}
