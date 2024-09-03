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

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include "plan_manager/fake_plan_manager.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "planner_node");
  ros::NodeHandle nh1("~");
  ros::NodeHandle nh2("~");
  ros::NodeHandle nh3("~");
  ros::NodeHandle nh4("~");

  ros::CallbackQueue custom_queue1;
  ros::CallbackQueue custom_queue2;
  ros::CallbackQueue custom_queue3;
  ros::CallbackQueue custom_queue4;

  nh1.setCallbackQueue(&custom_queue1);
  nh2.setCallbackQueue(&custom_queue2);
  nh3.setCallbackQueue(&custom_queue3);
  nh4.setCallbackQueue(&custom_queue4);

  FiniteStateMachineFake plan_manager(nh1, nh2, nh3, nh4);
  plan_manager.run();

  ros::AsyncSpinner spinner1(3, &custom_queue1);  // spinner for FSM
  ros::AsyncSpinner spinner2(1, &custom_queue2);  // spinner for planner
  ros::AsyncSpinner spinner3(2, &custom_queue3);  // spinner for coordinator
  ros::AsyncSpinner spinner4(3, &custom_queue4);  // spinner for map
  spinner1.start();
  spinner2.start();
  spinner3.start();
  spinner4.start();

  ros::waitForShutdown();
  return 0;
}
