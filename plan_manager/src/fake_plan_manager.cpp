/**
 * @file plan_manager.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-11-01
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <plan_manager/fake_plan_manager.h>

void FiniteStateMachineFake::run() {
  nh1_.param("drone_id", drone_id_, 0);
  nh1_.param("use_preset_goal", is_goal_preset_, false);
  nh1_.param("fsm/goal_tolerance", cfgs_.goal_tolerance, 1.0);
  nh1_.param("fsm/replan_tolerance", cfgs_.replan_tolerance, 1.0);
  nh1_.param("fsm/replan_duration", cfgs_.replan_duration, 0.1);
  nh1_.param("fsm/replan_start_time", cfgs_.replan_start_time, 0.4);
  nh1_.param("fsm/colli_check_duration", cfgs_.colli_check_duration, 2.0);
  nh1_.param("fsm/replan_max_failures", cfgs_.replan_max_failures, 3);

  /* Initialize planner */
  planner_.reset(new FakeBaselinePlanner(nh2_, nh3_, nh4_, FakeBaselineParameters(nh2_)));
  planner_->init();

  /* ROS publishers */
  traj_pub_           = nh1_.advertise<traj_utils::BezierTraj>("trajectory", 1);
  broadcast_traj_pub_ = nh1_.advertise<traj_utils::BezierTraj>("/broadcast_traj", 1);
  trigger_sub_ =
      nh1_.subscribe("/traj_start_trigger", 1, &FiniteStateMachineFake::TriggerCallback, this);
  pose_sub_ = nh1_.subscribe("pose", 10, &FiniteStateMachineFake::PoseCallback, this);

  is_exec_triggered_      = false;
  is_odom_received_       = false;
  is_safety_mode_enabled_ = false;
  is_map_updated_         = false;
  is_success_             = false;

  if (is_goal_preset_) {
    is_goal_received_ = true;
    nh1_.param("goal_x", goal_pos_[0], 0.0);
    nh1_.param("goal_y", goal_pos_[1], 0.0);
    nh1_.param("goal_z", goal_pos_[2], 0.0);
    ROS_INFO("[FSM] Receive preset goal (%.2f, %.2f, %.2f)", goal_pos_[0], goal_pos_[1],
             goal_pos_[2]);
  } else {
    is_goal_received_ = false;
  }

  traj_idx_ = 0;

  waypoints_.push(goal_pos_);

  /* Initialize odometry */
  odom_pos_      = Eigen::Vector3d::Zero();
  odom_vel_      = Eigen::Vector3d::Zero();
  odom_acc_      = Eigen::Vector3d::Zero();
  prev_odom_pos_ = Eigen::Vector3d::Zero();

  status_ = FSM_STATUS::INIT; /* Initialize state machine */
  ROS_INFO("[FSM] Initialization complete");
  // wait 2s
  ros::Duration(2.0).sleep();
  fsm_timer_ = nh1_.createTimer(ros::Duration(0.05), &FiniteStateMachineFake::FSMCallback, this);
  vis_timer_ = nh1_.createTimer(ros::Duration(0.1), &FiniteStateMachineFake::visCallback, this);
}

/** ***********************************************************************************************
 * State Machine
 * ***********************************************************************************************/

/**
 * @brief finite state machine for planning
 *  States:
 * - INIT: waiting for input information
 * - WAIT_TARGET: waiting for target information
 * - NEW_PLAN: planning a new trajectory from zero velocity
 * - REPLAN: replanning at the end of current trajectory
 * - EXEC_TRAJ: executing the trajectory
 * - EMERGENCY_REPLAN: replan the trajectory from current position
 * @param event
 */
void FiniteStateMachineFake::FSMCallback(const ros::TimerEvent& event) {
  switch (status_) {
    /* initialize */
    case FSM_STATUS::INIT:
      FSMChangeState(FSM_STATUS::WAIT_TARGET);
      break;

    /* wait for callback */
    case FSM_STATUS::WAIT_TARGET:
      if (!isInputLost() && is_goal_received_) {
        FSMChangeState(FSM_STATUS::NEW_PLAN);  // TODO(CHANGE TO NEW_PLAN)
      } else {
        ROS_INFO_ONCE("[FSM] Waiting for odom[%d] and future risk[%d] ", is_odom_received_,
                      is_map_updated_);
      }
      break;

    /* plan a new trajectory from current position */
    case FSM_STATUS::NEW_PLAN: {
      if (isInputLost()) {
        FSMChangeState(FSM_STATUS::WAIT_TARGET);
      }

      if (checkTimeLapse(1.0)) { /* plan a new trajectory every second */
        traj_start_time_ = ros::Time::now();

        is_success_ =
            planner_->replan(traj_start_time_.toSec(), odom_pos_, odom_vel_, odom_acc_, goal_pos_);

        if (is_success_) {
          publishTrajectory();
          ROS_INFO("[FSM] New trajectory planned");
        } else {
          publishEmptyTrajectory();
          ROS_WARN("[FSM] New trajectory planning failed");
        }
      }
      /** TODO: time delay !!! */
      if (is_exec_triggered_ && is_success_) { /* execute trajectory */
        FSMChangeState(FSM_STATUS::EXEC_TRAJ);
      }
      break;
    }

    /* execute the trajectory, replan when current traj is about to finish */
    case FSM_STATUS::EXEC_TRAJ: {
      if (isInputLost()) {
        FSMChangeState(FSM_STATUS::WAIT_TARGET);
      }
      if (!is_exec_triggered_) {
        FSMChangeState(FSM_STATUS::NEW_PLAN);
      }

      std::cout << termcolor::bright_red << "Target: " << waypoints_.front().transpose() << " now "
                << odom_pos_.transpose() << std::endl;
      if (checkTimeLapse(cfgs_.replan_duration)) {
        FSMChangeState(FSM_STATUS::REPLAN);
      }

      if (!planner_->isTrajSafe(cfgs_.colli_check_duration)) {
        ROS_WARN("[FSM] Not safe, replan");
        FSMChangeState(FSM_STATUS::REPLAN);
      }

      if (isGoalReached(odom_pos_)) {
        ROS_INFO("[FSM] Goal reached");
        FSMChangeState(FSM_STATUS::GOAL_REACHED);
      }
      break;
    }

    /* replan from a future position on the current trajectory */
    case FSM_STATUS::REPLAN:
      if (isInputLost()) {
        FSMChangeState(FSM_STATUS::WAIT_TARGET);
      } else {
        ros::Time t1      = ros::Time::now();
        traj_start_time_  = t1 + ros::Duration(cfgs_.replan_start_time);
        double start_time = traj_start_time_.toSec();

        Eigen::Vector3d pos = planner_->getPos(start_time);
        Eigen::Vector3d vel = planner_->getVel(start_time);
        Eigen::Vector3d acc = planner_->getAcc(start_time);

        bool is_success_ = planner_->replan(start_time, pos, vel, acc, goal_pos_);
        bool is_safe     = true;
        // bool is_safe     = planner_->isTrajSafe(2.0);

        // bool is_finished = isGoalReached(odom_pos_);
        // std::cout << termcolor::bright_red << "Target: " << goal_pos_.transpose() << " now "
        //           << odom_pos_ << std::endl;
        //
        // if (is_finished) {
        //   ROS_INFO("[FSM] Goal reached");
        //   FSMChangeState(FSM_STATUS::GOAL_REACHED);
        // }

        if (is_success_ && is_safe) { /* publish trajectory */
          num_replan_failures_ = 0;
          ROS_INFO("[FSM] Replanning success, costs %f", (ros::Time::now() - t1).toSec());
          publishTrajectory();
          FSMChangeState(FSM_STATUS::EXEC_TRAJ);
        } else {
          ROS_WARN("[FSM] Replanning failed");
          num_replan_failures_++;
          if (num_replan_failures_ > cfgs_.replan_max_failures) {
            // if (planner_->isPrevTrajFinished(ros::Time::now().toSec() + cfgs_.replan_start_time))
            // {
            FSMChangeState(FSM_STATUS::NEW_PLAN);
            publishEmptyTrajectory();
            traj_start_time_ = ros::Time::now() - ros::Duration(1.0);  // force new plan immediately
          }
        }
      }
      break;

    /* emergency replan */
    // case FSM_STATUS::EMERGENCY_REPLAN:
    //   if (!_is_safety_mode_enabled) {
    //     FSMChangeState(FSM_STATUS::NEW_PLAN);
    //   } else {
    //     bool is_success = localReplan(PLAN_TYPE::EMERGENCY);
    //     if (is_success) {
    //       publishTrajectory();
    //       FSMChangeState(FSM_STATUS::EXEC_TRAJ);
    //     } else {
    //       ROS_WARN("Emergency replanning failed");
    //     }
    //   }
    //   break;

    /* reached the goal, clear the buffer and wait new goal */
    case FSM_STATUS::GOAL_REACHED:
      is_goal_received_  = false;  // reset goal
      is_exec_triggered_ = false;
      waypoints_.pop();
      ROS_INFO("[FSM] Goal reached");
      ros::shutdown();
      FSMChangeState(FSM_STATUS::WAIT_TARGET);
      break;

    case FSM_STATUS::EXIT:
      ROS_INFO("[FSM] Exit");
      break;

    default:
      ROS_ERROR("[FSM] Invalid FSM state");
      break;
  }
}

/**
 * @brief change the state of the finite state machine
 * @param state
 */
void FiniteStateMachineFake::FSMChangeState(FSM_STATUS new_state) {
  FSMPrintState(new_state);
  status_ = new_state;
}

/**
 * @brief print the current state of the finite state machine via termcolor
 * This function is used for debugging purposes
 */
void FiniteStateMachineFake::FSMPrintState(FSM_STATUS new_state) {
  static string state_str[8] = {"INIT",      "WAIT_TARGET", "NEW_PLAN",     "REPLAN",
                                "EXEC_TRAJ", "EMERGENCY",   "GOAL_REACHED", "EXIT"};
  std::cout << termcolor::dark << termcolor::on_bright_green << "[UAV" << drone_id_
            << " FSM] status " << termcolor::bright_cyan << termcolor::on_white
            << state_str[static_cast<int>(status_)] << " >> "
            << state_str[static_cast<int>(new_state)] << termcolor::reset << std::endl;
}

/**
 * @brief Trigger can be used to start the planner and receive the goal position
 *
 * @param msg
 */
void FiniteStateMachineFake::TriggerCallback(const geometry_msgs::PoseStampedPtr& msg) {
  if (is_exec_triggered_) {
    ROS_INFO("[FSM] Execution has already triggered");
    return;
  }
  ROS_WARN("[FSM] trigger received");
  is_exec_triggered_ = true;

  if (!is_goal_received_) {
    if (waypoints_.empty()) {
      goal_pos_.x() = msg->pose.position.x;
      goal_pos_.y() = msg->pose.position.y;
      goal_pos_.z() = msg->pose.position.z;
      waypoints_.push(goal_pos_);
      ROS_INFO("[FSM] New goal received: %f, %f, %f", goal_pos_.x(), goal_pos_.y(), goal_pos_.z());
    } else {
      goal_pos_ = waypoints_.front();
      ROS_INFO("[FSM] Existing waypoints: %f, %f, %f", goal_pos_.x(), goal_pos_.y(), goal_pos_.z());
      ROS_INFO("[FSM] remaining waypoints: %d", (int)waypoints_.size());
    }
    is_goal_received_ = true;
  }

  planner_->setStartTime(ros::Time::now().toSec());
  traj_start_time_ = ros::Time::now();
}

/**
 * @brief get current position and attitude from odometry
 * @param msg
 */
void FiniteStateMachineFake::PoseCallback(const geometry_msgs::PoseStampedPtr& msg) {
  if (!is_state_locked_) {
    is_state_locked_  = true;
    odom_pos_.x()     = msg->pose.position.x;
    odom_pos_.y()     = msg->pose.position.y;
    odom_pos_.z()     = msg->pose.position.z;
    odom_att_.x()     = msg->pose.orientation.x;
    odom_att_.y()     = msg->pose.orientation.y;
    odom_att_.z()     = msg->pose.orientation.z;
    odom_att_.w()     = msg->pose.orientation.w;
    is_odom_received_ = true;
  }
  is_state_locked_ = false;

  if (!is_velocity_received_) {
    static double prev_odom_time = msg->header.stamp.toSec();
    double        dt             = msg->header.stamp.toSec() - prev_odom_time;
    if (dt < 0.0) {
      ROS_WARN(
          "[FSM] Odometry negative time difference. Current: msg->header.stamp.toSec() = %f, "
          "prev_odom_time = %f, dt = %f",
          msg->header.stamp.toSec(), prev_odom_time, dt);
      return;
    }
    odom_vel_.x() = (msg->pose.position.x - prev_odom_pos_.x()) / dt;
    odom_vel_.y() = (msg->pose.position.y - prev_odom_pos_.y()) / dt;
    odom_vel_.z() = (msg->pose.position.z - prev_odom_pos_.z()) / dt;

    prev_odom_time = msg->header.stamp.toSec();
    prev_odom_pos_ = odom_pos_;
  }
}

/**
 * @brief update rviz visualization
 */
void FiniteStateMachineFake::visCallback(const ros::TimerEvent& event) {
  planner_->visualizer_->visualizeFOV(odom_pos_, odom_att_, 84, 48, 5);
}

/**
 * @brief callback function for the click event in rviz
 * @param msg
 */
void FiniteStateMachineFake::clickCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  goal_pos_(0) = msg->pose.position.x;
  goal_pos_(1) = msg->pose.position.y;
  goal_pos_(2) = 1;
  ROS_INFO("Start position: (%f, %f, %f)", odom_pos_(0), odom_pos_(1), odom_pos_(2));
  ROS_INFO("End position: (%f, %f, %f)", goal_pos_(0), goal_pos_(1), goal_pos_(2));
  // TODO: change FSM state
  // TODO: revise this function and logic
}

/**********************************************************
 * Utility Functions
 * ********************************************************/

void FiniteStateMachineFake::publishTrajectory() {
  ROS_INFO("[FSM] Publishing trajectory");
  traj_idx_++;
  Trajectory traj = planner_->getTrajectory(); /* TODO: reduce copy */
  int        N    = traj.getOrder();
  TrajMsg    msg;

  msg.drone_id   = drone_id_;
  msg.traj_id    = traj_idx_;
  msg.start_time = traj_start_time_;
  msg.pub_time   = ros::Time::now();
  msg.order      = N;

  int piece_num = traj.getNumPieces();

  msg.duration.resize(piece_num);
  for (int i = 0; i < piece_num; i++) {
    msg.duration[i] = traj[i].getDuration();
  }

  Eigen::MatrixXd cpts;
  traj.getCtrlPoints(cpts);
  int R = cpts.rows();
  msg.cpts.resize(R);
  for (int i = 0; i < R; i++) {
    msg.cpts[i].x = cpts(i, 0);
    msg.cpts[i].y = cpts(i, 1);
    msg.cpts[i].z = cpts(i, 2);
  }

  traj_pub_.publish(msg);
  broadcast_traj_pub_.publish(msg);
}

/**
 * @brief publish empty trajectory
 */
void FiniteStateMachineFake::publishEmptyTrajectory() {
  ROS_INFO("[FSM] Publishing emergency trajectory");
  traj_idx_++;
  TrajMsg msg;
  msg.drone_id   = drone_id_;
  msg.traj_id    = traj_idx_;
  msg.start_time = ros::Time::now();
  msg.pub_time   = ros::Time::now();
  msg.order      = 4;
  msg.duration.resize(1);
  msg.duration[0] = 1.5;
  msg.cpts.resize(5);
  for (int i = 0; i < 5; i++) {
    msg.cpts[i].x = odom_pos_(0);
    msg.cpts[i].y = odom_pos_(1);
    msg.cpts[i].z = odom_pos_(2);
  }
  traj_pub_.publish(msg);
  broadcast_traj_pub_.publish(msg);
}
