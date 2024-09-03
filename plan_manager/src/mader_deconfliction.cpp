/**
 * @file mader_deconfliction.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-10-28
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <plan_manager/mader_deconfliction.hpp>

void MADER::init() {
  separator_solver_ = new separator::Separator();

  swarm_sub_ = nh_.subscribe("/broadcast_traj", 1, &MADER::trajectoryCallback, this);
  nh_.param("drone_id", drone_id_, 0);
  nh_.param("swarm/num_robots", num_robots_, 3);
  nh_.param("swarm/drone_size_x", drone_size_x_, 0.3);
  nh_.param("swarm/drone_size_y", drone_size_y_, 0.3);
  nh_.param("swarm/drone_size_z", drone_size_z_, 0.4);  // avoid z-axis wind effect

  ego_size_ = Eigen::Vector3d(drone_size_x_, drone_size_y_, drone_size_z_);

  /* Initialize Booleans */
  is_checking_                         = false;
  have_received_traj_while_checking_   = false;
  have_received_traj_while_optimizing_ = false;

  swarm_trajs_.resize(num_robots_ - 1);
  drone_id_to_index_.clear();
  index_to_drone_id_.clear();

  /* Assign traj_id to k in swarm_trajs_ */
  int j = 0;
  for (int i = 0; i < num_robots_; i++) {
    if (i != drone_id_) {
      drone_id_to_index_[i] = j;
      index_to_drone_id_[j] = i;
      std::cout << "drone_id_to_index_[" << i << "] = " << j << std::endl;
      j++;
    }
  }

  /* Pre-allocate Ego Cube */
  ego_cube_.col(0) = Eigen::Vector3d(drone_size_x_ / 2, drone_size_y_ / 2, drone_size_z_ / 2);
  ego_cube_.col(1) = Eigen::Vector3d(drone_size_x_ / 2, -drone_size_y_ / 2, drone_size_z_ / 2);
  ego_cube_.col(2) = Eigen::Vector3d(drone_size_x_ / 2, drone_size_y_ / 2, -drone_size_z_ / 2);
  ego_cube_.col(3) = Eigen::Vector3d(drone_size_x_ / 2, -drone_size_y_ / 2, -drone_size_z_ / 2);
  ego_cube_.col(4) = Eigen::Vector3d(-drone_size_x_ / 2, drone_size_y_ / 2, drone_size_z_ / 2);
  ego_cube_.col(5) = Eigen::Vector3d(-drone_size_x_ / 2, -drone_size_y_ / 2, drone_size_z_ / 2);
  ego_cube_.col(6) = Eigen::Vector3d(-drone_size_x_ / 2, drone_size_y_ / 2, -drone_size_z_ / 2);
  ego_cube_.col(7) = Eigen::Vector3d(-drone_size_x_ / 2, -drone_size_y_ / 2, -drone_size_z_ / 2);
}

/**
 * @brief Save boardcasted trajectory to swarm_trajs_
 * @param traj_msg
 */
void MADER::trajectoryCallback(const traj_utils::BezierTraj::ConstPtr &traj_msg) {
  int id = traj_msg->drone_id;

  /* filter out ego trajectories */
  if (id == drone_id_) {
    return;
  }

  if (is_checking_) {
    have_received_traj_while_checking_ = true;
    ROS_INFO("Received traj from agent %i while checking agent %i", id, drone_id_);
  } else {
    have_received_traj_while_optimizing_ = false;
  }

  int k = drone_id_to_index_[id];

  /* remove old trajectories from swarm_trajs_ buffer */
  ros::Time now = ros::Time::now();
  while (!swarm_trajs_[k].empty()) {
    /* If trajectory ends earlier */
    if (ros::Time::now() > swarm_trajs_[k].front().time_end) {
      swarm_trajs_[k].pop();
    } else {
      break;
    }
  }
  ROS_INFO("Traj size after remove: %d", (int)swarm_trajs_[k].size());

  /* update swarm_trajs_ */
  int n_seg = traj_msg->duration.size();
  int order = traj_msg->order + 1;

  SwarmTraj traj;
  traj.id            = traj_msg->drone_id;
  traj.time_received = ros::Time::now();
  traj.time_start    = traj_msg->start_time;

  ROS_INFO("Agent %i traj start time: %f", traj.id, traj.time_start.toSec());
  ROS_INFO("Agent %i traj received time: %f", traj.id, traj.time_received.toSec());

  for (int i = 0; i < n_seg; i++) {
    traj.duration   = traj_msg->duration[i];
    traj.time_start = traj_msg->start_time + ros::Duration(traj.duration) * i;
    traj.time_end   = traj.time_start + ros::Duration(traj.duration);
    traj.control_points.resize(order, 3);
    for (int j = 0; j < order; j++) {
      int k = j + order * i;
      traj.control_points.row(j) << traj_msg->cpts[k].x, traj_msg->cpts[k].y, traj_msg->cpts[k].z;
    }
    swarm_trajs_[k].push(traj);
  }

  /* print intermediate results */
  ROS_INFO("Drone %d has %d trajectories for drone %d", drone_id_, (int)swarm_trajs_[k].size(), k);
}

/**
 * @brief obtains control points of other drones in the prediction horizon
 *
 * @param pts input obstacle points buffer
 * @param t prediction horizon
 */
void MADER::getObstaclePoints(std::vector<Eigen::Vector3d> &pts, double horizon) {
  ros::Time t_start = ros::Time::now() + ros::Duration(0.05); /* Start time of planned trajectory */
  ros::Time t_end   = t_start + ros::Duration(horizon);       /* End time of planned trajectory */
  for (int i = 0; i < num_robots_ - 1; i++) {                 /* iterate all robots in the buffer */
    if (swarm_trajs_[i].empty()) {
      ROS_INFO("Drone %d has no trajectories for drone %d", drone_id_, index_to_drone_id_[i]);
      continue;
    }
    std::queue<SwarmTraj> traj_queue(swarm_trajs_[i]); /* copy the queue */

    SwarmTraj traj;
    while (!traj_queue.empty()) { /* check all trajectories preserved in the queue */
      traj = traj_queue.front();
      ROS_INFO("t_start: 0 , t_end: %f | traj: (%f, %f)", t_end.toSec() - t_start.toSec(),
               traj.time_start.toSec() - t_start.toSec(), traj.time_end.toSec() - t_start.toSec());
      /* If checked trajectory ends before planned trajectory */
      if (t_start > traj.time_end) {
        traj_queue.pop();
        continue;
      }

      /* If checked trajectory coincide with planned trajectory */
      if (t_end > traj.time_start || t_start < traj.time_end) {
        ROS_INFO("Drone %d : pushing %d obstacle points", traj.id, (int)traj.control_points.rows());
        ROS_INFO("t_start: 0 , t_end: %f | traj: (%f, %f)", t_end.toSec() - t_start.toSec(),
                 traj.time_start.toSec() - t_start.toSec(),
                 traj.time_end.toSec() - t_start.toSec());
        Eigen::MatrixXd cpts = traj.control_points;
        loadVertices(pts, cpts);
      }

      traj_queue.pop();
    }
  }
}

/**
 * @brief check if the trajectory is collision free
 *
 * @param traj_msg
 * @return true    safe
 * @return false   unsafe
 */
bool MADER::isSafeAfterOpt(const Bernstein::Bezier &traj) {
  is_checking_ = true;
  /* collision checking starts */

  double          t = traj.getDuration();
  Eigen::MatrixXd cpts;
  traj.getCtrlPoints(cpts);
  /* checkin: check waypoints */

  std::vector<Eigen::Vector3d> pointsA;
  std::vector<Eigen::Vector3d> pointsB;

  /* Push ego trajectory convex hull to buffer */
  /* for (int i = 0; i < cpts.rows(); i++) {
    pointsA.push_back(cpts.row(i));
  } */
  loadVertices(pointsA, cpts);

  /* checkin: check other trajectories */
  for (int k = 0; k < swarm_trajs_.size(); k++) { /* iterate all robots in the buffer */
    if (!swarm_trajs_[k].empty()) {               /* if the buffer is not empty */
      while (swarm_trajs_[k].front().time_end < ros::Time::now()) {
        swarm_trajs_[k].pop();
        if (swarm_trajs_[k].empty()) {
          break;
        }
      }
    }

    if (!swarm_trajs_[k].empty()) {
      Eigen::Vector3d n_k;
      double          d_k;
      pointsB.clear();
      Eigen::MatrixXd cpts = swarm_trajs_[k].front().control_points;
      std::cout << "Loading points from B" << std::endl;
      loadVertices(pointsB, cpts);
      /* for (int i = 0; i < cpts.rows(); i++) {
        pointsB.push_back(cpts.row(i));
      } */
      std::cout << "Time: ("
                << swarm_trajs_[k].front().time_start.toSec() - ros::Time::now().toSec() << ", "
                << swarm_trajs_[k].front().time_end.toSec() - ros::Time::now().toSec() << ")";
      std::cout << "\tDrone " << drone_id_ << " is checking with drone " << index_to_drone_id_[k]
                << "\tEgo Buffer size: " << pointsA.size()
                << "  Obstacle Buffer size: " << pointsB.size() << std::endl;

      if (!separator_solver_->solveModel(n_k, d_k, pointsA, pointsB)) {
        ROS_WARN("Drone %d will collides with drone %d", drone_id_, k);
        is_checking_ = false;
        return false;
      }
    }
  }
  is_checking_ = false;
  return true;
}

/**
 * @brief input vertices of trajectory convex hull, get Minkowski sum of the convex
 * hull and ego polytope, and push these vertices into the buffer `pts`
 * @param pts : points buffer
 * @param cpts: control points (vertices of trajectory convex hull)
 */
void MADER::loadVertices(std::vector<Eigen::Vector3d> &pts, Eigen::MatrixXd &cpts) {
  for (int i = 0; i < cpts.rows(); i++) {
    /* Add trajectory control point */
    Eigen::Vector3d pt = cpts.row(i);
    for (int j = 0; j < 8; j++) {
      pts.push_back(pt + ego_cube_.col(j));
    }
  }
}

/**
 * @brief
 * @param points  trajectory waypoints to be filled
 * @param i : index of the agent
 * @param tf
 */
void MADER::getAgentsTrajectory(std::vector<Eigen::Vector3d> &points, int idx_agent, double dt) {
  ros::Time t0 = ros::Time::now() + ros::Duration(dt);

  std::queue<SwarmTraj> traj_queue = swarm_trajs_[idx_agent];
  while (!traj_queue.empty()) {
    Eigen::MatrixXd cpts = traj_queue.front().control_points;

    if (traj_queue.front().time_start < t0 && traj_queue.front().time_end > t0) {
      /* Adding trajectory points to the buffer */
      /* Here we use control points as an approximation */
      for (int j = 0; j < 2; j++) {
        /* control point number: 0 2 */
        Eigen::Vector3d pt = cpts.row(2 * j);
        points.push_back(pt);
        ROS_INFO_STREAM("[CA|Traj" << idx_agent << "] got trajectory at "
                                   << t0.toSec() - traj_queue.front().time_start.toSec()
                                   << pt.transpose());
        t0 += ros::Duration(dt);
        if (t0 > traj_queue.front().time_end) {
          break;
        }
      }
    } else {
      traj_queue.pop();
      continue;
    }
  }
}
