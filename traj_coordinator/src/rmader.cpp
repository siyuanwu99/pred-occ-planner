/**
 * @file rmader.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-12-27
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <traj_coordinator/rmader.hpp>

void RMADER::init() {
  separator_solver_ = new separator::Separator();

  swarm_sub_ =
      nh_.subscribe("/broadcast_traj", 1, &RMADER::trajectoryCallback, dynamic_cast<MADER *>(this));
  nh_.param("drone_id", drone_id_, 0);
  nh_.param("swarm/num_robots", num_robots_, 3);
  nh_.param("swarm/drone_size_x", drone_size_x_, 0.3);
  nh_.param("swarm/drone_size_y", drone_size_y_, 0.3);
  nh_.param("swarm/drone_size_z", drone_size_z_, 0.4);     // avoid z-axis wind effect
  nh_.param("swarm/delay_check_length", delta_dc_, 0.05);  // 50ms by default

  ego_size_ = Eigen::Vector3d(drone_size_x_, drone_size_y_, drone_size_z_);

  /* Initialize Booleans */
  is_checking_                         = false;
  have_received_traj_while_checking_   = false;
  have_received_traj_while_optimizing_ = false;

  swarm_trajs_.reserve(num_robots_ - 1);
  drone_id_to_index_.clear();
  index_to_drone_id_.clear();

  /* Assign traj_id to k in swarm_trajs_ */
  // int j = 0;
  // for (int i = 0; i < num_robots_; i++) {
  //   if (i != drone_id_) {
  //     drone_id_to_index_[i] = j;
  //     index_to_drone_id_[j] = i;
  //     j++;
  //   }
  // }

  /* Pre-allocate Ego Cube */
  ego_cube_.col(0) = Eigen::Vector3d(drone_size_x_ / 2, drone_size_y_ / 2, drone_size_z_ / 2);
  ego_cube_.col(1) = Eigen::Vector3d(drone_size_x_ / 2, -drone_size_y_ / 2, drone_size_z_ / 2);
  ego_cube_.col(2) = Eigen::Vector3d(drone_size_x_ / 2, drone_size_y_ / 2, -drone_size_z_ / 2);
  ego_cube_.col(3) = Eigen::Vector3d(drone_size_x_ / 2, -drone_size_y_ / 2, -drone_size_z_ / 2);
  ego_cube_.col(4) = Eigen::Vector3d(-drone_size_x_ / 2, drone_size_y_ / 2, drone_size_z_ / 2);
  ego_cube_.col(5) = Eigen::Vector3d(-drone_size_x_ / 2, -drone_size_y_ / 2, drone_size_z_ / 2);
  ego_cube_.col(6) = Eigen::Vector3d(-drone_size_x_ / 2, drone_size_y_ / 2, -drone_size_z_ / 2);
  ego_cube_.col(7) = Eigen::Vector3d(-drone_size_x_ / 2, -drone_size_y_ / 2, -drone_size_z_ / 2);
  ROS_INFO("[CA] Robust MADER initialized");
}

/**
 * @brief check if the trajectory is collision free
 *  check if two sets of convex hulls linear separable
 */
bool RMADER::collisionCheck(const Bernstein::Bezier &traj) {
  Eigen::MatrixXd cpts;
  traj.getCtrlPoints(cpts);
  /* checkin: check waypoints */

  std::vector<Eigen::Vector3d> pointsA;
  std::vector<Eigen::Vector3d> pointsB;

  /* Push ego trajectory convex hull to buffer */
  loadVertices(pointsA, cpts);

  /* checkin: check other trajectories */
  for (auto &traj : swarm_trajs_) {
    double          t0 = ros::Time::now().toSec();  // TODO: t0 is not accurate
    Eigen::Vector3d n_k;
    double          d_k;
    pointsB.clear();
    if (traj.time_start < t0 && t0 < traj.time_end) {
      Eigen::MatrixXd cpts = traj.traj.getCtrlPoints();
      // std::cout << "Loading points from B" << std::endl;
      loadVertices(pointsB, cpts);
      ROS_INFO("[CA|A%i] time span (%f, %f)", traj.id, traj.time_start - t0, traj.time_end - t0);

      if (!separator_solver_->solveModel(n_k, d_k, pointsA, pointsB)) {
        ROS_INFO("[CA|A%i] Cannot find linear separation plane", traj.id);
        ROS_WARN("[CA] Drone %i will collides with drone %i", drone_id_, traj.id);
        return false;
      }
    }
  }

  return true;
}

/**
 * @brief check if the trajectory is collision free
 *  check if two sets of convex hulls linear separable
 *
 * @param traj_msg
 * @return true    safe
 * @return false   unsafe
 */
bool RMADER::isSafeAfterOpt(const Bernstein::Bezier &traj) {
  is_checking_ = true;
  /* collision checking starts */
  bool is_safe = collisionCheck(traj);
  is_checking_ = false;
  return is_safe;
}

bool RMADER::isSafeAfterDelayCheck(const Bernstein::Bezier &traj) {
  double t        = 0.0;
  double dc_start = ros::Time::now().toSec();

  while (t < delta_dc_) {
    t            = ros::Time::now().toSec() - dc_start;
    bool is_safe = collisionCheck(traj);
    if (!is_safe) {
      return false;
    }
  }
  ROS_WARN("[CA] Drone %i passed the delay check", drone_id_);
  return true;
}
