/**
 * @file mader.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-12-27
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <traj_coordinator/mader.hpp>

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

  swarm_trajs_.reserve(num_robots_ - 1);
  drone_id_to_index_.clear();
  index_to_drone_id_.clear();

  // /* Assign traj_id to k in swarm_trajs_ */
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
 * @brief Save boardcasted trajectory to swarm_trajs_
 * @param traj_msg
 */
void MADER::trajectoryCallback(const traj_utils::BezierTraj::ConstPtr &traj_msg) {
  int id = traj_msg->drone_id;

  /* filter out ego trajectories */
  if (id == drone_id_) {
    return;
  }
  // int k = drone_id_to_index_[id];

  if (is_checking_) {
    have_received_traj_while_checking_ = true;
    ROS_INFO("[CA|A%i] trajectory received during checking", id);
  } else {
    have_received_traj_while_optimizing_ = false;
  }

  // /* remove old trajectories from swarm_trajs_ buffer */
  // ros::Time now = ros::Time::now();
  // while (!swarm_trajs_[k].empty()) {
  //   /* If trajectory ends earlier */
  //   if (now > swarm_trajs_[k].front().time_end) {
  //     swarm_trajs_[k].pop();
  //   } else {
  //     break;
  //   }
  // }
  // ROS_INFO("[CA|A%i] %d traj in buffer after removal", id, (int)swarm_trajs_[k].size());

  /* update swarm_trajs_ */
  int n_seg = traj_msg->duration.size();
  int order = traj_msg->order + 1;

  SwarmTraj traj;
  traj.id            = traj_msg->drone_id;
  traj.time_received = ros::Time::now().toSec();
  ros::Time time_pub = traj_msg->pub_time;
  // ros::Time time_start = traj_msg->start_time;

  ROS_INFO("[CA|A%i] traj communication delay %f ms", traj.id,
           (traj.time_received - time_pub.toSec()) * 1000);

  traj.time_start = traj_msg->start_time.toSec();
  double t_end    = traj.time_start;

  std::vector<double> durations;
  Eigen::MatrixX3d    cpts;
  cpts.resize(order * n_seg, 3);
  for (int i = 0; i < n_seg; i++) {
    t_end += traj_msg->duration[i];
    durations.push_back(traj_msg->duration[i]);
    for (int j = 0; j < order; j++) {
      int k = j + i * order;
      cpts.row(k) << traj_msg->cpts[k].x, traj_msg->cpts[k].y, traj_msg->cpts[k].z;
    }
  }
  traj.time_end = t_end;
  traj.traj     = Traj(durations, cpts);

  std::vector<SwarmTraj>::iterator obs_ptr =
      std::find_if(swarm_trajs_.begin(), swarm_trajs_.end(),
                   [=](const SwarmTraj &tmp) { return tmp.id == traj.id; });

  bool exists_in_local_map = (obs_ptr != std::end(swarm_trajs_));
  if (exists_in_local_map) {
    *obs_ptr = traj;
    ROS_INFO("[CA|A%i] traj updated", traj.id);
  } else {
    swarm_trajs_.push_back(traj);
    ROS_INFO("[CA|A%i] traj created", traj.id);
  }
}

/**
 * @brief obtains control points of other drones in the prediction horizon
 *
 * @param pts input obstacle points buffer
 * @param t prediction horizon
 */
void MADER::getObstaclePoints(std::vector<Eigen::Vector3d> &pts, double horizon) {
  double t0 = ros::Time::now().toSec();  // TODO: t0 is not accurate

  double tf = t0 + horizon;
  for (auto &traj : swarm_trajs_) {
    if (tf > traj.time_start || t0 < traj.time_end) {
      Eigen::MatrixXd cpts;
      traj.traj.getCtrlPoints(cpts);
      ROS_INFO("[CA|A%d] pushing %d obstacle points", traj.id, (int)cpts.rows());
      ROS_INFO("t_start: 0 , t_end: %f | traj: (%f, %f)", tf - t0, traj.time_start - t0,
               traj.time_end - t0);
      loadVertices(pts, cpts);
    }
  }
}

/**
 * @brief check if the trajectory is collision free
 *  check if two sets of convex hulls linear separable
 *
 * @param traj_msg
 * @return true    safe
 * @return false   unsafe
 */
bool MADER::isSafeAfterOpt(const Traj &traj) {
  is_checking_ = true;
  /* collision checking starts */

  // double          t = traj.getDuration();
  Eigen::MatrixXd cpts;
  traj.getCtrlPoints(cpts);
  /* checkin: check waypoints */

  std::vector<Eigen::Vector3d> pointsA;
  std::vector<Eigen::Vector3d> pointsB;

  /* Push ego trajectory convex hull to buffer */
  loadVertices(pointsA, cpts);

  /* checkin: check other trajectories */
  for (auto &traj : swarm_trajs_) {
    if (traj.id == drone_id_) {
      continue;
    }
    double          t0 = ros::Time::now().toSec();  // TODO: t0 is not accurate
    Eigen::Vector3d n_k;
    double          d_k;
    pointsB.clear();
    if (traj.time_start < t0 && t0 < traj.time_end) {
      Eigen::MatrixXd cpts = traj.traj.getCtrlPoints();

      /* remove passed control points */
      int order     = traj.traj.getOrder() + 1;
      int piece_idx = traj.traj.locatePiece(t0 - traj.time_start);
      cpts          = cpts.bottomRows(cpts.rows() - piece_idx * order);

      /* add current control points */
      loadVertices(pointsB, cpts);
      ROS_INFO("[CA|A%i] time span (%f, %f)", traj.id, traj.time_start - t0, traj.time_end - t0);

      if (!separator_solver_->solveModel(n_k, d_k, pointsA, pointsB)) {
        ROS_INFO("[CA|A%i] Cannot find linear separation plane", traj.id);
        ROS_WARN("[CA] Drone %i will collides with drone %i", drone_id_, traj.id);
        is_checking_ = false;
        return false;
      }
    }
    // std::cout << "Loading points from B" << std::endl;
  }
  is_checking_ = false;
  return true;
}

/**
 * @brief input vertices of trajectory convex hull, get Minkowski sum of the convex
 * hull and ego polytope, and push these vertices into the buffer `pts`
 * @param pts : points buffer to be filled
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
 * @brief DEPRECATED
 * @param points  trajectory waypoints to be filled
 * @param i : index of the agent
 * @param tf
 */
void MADER::getAgentsTrajectory(std::vector<Eigen::Vector3d> &points, int idx_agent, double dt) {
  double t0 = ros::Time::now().toSec() + dt;

  std::vector<SwarmTraj>::iterator obs_ptr =
      std::find_if(swarm_trajs_.begin(), swarm_trajs_.end(),
                   [=](const SwarmTraj &tmp) { return tmp.id == idx_agent; });

  if (obs_ptr == swarm_trajs_.end() || idx_agent == drone_id_) {
    return;
  }
  Eigen::MatrixXd cpts = obs_ptr->traj.getCtrlPoints();
  if (obs_ptr->time_start < t0 && obs_ptr->time_end > t0) {
    /* Adding trajectory points to the buffer */
    /* Here we use control points as an approximation */
    for (int j = 0; j < 2; j++) {
      /* control point number: 0 2 */
      Eigen::Vector3d pt = cpts.row(2 * j);
      points.push_back(pt);
      ROS_INFO_STREAM("[CA|A" << idx_agent << "] got trajectory at " << t0 - obs_ptr->time_start
                              << " | pt: " << pt.transpose());
      return;
    }
  }
}

/**
 * @brief get waypoints of the agent 'idx_agent' at time 't'
 * @param pts : waypoints buffer to be filled
 * @param idx_agent
 * @param t
 */
void MADER::getWaypoints(std::vector<Eigen::Vector3d> &pts, int idx_agent, double t0) {
  std::vector<SwarmTraj>::iterator ptr =
      std::find_if(swarm_trajs_.begin(), swarm_trajs_.end(),
                   [=](const SwarmTraj &tmp) { return tmp.id == idx_agent; });

  if (ptr == swarm_trajs_.end() || idx_agent == drone_id_) {
    return;
  }

  Eigen::MatrixXd cpts = ptr->traj.getCtrlPoints();
  if (ptr->time_start < t0 && ptr->time_end > t0) {
    /* Adding trajectory points to the buffer */
    /* Here we use control points as an approximation */
    Eigen::Vector3d pt = ptr->traj.getPos(t0 - ptr->time_start);
    /* control point number: 0 2 */
    pts.push_back(pt);
    ROS_INFO_STREAM("[CA|A" << idx_agent << "] got trajectory at " << t0 - ptr->time_start
                            << " | pt: " << pt.transpose());
    return;
  } else {
    pts.push_back(ptr->traj.getPos(ptr->traj.getDuration())); /* push last point */
    ROS_INFO("[CA|A%i] failed to get trajectory at time, ts: %f, te: %f", idx_agent,
             ptr->time_start - t0, ptr->time_end - t0);
  }
}
