/**
 * @file mader_deconfliction.hpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-10-31
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef MADER_DECONFLICTION_H
#define MADER_DECONFLICTION_H

#include <ros/ros.h>
#include <traj_utils/BezierTraj.h>
#include <Eigen/Eigen>
#include <map>
#include <memory>
#include <queue>
#include <traj_utils/bernstein.hpp>
#include <vector>
#include "separator.hpp"

struct SwarmTraj {
  int              id;
  ros::Time        time_received;
  ros::Time        time_start;
  ros::Time        time_end;
  double           duration;
  Eigen::MatrixX3d control_points;
};

// we should give some hyperparameter to control the size of the cube
// Assume our robot is homogenous, i.e. all robots have the same size

class MADER {
 public:
  /* constructor */
  MADER(ros::NodeHandle &nh) : nh_(nh) {}
  ~MADER() {}

  /* functions */
  void init();
  void reset();
  void trajectoryCallback(const traj_utils::BezierTraj::ConstPtr &traj_msg);
  bool isSafeAfterOpt(const Bernstein::Bezier &traj);
  bool isSafeAfterChk() {
    if (have_received_traj_while_checking_) {
      have_received_traj_while_checking_ = false;  // reset
      return false;
    }
    return true;
  }
  bool updateTrajObstacles(/* arguments */);
  void getAgentsSize(Eigen::Vector3d &size) { size = ego_size_; }
  void getAgentsTrajectory(std::vector<Eigen::Vector3d> &points, int idx_agent, double dt);
  void getObstaclePoints(std::vector<Eigen::Vector3d> &pts, double t);
  /**
   * @brief input vertices of trajectory convex hull, get Minkowski sum of the convex
   * hull and ego polytope, and push these vertices into the buffer `pts`
   * @param pts : points buffer
   * @param cpts: control points (vertices of trajectory convex hull)
   */
  void loadVertices(std::vector<Eigen::Vector3d> &pts, Eigen::MatrixXd &cpts);
  int  getNumAgents() { return num_robots_; }
  typedef std::shared_ptr<MADER> Ptr;

 private:
  ros::NodeHandle nh_;
  ros::Subscriber swarm_sub_;
  /* variables */
  std::vector<std::queue<SwarmTraj>> swarm_trajs_;
  std::map<int, int>                 drone_id_to_index_;
  std::map<int, int>                 index_to_drone_id_;

  bool is_planner_initialized_;
  bool have_received_traj_while_checking_;
  bool have_received_traj_while_optimizing_;
  bool is_traj_safe_;
  bool is_checking_;

  int    drone_id_;
  int    num_robots_;
  double drone_size_x_;
  double drone_size_y_;
  double drone_size_z_;

  Eigen::Vector3d             ego_size_;
  Eigen::Matrix<double, 3, 8> ego_cube_; /* Eight vertices of the ego cube */

  separator::Separator *separator_solver_;
};

#endif  // MADER_DECONFLICTION_H
