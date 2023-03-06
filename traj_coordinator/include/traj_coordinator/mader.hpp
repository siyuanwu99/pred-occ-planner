/**
 * @file mader.hpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-12-27
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef MADER_H
#define MADER_H

#include <ros/ros.h>
#include <traj_utils/BezierTraj.h>
#include <Eigen/Eigen>
#include <map>
#include <memory>
#include <queue>
#include <traj_utils/bernstein.hpp>
#include <vector>
#include "separator.hpp"

typedef Bernstein::BernsteinPiece Piece;
typedef Bernstein::Bezier         Traj;

struct SwarmTraj {
  int    id;
  double duration;
  double time_received;  // ros time
  double time_start;     // ros time
  double time_end;       // ros time
  Traj   traj;
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
      ROS_INFO("[CA] Commited during checking");
      return false;
    }
    return true;
  }
  /**
   * @brief get the size of the ego agent
   * @param size : size of the ego agent (x, y, z)
   */
  void            getAgentsSize(Eigen::Vector3d &size) { size = ego_size_; }
  Eigen::Vector3d getAgentsSize() { return ego_size_; }
  void getAgentsTrajectory(std::vector<Eigen::Vector3d> &points, int idx_agent, double dt);
  void getObstaclePoints(std::vector<Eigen::Vector3d> &pts, double t);
  /**
   * @brief get waypoints of the agent 'idx_agenq' at time 't'
   * @param pts: points buffer
   * @param idx_agent: index of the agent
   * @param t: global time
   */
  void getWaypoints(std::vector<Eigen::Vector3d> &pts, int idx_agent, double t);

  /**
   * @brief input vertices of trajectory convex hull, get Minkowski sum of the convex
   * hull and ego polytope, and push these vertices into the buffer `pts`
   * @param pts : points buffer
   * @param cpts: control points (vertices of trajectory convex hull)
   */
  void loadVertices(std::vector<Eigen::Vector3d> &pts, Eigen::MatrixXd &cpts);

  /**
   * @brief get the number of agents
   * @return
   */
  int getNumAgents() { return num_robots_; }

  typedef std::shared_ptr<MADER> Ptr;

 protected:
  ros::NodeHandle nh_;
  ros::Subscriber swarm_sub_;
  /* variables */
  std::vector<SwarmTraj> swarm_trajs_;
  std::map<int, int>     drone_id_to_index_;
  std::map<int, int>     index_to_drone_id_;

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
