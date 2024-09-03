/**
 * @file particle.hpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief particle-based air traffic coordinator
 * @version 1.0
 * @date 2023-07-25
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef PARTICLE_HPP_
#define PARTICLE_HPP_

#include <geometry_msgs/PolygonStamped.h>
#include <ros/ros.h>
#include <traj_utils/BezierTraj.h>
#include <Eigen/Eigen>
#include <map>
#include <memory>
#include <queue>
#include <random>
#include <traj_utils/bernstein.hpp>
#include <vector>
#include "separator.hpp"

typedef Bernstein::BernsteinPiece Piece;
typedef Bernstein::Bezier         Traj;

struct SwarmParticleTraj {
  int    id;
  double duration;
  double time_received;  // ros time
  double time_start;     // ros time
  double time_end;       // ros time
  Traj   traj;
};

class ParticleATC {
 protected:
  ros::NodeHandle nh_;
  ros::Subscriber swarm_sub_;
  ros::Subscriber particles_sub_;
  ros::Publisher  ego_particles_pub_;
  ros::Timer      ego_particles_timer_;

  /* variables */
  bool is_planner_initialized_;
  bool have_received_traj_while_checking_;
  bool have_received_traj_while_optimizing_;
  bool is_traj_safe_;
  bool is_checking_;

  int drone_id_;
  int num_robots_;

  double drone_size_x_;
  double drone_size_y_;
  double drone_size_z_;

  // std::map<int, int> drone_id_to_index_;
  // std::map<int, int> index_to_drone_id_;

  separator::Separator *separator_solver_;

  /* data */
  std::vector<SwarmParticleTraj>            swarm_trajs_;
  std::vector<Eigen::Vector3d>              ego_particles_;
  std::vector<std::vector<Eigen::Vector3d>> particles_buffer_;

  /* collision risk */
  int   num_resample_;
  float replan_risk_rate_;

 public:
  /* constructor */
  ParticleATC(ros::NodeHandle &nh) : nh_(nh) {}
  ~ParticleATC() {}

  /* functions */
  void init();
  void reset();
  void initEgoParticles();

  int                                 getEgoID() { return drone_id_; }
  int                                 getEgoParticlesNum() { return ego_particles_.size(); }
  const std::vector<Eigen::Vector3d> &getEgoParticles() const { return ego_particles_; }
  const std::vector<Eigen::Vector3d> &getParticlesBuffer(int idx) const {
    return particles_buffer_[idx];
  }

  /**
   * @brief callback function, receive particles from other agents and save them to the buffer
   *
   * @param particles_msg particles message
   */
  void particlesCallback(const geometry_msgs::PolygonStamped::ConstPtr &particles_msg);

  /**
   * @brief timer callback function, publish ego particles
   */
  void egoParticlesCallback(const ros::TimerEvent &);

  /**
   * @brief trajectory callback function, receive trajectories from other agents and save them to
   * the buffer
   *
   * @param traj_msg bezier trajectory message
   */
  void trajectoryCallback(const traj_utils::BezierTraj::ConstPtr &traj_msg);

  /**
   * @brief check whether the trajectory is safe
   * @param traj: trajectory to be checked
   * @return true if safe, false otherwise
   */
  bool isSafeAfterOpt(const Bernstein::Bezier &traj);

  /**
   * @brief obtains control points of other drones within the prediction horizon
   *
   * @param pts input obstacle points buffer
   * @param t prediction horizon
   */
  void getObstaclePoints(std::vector<Eigen::Vector3d> &pts, double t);

  /**
   * @brief get waypoints of the agent 'idx_agenq' at time 't'
   * @param pts: points buffer
   * @param idx_agent: index of the agent
   * @param t: global time
   * @return false: if trajectory already finished
   */
  bool getWaypoints(std::vector<Eigen::Vector3d> &pts, int idx_agent, double t);

  /**
   * @brief input vertices of trajectory convex hull, get Minkowski sum of the convex
   * hull and ego polytope, and push these vertices into the buffer `pts`
   * @param pts : points buffer
   * @param cpts: control points (vertices of trajectory convex hull)
   */
  void loadParticles(std::vector<Eigen::Vector3d> &pts, const Eigen::MatrixXd &cpts, int idx_agent);
  void loadParticles(std::vector<Eigen::Vector3d> &pts, const Eigen::Vector3d &cpts, int idx_agent);

  /**
   * @brief get the number of agents
   * @return
   */
  int getNumAgents() { return num_robots_; }

  /**
   * @brief get the number of particles
   * @return
   */
  bool getParticlesWithRisk(std::vector<Eigen::Vector3d> &pts,
                            std::vector<float>           &risks,
                            int                           idx_agent,
                            double                        t0);

  typedef std::shared_ptr<ParticleATC> Ptr;
};
#endif  // PARTICLE_HPP_
