/**
 * @file particle.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief particle-based air traffic coordinator
 * @version 1.0
 * @date 2023-07-25
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <traj_coordinator/particle.hpp>

/**
 * @brief initialize the particle-based air traffic coordinator
 */

void ParticleATC::init() {
  separator_solver_ = new separator::Separator();

  swarm_sub_     = nh_.subscribe("/broadcast_traj_to_planner", 1, &ParticleATC::trajectoryCallback,
                                 this, ros::TransportHints().tcpNoDelay());
  particles_sub_ = nh_.subscribe("/broadcast_particles", 1, &ParticleATC::particlesCallback, this);
  ego_particles_pub_   = nh_.advertise<geometry_msgs::PolygonStamped>("/broadcast_particles", 1);
  ego_particles_timer_ = nh_.createTimer(ros::Duration(2), &ParticleATC::egoParticlesCallback,
                                         this);  // 10Hz

  nh_.param("drone_id", drone_id_, 0);
  nh_.param("swarm/num_robots", num_robots_, 3);
  nh_.param("swarm/drone_size_x", drone_size_x_, 0.3);
  nh_.param("swarm/drone_size_y", drone_size_y_, 0.3);
  nh_.param("swarm/drone_size_z", drone_size_z_, 0.3);
  nh_.param("swarm/replan_risk_rate", replan_risk_rate_, 0.0F);
  nh_.param("swarm/num_resample", num_resample_, 10);

  /* Initialize Booleans */
  is_checking_                         = false;
  have_received_traj_while_checking_   = false;
  have_received_traj_while_optimizing_ = false;

  /* Initialize hash table */
  swarm_trajs_.reserve(num_robots_ - 1);

  /* Initialize ego particles sets */
  initEgoParticles();

  ROS_INFO("[CA] Particle ATC initialized");
}

void ParticleATC::reset() {
  separator_solver_ = new separator::Separator();

  is_checking_                         = false;
  have_received_traj_while_checking_   = false;
  have_received_traj_while_optimizing_ = false;

  swarm_trajs_.reserve(num_robots_ - 1);
  ROS_INFO("[CA] Particle ATC reset");
  // TODO: guarantee that the particles buffer is filled
}

void ParticleATC::initEgoParticles() {
  ego_particles_.clear();
  double WIDTH = 0.30;
  double STEP  = 0.15;
  /* body particles, we use a cube to approximate the body */
  for (double x = -drone_size_x_ / 2; x <= drone_size_x_ / 2; x += STEP) {
    for (double y = -drone_size_y_ / 2; y <= drone_size_y_ / 2; y += STEP) {
      for (double z = -drone_size_z_ / 2; z <= drone_size_z_ / 2; z += STEP) {
        Eigen::Vector3d pt(x, y, z);
        ego_particles_.push_back(pt);
      }
    }
  }
  ROS_INFO("[CA] ego particles initialized with %lu particles", ego_particles_.size());

  /* fill the ego particles buffer */
  particles_buffer_.resize(num_robots_);
  for (int i = 0; i < num_robots_; ++i) {
    particles_buffer_[i].clear();
    particles_buffer_[i].reserve(ego_particles_.size());
  }

  for (int i = 0; i < ego_particles_.size(); ++i) {
    particles_buffer_[drone_id_].push_back(ego_particles_[i]);
  }
}

void ParticleATC::particlesCallback(const geometry_msgs::PolygonStamped::ConstPtr &particles_msg) {
  int rbt_idx = std::stoi(particles_msg->header.frame_id.substr(3, 1));
  std::cout << rbt_idx << std::endl;
  if (rbt_idx >= num_robots_) {
    ROS_ERROR("[CA] received particles from drone %d, which is not in the swarm", rbt_idx);
    return;
  }
  if (rbt_idx == drone_id_) return;

  if (particles_buffer_[rbt_idx].size() != particles_msg->polygon.points.size()) {
    /* update the particles buffer */
    particles_buffer_[rbt_idx].clear();
    for (int i = 0; i < particles_msg->polygon.points.size(); ++i) {
      Eigen::Vector3d pt;
      pt << particles_msg->polygon.points[i].x, particles_msg->polygon.points[i].y,
          particles_msg->polygon.points[i].z;
      particles_buffer_[rbt_idx].push_back(pt);
    }
    ROS_INFO("[CA] received %lu particles from drone %d", particles_buffer_[rbt_idx].size(),
             rbt_idx);
  }
}

void ParticleATC::egoParticlesCallback(const ros::TimerEvent &event) {
  geometry_msgs::PolygonStamped particles_msg;
  particles_msg.header.stamp    = ros::Time::now();
  particles_msg.header.frame_id = "uav" + std::to_string(drone_id_) + "/base_link";
  particles_msg.polygon.points.resize(ego_particles_.size());
  for (int i = 0; i < ego_particles_.size(); ++i) {
    particles_msg.polygon.points[i].x = ego_particles_[i](0);
    particles_msg.polygon.points[i].y = ego_particles_[i](1);
    particles_msg.polygon.points[i].z = ego_particles_[i](2);
  }
  ego_particles_pub_.publish(particles_msg);
}

/**
 * @brief trajectory callback function, receive trajectories from other agents and save them to
 * the buffer
 *
 * @param traj_msg bezier trajectory message
 */
void ParticleATC::trajectoryCallback(const traj_utils::BezierTraj::ConstPtr &traj_msg) {
  int id = traj_msg->drone_id;

  /* filter out ego trajectories */
  if (id == drone_id_) {
    return;
  }

  if (is_checking_) {
    have_received_traj_while_checking_ = true;
    ROS_INFO("[CA|A%i] trajectory received during checking", id);
  } else {
    have_received_traj_while_optimizing_ = false;
  }

  /* update swarm_trajs_ */
  int n_seg = traj_msg->duration.size();
  int order = traj_msg->order + 1;

  SwarmParticleTraj traj;
  traj.id            = traj_msg->drone_id;
  traj.time_received = ros::Time::now().toSec();
  ros::Time time_pub = traj_msg->pub_time;
  // ros::Time time_start = traj_msg->start_time;

  ROS_INFO("[CA|A%i] traj communication delay %f ms", traj.id,
           (traj.time_received - time_pub.toSec()) * 1000);

  traj.time_start = traj_msg->start_time.toSec();
  double t_end    = traj.time_start;

  /* load traj duration and control points from msg */
  std::vector<double> durations;
  Eigen::MatrixX3d    cpts;
  cpts.resize(order * n_seg, 3);
  for (int i = 0; i < n_seg; i++) {
    t_end += traj_msg->duration[i];
    durations.push_back(traj_msg->duration[i]);
    /* load control points from msg */
    for (int j = 0; j < order; j++) {
      int k = j + i * order;
      cpts.row(k) << traj_msg->cpts[k].x, traj_msg->cpts[k].y, traj_msg->cpts[k].z;
    }
  }
  traj.time_end = t_end;
  traj.traj     = Traj(durations, cpts);

  /* update swarm_trajs_ */
  std::vector<SwarmParticleTraj>::iterator obs_ptr =
      std::find_if(swarm_trajs_.begin(), swarm_trajs_.end(),
                   [=](const SwarmParticleTraj &tmp) { return tmp.id == traj.id; });

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
 * @brief obtains control points of other drones within the prediction horizon
 *
 * @param pts input obstacle points buffer
 * @param t prediction horizon
 */
void ParticleATC::getObstaclePoints(std::vector<Eigen::Vector3d> &pts, double horizon) {
  double t0 = ros::Time::now().toSec();  // TODO: t0 is not accurate

  double tf = t0 + horizon;
  for (auto &traj : swarm_trajs_) {
    if (tf > traj.time_start || t0 < traj.time_end) {
      Eigen::MatrixXd cpts;
      traj.traj.getCtrlPoints(cpts);
      ROS_INFO("[CA|A%d] pushing %d obstacle points", traj.id, (int)cpts.rows());
      ROS_INFO("t_start: 0 , t_end: %f | traj: (%f, %f)", tf - t0, traj.time_start - t0,
               traj.time_end - t0);
      loadParticles(pts, cpts, traj.id);
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
bool ParticleATC::isSafeAfterOpt(const Traj &traj) {
  is_checking_ = true;
  /* collision check starts */

  Eigen::MatrixXd cpts;
  traj.getCtrlPoints(cpts);
  /* checkin: check waypoints */

  std::vector<Eigen::Vector3d> pointsA;
  std::vector<Eigen::Vector3d> pointsB;

  /* Push ego trajectory convex hull to buffer */
  // loadParticles(pointsA, cpts, drone_id_);
  for (int i = 0; i < cpts.rows(); i++) {
    pointsA.push_back(cpts.row(i));
  }

  /* checkin: check other trajectories */
  // std::cout << "swarm_trajs_.size(): " << swarm_trajs_.size() << std::endl;
  std::cout << "pointsA.size(): " << pointsA.size() << std::endl;
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

      for (int i = 0; i < cpts.rows(); i++) {
        pointsB.push_back(cpts.row(i));
      }

      /* add current control points */
      // std::cout << "particle_buffer_.size(): " << particles_buffer_.size() << std::endl;
      // std::cout << "cpts.rows(): " << cpts.rows() << std::endl;
      // loadParticles(pointsB, cpts, traj.id);
      std::cout << "pointsB.size(): " << pointsB.size() << std::endl;
      // DEBUG: if we use the above function, pointB will be too large
      // Alternative: use quick hull algorithm to get convex hull of pointsB
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
void ParticleATC::loadParticles(std::vector<Eigen::Vector3d> &pts,
                                const Eigen::MatrixXd        &cpts,
                                int                           idx_agent) {
  if (particles_buffer_[idx_agent].empty()) return;
  for (int i = 0; i < cpts.rows(); i++) {
    /* Add trajectory control point */
    Eigen::Vector3d pt = cpts.row(i);
    for (auto &e : particles_buffer_[idx_agent]) pts.push_back(pt + e);
  }
}

void ParticleATC::loadParticles(std::vector<Eigen::Vector3d> &pts,
                                const Eigen::Vector3d        &pt,
                                int                           idx_agent) {
  if (particles_buffer_[idx_agent].empty()) return;
  for (auto &e : particles_buffer_[idx_agent]) pts.push_back(pt + e);
}

/**
 * @brief get waypoints of the agent 'idx_agent' at time 't'
 * @param pts : waypoints buffer to be filled
 * @param idx_agent
 * @param t : absolute timestamp
 * @return false if the trajectory ends
 */
bool ParticleATC::getWaypoints(std::vector<Eigen::Vector3d> &pts, int idx_agent, double t0) {
  /* find the trajectory of the agent */
  std::vector<SwarmParticleTraj>::iterator ptr =
      std::find_if(swarm_trajs_.begin(), swarm_trajs_.end(),
                   [=](const SwarmParticleTraj &tmp) { return tmp.id == idx_agent; });

  if (ptr == swarm_trajs_.end() || idx_agent == drone_id_) { /* if trajectory ends or ego-robot */
    return false;
  }

  if (ptr->time_start < t0 && ptr->time_end > t0) {
    /* Adding trajectory points to the buffer */
    Eigen::Vector3d pt = ptr->traj.getPos(t0 - ptr->time_start);
    loadParticles(pts, pt, idx_agent);
    // ROS_INFO_STREAM("[CA|A" << idx_agent << "] got trajectory at " << t0 - ptr->time_start
    //                         << " | pt: " << pt.transpose());
    return true;
  } else if (ptr->time_start > t0) {
    // ROS_INFO("[CA|A%i] traj not start yet, ts: %f, te: %f", idx_agent, ptr->time_start - t0,
    //          ptr->time_end - t0);
    return true;
  } else if (ptr->time_end < t0) {
    pts.push_back(ptr->traj.getPos(ptr->traj.getDuration())); /* push last point */
    // ROS_INFO("[CA|A%i] traj already ends, ts: %f, te: %f", idx_agent, ptr->time_start - t0,
    //          ptr->time_end - t0);
    return false;
  }
  return false;
}

bool ParticleATC::getParticlesWithRisk(std::vector<Eigen::Vector3d> &pts,
                                       std::vector<float>           &risks,
                                       int                           idx_agent,
                                       double                        t0) {
  /* get waypoints */
  std::vector<Eigen::Vector3d> pts_waypoints;

  /* get waypoints and particles */
  bool is_get_waypoints = this->getWaypoints(pts_waypoints, idx_agent, t0);
  if (!is_get_waypoints || pts_waypoints.empty()) {
    return false;
  }
  std::cout << "pts_waypoints.size(): " << pts_waypoints.size() << std::endl;
  /* get covariance */
  std::vector<SwarmParticleTraj>::iterator ptr =
      std::find_if(swarm_trajs_.begin(), swarm_trajs_.end(),
                   [=](const SwarmParticleTraj &tmp) { return tmp.id == idx_agent; });
  if (ptr->time_start > t0) return true; /* trajectory not start yet */

  float pos_stddev = replan_risk_rate_ * static_cast<float>(t0 - ptr->time_start);
  // ROS_INFO("[dbg] (dt: %.3f)resample variance: %.3f\n", t0 - ptr->time_start, pos_stddev);

  /* resample particles */
  pts.clear();
  risks.clear();

  if (pos_stddev < 1e-3) { /* no resample */
    pts.reserve(pts_waypoints.size());
    risks.reserve(pts_waypoints.size());
    for (auto &e : pts_waypoints) {
      pts.push_back(e);
      risks.push_back(1.0f);
    }
  } else { /* resample particles */
    pts.reserve(pts_waypoints.size() * num_resample_);
    risks.reserve(pts_waypoints.size() * num_resample_);
    std::default_random_engine      generator(time(NULL));
    std::normal_distribution<float> n_x(0.0, pos_stddev);
    std::normal_distribution<float> n_y(0.0, pos_stddev);
    std::normal_distribution<float> n_z(0.0, pos_stddev);

    for (auto &e : pts_waypoints) {
      std::vector<float> risk_buf;
      risk_buf.reserve(num_resample_);
      for (int i = 0; i < num_resample_; i++) {
        float nx = n_x(generator);
        float ny = n_y(generator);
        float nz = n_z(generator);

        Eigen::Vector3d pt;
        pt = e + Eigen::Vector3f(nx, ny, nz).cast<double>();

        // weights follows the gaussian model
        float risk = std::exp(-0.5f * (nx * nx + ny * ny + nz * nz) / (pos_stddev * pos_stddev));
        pts.push_back(pt);
        risk_buf.push_back(risk);
        // std::cout << "resample: " << nx << ", " << ny << ", " << nz << "\trisk:" << risk
        //           << std::endl;
      }
      float sum = std::accumulate(risk_buf.begin(), risk_buf.end(), 0.0f);
      for (auto &r : risk_buf) r = r * num_resample_ / sum;
      risks.insert(risks.end(), risk_buf.begin(), risk_buf.end());
    }

    /* sum of weights and normalize */
    // float sum = std::accumulate(risks.begin(), risks.end(), 0.0f);
    // float N   = static_cast<float>(pts_waypoints.size());
    // for (auto &e : risks) e = e * N / sum;
  }

  std::cout << "[CA|A" << idx_agent << "] particles: " << pts.size() << std::endl;
  // std::cout << "[CA|A" << idx_agent
  //           << "] risks 10: " << std::accumulate(risks.begin(), risks.begin() + 10, 0.0f)
  //           << std::endl;

  return true;
}
