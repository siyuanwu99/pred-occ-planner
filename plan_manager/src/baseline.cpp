/**
 * @file baseline.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-10-24
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <plan_manager/baseline.h>
#include <ros/ros.h>

void BaselinePlanner::init() {
  /*** INITIALIZE MAP ***/
  map_.reset(new RiskBase());
  map_->init(nh_map_);

  /*** INITIALIZE A STAR ***/
  a_star_.reset(new RiskHybridAstar());
  a_star_->setParam(nh_planner_);
  a_star_->setEnvironment(map_);
  a_star_->init(Eigen::Vector3d::Zero(), Eigen::Vector3d(10, 10, 2.5));  // TODO: odom_pos_?

  /*** INITIALIZE BEZIER OPT ***/
  traj_optimizer_.reset(new traj_opt::BezierOpt());
  ROS_INFO("Trajectory optimizer initialized.");

  /*** INITIALIZE MADER DECONFLICTION ***/
  collision_avoider_.reset(new ParticleATC(nh_coordinator_));
  collision_avoider_->init();
  map_->setCoordinator(collision_avoider_);

  /*** INITIALIZE VISUALIZATION ***/
  std::string ns = "world";
  visualizer_.reset(new visualizer::Visualizer(nh_map_, ns));

  /*** INITIALIZE SUBSCRIBER ***/
  obstacle_pub_ = nh_map_.advertise<sensor_msgs::PointCloud2>("vis_obstacle", 100);

  ROS_INFO("Baseline planner initialized");
}

bool BaselinePlanner::isTrajSafe(double T) {
  double t0 = ros::Time::now().toSec() - traj_start_time_;
  if (t0 < 0) {
    // ROS_WARN("[Planner] Check trajectory safety before start: dt:%.2f ", -t0);
    t0 = 0;
  }
  if (t0 > T) {
    ROS_WARN("[Planner] Check trajectory after finished .");
    return true;
  }

  double dur = traj_.getDuration();
  T          = T > dur ? dur : T;

  for (double t = t0; t < T; t += 0.1) {
    Eigen::Vector3d pos = traj_.getPos(t);
    double          dt  = t + traj_start_time_ - map_->getMapTime().toSec();
    if (map_->getClearOcccupancy(pos, dt) == 1) {
      ROS_WARN("[Planner] Trajectory not safe at relative time %.2f.", t);
      return false;
    }
  }
  return true;
}

/**
 * @brief show A star results
 *
 */
void BaselinePlanner::showAstarPath() {
  std::vector<Eigen::Vector3d> path = a_star_->getPath(0.1);
  visualizer_->visualizeAstarPath(path);
  visualizer_->visualizeAstarPathXYT(path, 0.1);
}

/**
 * @brief display obstacles
 *
 * @param points buffer of points
 */
void BaselinePlanner::showObstaclePoints(const std::vector<Eigen::Vector3d> &points) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int i = 0; i < static_cast<int>(points.size()); i++) {
    pcl::PointXYZ p;
    p.x = points[i](0);
    p.y = points[i](1);
    p.z = points[i](2);
    cloud.push_back(p);
  }
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp    = ros::Time::now();
  cloud_msg.header.frame_id = "world";
  obstacle_pub_.publish(cloud_msg);
}

/**
 * @brief get empty trajectory
 *
 * If all the Bezier control points are the same, the trajectory is a point.
 * @param pos positions of Bezier control points
 */
void BaselinePlanner::setEmptyTrajectory(const Eigen::Vector3d &pos) {
  traj_ = Bernstein::Bezier(cfg_.corridor_tau);

  Eigen::Matrix<double, 5, 3> cpts;
  cpts.row(0) = pos;
  cpts.row(1) = pos;
  cpts.row(2) = pos;
  cpts.row(3) = pos;
  cpts.row(4) = pos;

  traj_.setControlPoints(cpts);
}

/**
 * @brief generate a axis-aligned cuboid as the initial corridor
 *
 * @param left_higher_corner left, front, higher corner, (x, y, z) maximum
 * @param right_lower_corner right, back, lower corner, (x, y, z) minimum
 * @return H-representation of the cuboid
 */
Eigen::Matrix<double, 6, 4> BaselinePlanner::getInitCorridor(
    const Eigen::Vector3d &left_higher_corner, const Eigen::Vector3d &right_lower_corner) {
  Eigen::Matrix<double, 6, 4> corridor = Eigen::Matrix<double, 6, 4>::Zero();

  corridor(0, 0) = 1.0;
  corridor(1, 1) = 1.0;
  corridor(2, 2) = 1.0;
  corridor(3, 0) = -1.0;
  corridor(4, 1) = -1.0;
  corridor(5, 2) = -1.0;

  corridor.block<3, 1>(0, 3) = -left_higher_corner;
  corridor.block<3, 1>(3, 3) = right_lower_corner;
  return corridor;
}

bool BaselinePlanner::checkGoalReachability(const Eigen::MatrixX4d &corridor,
                                            const Eigen::Vector3d  &start_pos,
                                            Eigen::Vector3d        &goal_pos) {
  Eigen::Vector4d g;
  g << goal_pos, 1.0;
  Eigen::VectorXd rst = corridor * g;
  // std::cout << corridor.rows() << std::endl;
  // std::cout << "rst: " << rst.transpose() << std::endl;
  if (rst.size() <= 0) {
    return true;
  }
  if (rst.maxCoeff() <= 0) {
    return true;
  } else {
    ROS_INFO("[Planner] Goal not reachable, projecting goal to the corridor.");
    /* put goal to the vertices of corridor */

    Eigen::Matrix<double, 3, 1>  c = -goal_pos + start_pos;
    Eigen::Matrix<double, 3, 1>  x;
    Eigen::Matrix<double, -1, 1> b;
    Eigen::Matrix<double, -1, 3> A;

    int m = corridor.rows();
    A.resize(m, 3);
    b.resize(m);
    A = corridor.leftCols<3>();
    b = -corridor.rightCols<1>();

    double          rst      = sdlp::linprog<3>(c, A, b, x);
    Eigen::Vector3d goal_max = x;

    c                        = goal_pos - start_pos;
    double          rst2     = sdlp::linprog<3>(c, A, b, x);
    Eigen::Vector3d goal_min = x;

    goal_pos = 0.5 * (goal_max + goal_min);

    return false;
  }
}

bool BaselinePlanner::checkCorridorIntersect(const Eigen::MatrixX4d &corridor1,
                                             const Eigen::MatrixX4d &corridor2) {
  Eigen::MatrixX4d intersect(corridor1.rows() + corridor2.rows(), 4);
  intersect << corridor1, corridor2;
  return checkCorridorValidity(intersect);
}

bool BaselinePlanner::checkCorridorValidity(const Eigen::MatrixX4d &corridor) {
  int                          m = corridor.rows();
  Eigen::Matrix<double, 3, 1>  c = Eigen::Matrix<double, 3, 1>::Zero();
  Eigen::Matrix<double, 3, 1>  x;
  Eigen::Matrix<double, -1, 1> b;
  Eigen::Matrix<double, -1, 3> A;
  A.resize(m, 3);
  b.resize(m);
  A = corridor.leftCols<3>();
  b = -corridor.rightCols<1>();

  double rst = sdlp::linprog<3>(c, A, b, x);
  return !std::isinf(rst);
}

void BaselinePlanner::ShrinkCorridor(Eigen::MatrixX4d &corridor) {
  for (int i = 0; i < corridor.rows(); i++) {
    double A = corridor(i, 0);
    double B = corridor(i, 1);
    double C = corridor(i, 2);
    corridor(i, 3) += std::sqrt(A * A + B * B + C * C) * cfg_.shrink_size;
  }
}

void BaselinePlanner::ShrinkCorridor(Eigen::MatrixX4d &corridor, const Eigen::Vector3d &path) {
  for (int i = 0; i < corridor.rows(); i++) {
    double A    = corridor(i, 0);
    double B    = corridor(i, 1);
    double C    = corridor(i, 2);
    double norm = std::sqrt(A * A + B * B + C * C);

    Eigen::Vector3d n(A, B, C);
    Eigen::Vector3d z(0, 0, 1);
    // if (n.dot(path) / norm/ path.norm() > 0.8) continue;  // not shrink front and back
    // if (std::abs(C) / norm> 0.8) continue;  // not shrink top and bottom
    corridor(i, 3) += norm * cfg_.shrink_size;
  }
}

// /**
//  * @brief add agents trajectory to map
//  * get agents trajectory from collision avoider
//  * add agents trajectory to map
//  */
// void BaselinePlanner::addAgentsTrajectoryToMap() {
//   int             n = collision_avoider_->getNumAgents();
//   Eigen::Vector3d robot_size;
//   collision_avoider_->getAgentsSize(robot_size);
//   std::vector<Eigen::Vector3d> obs_points;
//   for (int i = 0; i < n - 1; i++) {
//     obs_points.clear();
//     collision_avoider_->getAgentsTrajectory(obs_points, i, 0.2);
//     if (obs_points.size() > 0) {
//       for (int j = 0; j < obs_points.size(); j++) {
//         std::vector<Eigen::Vector3d> pts;
//         pts.push_back(obs_points[j] - odom_pos_);
//         map_->addObstacles(pts, robot_size, ros::Time::now() + ros::Duration(0.2 * j));
//       }
//     }
//   }
// }

bool BaselinePlanner::replan(double                 t,
                             const Eigen::Vector3d &start_pos,
                             const Eigen::Vector3d &start_vel,
                             const Eigen::Vector3d &start_acc,
                             const Eigen::Vector3d &goal_pos) {
  ROS_INFO("Replanning ... start position (%f, %f, %f), time: %f", start_pos(0), start_pos(1),
           start_pos(2), t);
  traj_start_time_ = t;

  // TODO: check time system
  ros::Time t0, t1;
  t0 = ros::Time::now();

  /*----- Path Searching on DSP Dynamic -----*/
  std::cout << "/*----- Path Searching on DSP Static -----*/" << std::endl;
  a_star_->reset();
  t1                 = ros::Time::now();
  t0                 = map_->getMapTime();
  double t_after_map = (t1 - t0).toSec();
  ROS_INFO("[Astar] start position on map: %f | %i", t_after_map,
           static_cast<int>((t1 - t0).toSec() / cfg_.corridor_tau));
  ASTAR_RET rst = a_star_->search(start_pos, start_vel, start_acc, goal_pos,
                                  Eigen::Vector3d(0, 0, 0), true, true, t_after_map);
  if (rst == 0) {
    t1                 = ros::Time::now();
    t0                 = map_->getMapTime();
    double t_after_map = (t1 - t0).toSec();
    a_star_->reset();
    rst = a_star_->search(start_pos, start_vel, start_acc, goal_pos, Eigen::Vector3d(0, 0, 0),
                          false, true, t_after_map);
  }

  auto t2 = ros::Time::now();
  ROS_INFO("[Astar] cost: %f ms", (t2 - t1).toSec() * 1000);

  /* if no path found, set empty trajectory */
  if (rst == NO_PATH) {
    ROS_INFO("[Astar] No path found!");
    ROS_ERROR("[Astar] No path found!");
    return false;
  }
  showAstarPath();  // visualization

  /*----- Safety Corridor Generation -----*/
  std::cout << "/*----- Safety Corridor Generation -----*/" << std::endl;
  std::vector<Eigen::Matrix<double, 6, 1>> route_vel = a_star_->getPathWithVel(cfg_.corridor_tau);
  // std::vector<Eigen::Vector3d>             route     = a_star_->getPath(cfg_.corridor_tau);
  std::vector<Eigen::Vector3d>  wpts;
  std::vector<Eigen::MatrixX4d> hPolys;

  wpts.resize(route_vel.size()); /* copy route_vel to route */
  if (route_vel.size() < 2) {
    ROS_ERROR("[Astar] too less pieces (%lu) for opt", route_vel.size());
    return false;
  }

  std::cout << "start_position = \t" << start_pos.transpose() << std::endl;
  for (int i = 0; i < wpts.size(); i++) {
    wpts[i] = route_vel[i].head(3); /* copy position to route */
    std::cout << "local_wpts[" << i << "] = " << wpts[i].transpose() << std::endl;
    if (wpts[i].z() < 0) wpts[i].z() = 0.1;
  }

  std::vector<Eigen::Vector3d> pc;
  pc.reserve(2000);

  Eigen::Vector3d lower_corner  = Eigen::Vector3d(-4, -4, -1) + start_pos;
  Eigen::Vector3d higher_corner = Eigen::Vector3d(4, 4, 1) + start_pos;
  if (lower_corner.z() < 0) lower_corner.z() = 0;
  if (higher_corner.z() > 4) higher_corner.z() = 4;
  Eigen::Matrix<double, 6, 4> init_corridor = getInitCorridor(higher_corner, lower_corner);

  t1 = ros::Time::now();
  for (int i = 0; i < wpts.size() - 1; i++) {
    /* Get a local bounding box */
    Eigen::Vector3d llc, lhc; /* local lower corner and higher corner */
    lhc(0) = std::min(std::max(wpts[i](0), wpts[i + 1](0)) + cfg_.init_range, higher_corner(0));
    lhc(1) = std::min(std::max(wpts[i](1), wpts[i + 1](1)) + cfg_.init_range, higher_corner(1));
    lhc(2) = std::min(std::max(wpts[i](2), wpts[i + 1](2)) + cfg_.init_range, higher_corner(2));
    llc(0) = std::max(std::min(wpts[i](0), wpts[i + 1](0)) - cfg_.init_range, lower_corner(0));
    llc(1) = std::max(std::min(wpts[i](1), wpts[i + 1](1)) - cfg_.init_range, lower_corner(1));
    llc(2) = std::max(std::min(wpts[i](2), wpts[i + 1](2)) - cfg_.init_range, lower_corner(2));
    Eigen::Matrix<double, 6, 4> bd = init_corridor;
    bd.block<3, 1>(0, 3)           = -lhc;
    bd.block<3, 1>(3, 3)           = llc;

    pc.clear();
    // std::cout << "[dbg] t1_glb = " << traj_start_time_ + i * cfg_.corridor_tau << std::endl;
    double t1_glb = traj_start_time_ + i * cfg_.corridor_tau;
    double t2_glb = traj_start_time_ + (i + 1) * cfg_.corridor_tau;
    map_->getObstaclePoints(pc, t1_glb, t2_glb, llc, lhc);
    std::cout << "pc.size() = " << pc.size() << std::endl;

    Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> m_pc(pc[0].data(), 3,
                                                                         pc.size());

    ros::Time        t3 = ros::Time::now();
    Eigen::MatrixX4d hPoly;
    Eigen::Vector3d  r = Eigen::Vector3d::Ones();
    firi::firi(bd, m_pc, wpts[i], wpts[i + 1], hPoly, r, 2);
    ros::Time t4 = ros::Time::now();
    ShrinkCorridor(hPoly, wpts[i + 1] - wpts[i]);
    if (!checkCorridorValidity(hPoly)) {
      ROS_INFO("[FIRI] %ith corridor takes %f ms, not feasible", i, (t4 - t3).toSec() * 1000);
      break;
    } else {
      ROS_INFO("[FIRI] %ith corridor takes %f ms", i, (t4 - t3).toSec() * 1000);
      hPolys.push_back(hPoly);
    }
  }
  // this->showObstaclePoints(pc);  // visualization

  /* check if adjacent corridors intersect */
  for (int i = 0; i < hPolys.size() - 1; i++) {
    if (!checkCorridorIntersect(hPolys[i], hPolys[i + 1])) {
      ROS_INFO("[Planner] Corridor %i and %i not intersect!", i, i + 1);
      ROS_ERROR("[Planner] Corridor %i and %i not intersect!", i, i + 1);
      if (i < 2) {
        return false;
      } else {
        hPolys.erase(hPolys.begin() + i, hPolys.end());
        break;
      }
    }
  }

  t2 = ros::Time::now();
  ROS_INFO("[CrdGen] Gen %lu corridors cost: %f ms", hPolys.size(), (t2 - t1).toSec() * 1000);
  visualizer_->visualizePolytope(hPolys);

  if (hPolys.size() <= 1) {
    ROS_ERROR("[Planner] Cannot find safety corridors!");
    return false;
  }
  /* Goal position and time allocation */
  Eigen::Vector3d local_goal_pos = route_vel[hPolys.size() - 1].head(3);
  Eigen::Vector3d local_goal_vel = route_vel[hPolys.size() - 1].tail(3);

  if (!checkGoalReachability(hPolys.back(), start_pos, local_goal_pos)) {
    for (auto it = hPolys.end() - 1; it != hPolys.begin(); it--) {
      if (checkGoalReachability(*it, start_pos, local_goal_pos)) {
        hPolys.erase(it + 1, hPolys.end());
        int idx        = hPolys.size() - 1;
        local_goal_pos = route_vel[idx].head(3);
        local_goal_vel = route_vel[idx].tail(3);
        ROS_INFO("[CrdGen] Goal reachable at corridor (%lu/%lu)", hPolys.size(), route_vel.size());
        ROS_WARN("[CrdGen] Goal reachable at corridor (%lu/%lu)", hPolys.size(), route_vel.size());
        break;
      }
    }
  }

  /*----- Trajectory Optimization -----*/
  std::cout << "/*----- Trajectory Optimization -----*/" << std::endl;
  // std::cout << "route size: " << route.size() << std::endl;

  visualizer_->visualizeStartGoal(start_pos);       // visualization
  visualizer_->visualizeStartGoal(local_goal_pos);  // visualization

  std::vector<double> time_alloc;
  time_alloc.resize(hPolys.size(), cfg_.corridor_tau);
  std::cout << "time_alloc size: " << time_alloc.size() << std::endl;

  traj_optimizer_.reset(new traj_opt::BezierOpt());
  Eigen::Matrix3d init_state, final_state;
  init_state.row(0)  = start_pos;
  init_state.row(1)  = start_vel;
  init_state.row(2)  = start_acc;
  final_state.row(0) = local_goal_pos;
  final_state.row(1) = local_goal_vel;
  final_state.row(2) = Eigen::Vector3d(0, 0, 0);

  t1 = ros::Time::now();
  traj_optimizer_->setup(init_state, final_state, time_alloc, hPolys, cfg_.opt_max_vel,
                         cfg_.opt_max_acc);
  if (!traj_optimizer_->optimize()) {
    t2 = ros::Time::now();
    ROS_INFO("[TrajOpt] cost: %f ms", (t2 - t1).toSec() * 1000);
    ROS_ERROR("Trajectory optimization failed!, trajectory piece %lu", hPolys.size());
    return false;
  }
  t2 = ros::Time::now();
  ROS_INFO("[TrajOpt] cost: %f ms", (t2 - t1).toSec() * 1000);

  Bernstein::Bezier traj;
  traj_optimizer_->getOptBezier(traj);

  /*----- Trajectory Deconfliction -----*/
  t1 = ros::Time::now();
  if (!collision_avoider_->isSafeAfterOpt(traj)) {
    ROS_ERROR("Trajectory collides after optimization!");
    t2 = ros::Time::now();
    ROS_INFO("[Deconflict] cost: %f ms", (t2 - t1).toSec() * 1000);
    return false;
  }
  t2 = ros::Time::now();
  ROS_INFO("[Deconflict] cost: %f ms", (t2 - t1).toSec() * 1000);
  prev_traj_start_time_ = traj_start_time_;
  traj_                 = traj;
  return true;
}
