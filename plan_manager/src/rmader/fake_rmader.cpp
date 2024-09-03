/**
 * @file fake_rmader.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-10-24
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <plan_manager/rmader/fake_rmader.h>
#include <ros/ros.h>

void FakeRmaderPlanner::init() {
  /*** INITIALIZE MAP ***/
  map_.reset(new FakeRiskVoxel());
  map_->init(nh_);

  /*** INITIALIZE A STAR ***/
  a_star_.reset(new FakeRiskHybridAstar());
  a_star_->setParam(nh_);
  a_star_->setEnvironment(map_);
  a_star_->init(odom_pos_, Eigen::Vector3d(10, 10, 4));

  /*** INITIALIZE BEZIER OPT ***/
  traj_optimizer_.reset(new traj_opt::BezierOpt());
  ROS_INFO("Trajectory optimizer initialized.");

  /*** INITIALIZE MADER DECONFLICTION ***/
  collision_avoider_.reset(new RMADER(nh_));
  collision_avoider_->init();
  map_->setCoordinator(collision_avoider_);

  /*** INITIALIZE VISUALIZATION ***/
  std::string ns = "world";
  visualizer_.reset(new visualizer::Visualizer(nh_, ns));

  /*** INITIALIZE SUBSCRIBER ***/
  pose_sub_     = nh_.subscribe("pose", 10, &FakeRmaderPlanner::PoseCallback, this);
  obstacle_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("vis_obstacle", 100);

  /*** INITIALIZE AUXILIARY VARIABLES ***/
  prev_pt_ = ros::Time::now().toSec();
  prev_px_ = 0.0;
  prev_py_ = 0.0;
  prev_pz_ = 0.0;
  prev_vt_ = ros::Time::now().toSec();
  prev_vx_ = 0.0;
  prev_vy_ = 0.0;
  prev_vz_ = 0.0;

  odom_vel_ = Eigen::Vector3d(0, 0, 0);
  odom_acc_ = Eigen::Vector3d(0, 0, 0);

  /*** BOOLEANS ***/
  is_map_updated_       = true;
  is_state_locked_      = false;
  is_velocity_received_ = false;
  ROS_INFO("Baseline planner initialized");
}

/**
 * @brief get current position and attitude from odometry
 * @param msg
 */
void FakeRmaderPlanner::PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  if (!is_state_locked_) {
    is_state_locked_ = true;

    odom_pos_.x() = msg->pose.position.x;
    odom_pos_.y() = msg->pose.position.y;
    odom_pos_.z() = msg->pose.position.z;

    odom_att_.x() = msg->pose.orientation.x;
    odom_att_.y() = msg->pose.orientation.y;
    odom_att_.z() = msg->pose.orientation.z;
    odom_att_.w() = msg->pose.orientation.w;

    is_odom_received_ = true;
  }
  is_state_locked_ = false;

  if (!is_velocity_received_) {
    double dt     = msg->header.stamp.toSec() - prev_pt_;
    odom_vel_.x() = (odom_pos_.x() - prev_px_) / dt;
    odom_vel_.y() = (odom_pos_.y() - prev_py_) / dt;
    odom_vel_.z() = (odom_pos_.z() - prev_pz_) / dt;

    prev_pt_ = msg->header.stamp.toSec();
    prev_px_ = odom_pos_.x();
    prev_py_ = odom_pos_.y();
    prev_pz_ = odom_pos_.z();
  }
}

void FakeRmaderPlanner::clickCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  goal_pos_(0) = msg->pose.position.x;
  goal_pos_(1) = msg->pose.position.y;
  goal_pos_(2) = 1;
  ROS_INFO("Start position: (%f, %f, %f)", odom_pos_(0), odom_pos_(1), odom_pos_(2));
  ROS_INFO("End position: (%f, %f, %f)", goal_pos_(0), goal_pos_(1), goal_pos_(2));
  plan();
}

/**
 * @brief show A star results
 *
 */
void FakeRmaderPlanner::showAstarPath() {
  std::vector<Eigen::Vector3d> path = a_star_->getPath(0.1);
  visualizer_->visualizeAstarPath(path);
  visualizer_->visualizeAstarPathXYT(path, 0.1);
}

void FakeRmaderPlanner::showObstaclePoints(const std::vector<Eigen::Vector3d>& points) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int i = 0; i < points.size(); i++) {
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
 * set current position as the trajectory
 */
void FakeRmaderPlanner::setEmptyTrajectory() {
  traj_ = Bernstein::Bezier(cfg_.corridor_tau);

  Eigen::Matrix<double, 5, 3> cpts;
  cpts.row(0) = odom_pos_;
  cpts.row(1) = odom_pos_;
  cpts.row(2) = odom_pos_;
  cpts.row(3) = odom_pos_;
  cpts.row(4) = odom_pos_;

  traj_.setControlPoints(cpts);
}

/**
 * @brief add agents trajectory to map
 * get agents trajectory from collision avoider
 * add agents trajectory to map
 */
void FakeRmaderPlanner::addAgentsTrajectoryToMap() {
  int             n = collision_avoider_->getNumAgents();
  Eigen::Vector3d robot_size;
  collision_avoider_->getAgentsSize(robot_size);
  std::vector<Eigen::Vector3d> obs_points;
  for (int i = 0; i < n - 1; i++) {
    obs_points.clear();
    collision_avoider_->getAgentsTrajectory(obs_points, i, 0.2);
    if (obs_points.size() > 0) {
      for (int j = 0; j < obs_points.size(); j++) {
        std::vector<Eigen::Vector3d> pts;
        pts.push_back(obs_points[j] - odom_pos_);
        map_->addObstacles(pts, robot_size, ros::Time::now() + ros::Duration(0.2 * j));
      }
    }
  }
}

bool FakeRmaderPlanner::plan() {
  ROS_INFO("Planning...");
  ros::Time t0, t1;
  t0       = ros::Time::now();
  t_start_ = ros::Time::now();  // time when trajectory started (and odom applied)
  // /* ----- Add other agents to the DSP Map ----- */
  // addAgentsTrajectoryToMap();
  // t1 = ros::Time::now();
  // ROS_INFO("Add trajectory to map: Time used: %f ms", (t1 - t0).toSec() * 1000);

  /*----- Path Searching on DSP Dynamic -----*/
  std::cout << "/*----- Path Searching on DSP Static -----*/" << std::endl;
  a_star_->reset();
  t1            = ros::Time::now();
  ASTAR_RET rst = a_star_->search(odom_pos_, odom_vel_, odom_acc_, goal_pos_,
                                  Eigen::Vector3d(0, 0, 0), true, true, 0);
  if (rst == 0) {
    a_star_->reset();
    rst = a_star_->search(odom_pos_, odom_vel_, odom_acc_, goal_pos_, Eigen::Vector3d(0, 0, 0),
                          false, true, 0);
  }

  auto t2 = ros::Time::now();
  ROS_INFO("[Astar] cost: %f ms", (t2 - t1).toSec() * 1000);
  if (rst == NO_PATH) {
    ROS_ERROR("[Astar] No path found!");
    setEmptyTrajectory();
    return false;
  }
  showAstarPath();

  /*----- Safety Corridor Generation -----*/
  std::cout << "/*----- Safety Corridor Generation -----*/" << std::endl;
  std::vector<Eigen::Matrix<double, 6, 1>> route_vel = a_star_->getPathWithVel(cfg_.corridor_tau);
  // std::vector<Eigen::Vector3d>             route     = a_star_->getPath(cfg_.corridor_tau);
  std::vector<Eigen::Vector3d>  route;
  std::vector<Eigen::MatrixX4d> hPolys;

  route.resize(route_vel.size()); /* copy route_vel to route */

  std::cout << "odom_pos = \t" << odom_pos_.transpose() << std::endl;
  for (int i = 0; i < route.size(); i++) {
    route[i] = route_vel[i].head(3); /* copy position to route */
    std::cout << "route[" << i << "] = " << route[i].transpose() << std::endl;
  }

  std::vector<Eigen::Vector3d> pc;
  pc.reserve(2000);

  Eigen::Vector3d lower_corner  = Eigen::Vector3d(-4, -4, -1) + odom_pos_;
  Eigen::Vector3d higher_corner = Eigen::Vector3d(4, 4, 1) + odom_pos_;

  t1 = ros::Time::now();
  for (int i = 0; i < route.size() - 1; i++) {
    /* Get a local bounding box */
    Eigen::Vector3d llc, lhc; /* local lower corner and higher corner */
    lhc(0) = std::min(std::max(route[i](0), route[i + 1](0)) + cfg_.init_range, higher_corner(0));
    lhc(1) = std::min(std::max(route[i](1), route[i + 1](1)) + cfg_.init_range, higher_corner(1));
    lhc(2) = std::min(std::max(route[i](2), route[i + 1](2)) + cfg_.init_range, higher_corner(2));
    llc(0) = std::max(std::min(route[i](0), route[i + 1](0)) - cfg_.init_range, lower_corner(0));
    llc(1) = std::max(std::min(route[i](1), route[i + 1](1)) - cfg_.init_range, lower_corner(1));
    llc(2) = std::max(std::min(route[i](2), route[i + 1](2)) - cfg_.init_range, lower_corner(2));
    pc.clear();
    double t1_glb = t_start_.toSec() + i * cfg_.corridor_tau;
    double t2_glb = t_start_.toSec() + (i + 1) * cfg_.corridor_tau;
    map_->getObstaclePoints(pc, t1_glb, t2_glb, llc, lhc);
    Eigen::Matrix<double, 6, 4> bd = Eigen::Matrix<double, 6, 4>::Zero();  // initial corridor
    bd(0, 0)                       = 1.0;
    bd(1, 1)                       = 1.0;
    bd(2, 2)                       = 1.0;
    bd(3, 0)                       = -1.0;
    bd(4, 1)                       = -1.0;
    bd(5, 2)                       = -1.0;
    bd.block<3, 1>(0, 3)           = -lhc;
    bd.block<3, 1>(3, 3)           = llc;
    Eigen::MatrixX4d hPoly;

    Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> m_pc(pc[0].data(), 3,
                                                                         pc.size());

    ros::Time       t3 = ros::Time::now();
    Eigen::Vector3d r  = Eigen::Vector3d::Ones();
    firi::firi(bd, m_pc, route[i], route[i + 1], hPoly, r, 2);
    ros::Time t4 = ros::Time::now();
    ROS_INFO("[FIRI] %ith corridor takes %f ms", i, (t4 - t3).toSec() * 1000);
    hPolys.push_back(hPoly);
    this->showObstaclePoints(pc);
  }

  t2 = ros::Time::now();
  ROS_INFO("[CrdGen] cost: %f ms", (t2 - t1).toSec() * 1000);
  visualizer_->visualizePolytope(hPolys);

  /*----- Trajectory Optimization -----*/
  std::cout << "/*----- Trajectory Optimization -----*/" << std::endl;
  std::cout << "hPolys size: " << hPolys.size() << std::endl;
  // std::cout << "route size: " << route.size() << std::endl;

  /* Goal position and time allocation */
  Eigen::Vector3d local_goal_pos = route_vel.back().head(3);
  Eigen::Vector3d local_goal_vel = route_vel.back().tail(3);

  std::vector<double> time_alloc;
  time_alloc.resize(hPolys.size(), cfg_.corridor_tau);
  std::cout << "time_alloc size: " << time_alloc.size() << std::endl;
  traj_optimizer_.reset(new traj_opt::BezierOpt());
  Eigen::Matrix3d init_state, final_state;
  init_state.row(0)  = odom_pos_;
  init_state.row(1)  = odom_vel_;
  init_state.row(2)  = odom_acc_;
  final_state.row(0) = local_goal_pos;
  final_state.row(1) = local_goal_vel;
  final_state.row(2) = Eigen::Vector3d(0, 0, 0);

  // std::cout << "init_state: " << init_state << std::endl;
  // std::cout << "final_state: " << final_state << std::endl;
  visualizer_->visualizeStartGoal(odom_pos_);
  visualizer_->visualizeStartGoal(local_goal_pos);

  t1 = ros::Time::now();
  traj_optimizer_->setup(init_state, final_state, time_alloc, hPolys, cfg_.opt_max_vel,
                         cfg_.opt_max_acc);
  if (!traj_optimizer_->optimize()) {
    t2 = ros::Time::now();
    ROS_INFO("[TrajOpt] cost: %f ms", (t2 - t1).toSec() * 1000);
    ROS_ERROR("Trajectory optimization failed!");
    setEmptyTrajectory();
    return false;
  }
  t2 = ros::Time::now();
  ROS_INFO("[TrajOpt] cost: %f ms", (t2 - t1).toSec() * 1000);

  traj_optimizer_->getOptBezier(traj_);

  /*----- Trajectory Deconfliction -----*/
  t1 = ros::Time::now();
  if (!collision_avoider_->isSafeAfterOpt(traj_)) {
    ROS_ERROR("Trajectory collides after optimization!");
    t2 = ros::Time::now();
    ROS_INFO("[MADER] cost: %f ms", (t2 - t1).toSec() * 1000);
    setEmptyTrajectory(); /* set current position as traj to prevent planning failed */
    return false;
  }
  t2 = ros::Time::now();
  ROS_INFO("[MADER] cost: %f ms", (t2 - t1).toSec() * 1000);
  return true;
}

bool FakeRmaderPlanner::delayCheck(const Bernstein::Bezier& traj) {
  bool is_safe = collision_avoider_->isSafeAfterDelayCheck(traj);
  return is_safe;
}
