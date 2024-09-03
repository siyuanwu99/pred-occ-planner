/**
 * This file based on Fast-Planner.
 *
 * Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology,
 * <uav.ust.hk> Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at
 * gmail dot com> for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * Fast-Planner is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Fast-Planner is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
 */

#include <path_searching/risk_hybrid_a_star.h>

#include <sstream>

using namespace std;
using namespace Eigen;

RiskHybridAstar::~RiskHybridAstar() {
  for (int i = 0; i < allocate_num_; i++) {
    delete path_node_pool_[i];
  }
}

void RiskHybridAstar::init(const Eigen::Vector3d& map_center, const Eigen::Vector3d& map_size) {
  /* ---------- map params ---------- */
  this->inv_resolution_ = 1.0 / resolution_;
  inv_time_resolution_  = 1.0 / time_resolution_;
  // tolerance_            = ceil(inv_resolution_);

  map_center_ = map_center;
  map_size_   = map_size;

  cout << "center_: " << map_center_.transpose() << endl;
  cout << "map size: " << map_size_.transpose() << endl;

  /* ---------- pre-allocated node ---------- */
  path_node_pool_.resize(allocate_num_);
  for (int i = 0; i < allocate_num_; i++) {
    path_node_pool_[i] = new PathNode();
  }

  phi_          = Eigen::MatrixXd::Identity(6, 6);
  use_node_num_ = 0;
  iter_num_     = 0;
}

void RiskHybridAstar::setEnvironment(const RiskBase::Ptr& grid_map) { grid_map_ = grid_map; }

void RiskHybridAstar::setParam(ros::NodeHandle& nh) {
  nh.param("search/max_tau", max_tau_, -1.0); /* 每次前向积分的时间，设为地图的时间长度 */
  nh.param("search/init_max_tau", init_max_tau_, -1.0); /* 按照之前的输入前向积分的时间 */
  nh.param("search/max_vel", max_vel_, -1.0);
  nh.param("search/max_acc", max_acc_, -1.0);
  nh.param("search/w_time", w_time_, -1.0);
  nh.param("search/horizon", horizon_, -1.0);
  nh.param("search/resolution_astar", resolution_, -1.0);
  nh.param("search/time_resolution", time_resolution_, -1.0);
  nh.param("search/lambda_heu", lambda_heu_, -1.0);
  nh.param("search/allocate_num", allocate_num_, -1);
  nh.param("search/check_num", check_num_, -1);
  nh.param("search/optimistic", optimistic_, true);
  nh.param("search/tolerance", tolerance_, 1);    /* 终点误差容忍度 */
  nh.param("search/is_test", is_testing_, false); /* if testing mode, record traversed voxels */
  tie_breaker_ = 1.0 + 1.0 / 10000;

  double vel_margin;
  nh.param("search/vel_margin", vel_margin, 0.0);
  max_vel_ += vel_margin;
}

void RiskHybridAstar::reset() {
  expanded_nodes_.clear();
  node_path_.clear();

  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty;
  open_set_.swap(empty);

  for (int i = 0; i < use_node_num_; i++) {
    PathNodePtr node = path_node_pool_[i];
    node->setParent(NULL);
    node->setNodeState(NOT_EXPAND);
  }

  use_node_num_ = 0;
  iter_num_     = 0;
  is_shot_succ_ = false;
  has_path_     = false;
}

/**
 * @brief
 * @param start_pt : start position
 * @param start_v  : start velocity
 * @param start_a  : start acceleration
 * @param end_pt  : end position
 * @param end_v  : end velocity
 * @param init : true if this is the first time to search path
 * @param dynamic : true if the trajectory time is considered
 * @param time_start : start time of the trajectory
 * @return
 */
ASTAR_RET RiskHybridAstar::search(Eigen::Vector3d start_pt,
                                  Eigen::Vector3d start_v,
                                  Eigen::Vector3d start_a,
                                  Eigen::Vector3d end_pt,
                                  Eigen::Vector3d end_v,
                                  bool            init,
                                  bool            dynamic,
                                  double          time_start) {
  occupied_voxels_.clear();
  visited_voxels_.clear();
  Eigen::Vector3f map_center_f;
  map_center_ = grid_map_->getMapCenter().cast<double>();
  start_vel_  = start_v;
  start_acc_  = start_a;

  PathNodePtr cur_node = path_node_pool_[0];
  cur_node->setParent(NULL);
  cur_node->state.head(3) = start_pt;
  cur_node->state.tail(3) = start_v;
  cur_node->setIndex(posToIndex(start_pt));
  cur_node->setGScore(0.0);

  Eigen::VectorXd end_state(6);
  Eigen::Vector3i end_index;
  double          time_to_goal;

  end_state.head(3) = end_pt;
  end_state.tail(3) = end_v;
  end_index         = posToIndex(end_pt);
  /* Estimate heuristic from start node to the end */
  cur_node->setFScore(lambda_heu_ * estimateHeuristic(cur_node->state, end_state, time_to_goal));

  /* push first node to the open set */
  cur_node->setNodeState(IN_OPEN_SET);
  this->open_set_.push(cur_node);

  use_node_num_ += 1;

  if (dynamic) {
    time_origin_       = time_start;
    cur_node->time     = time_start;
    cur_node->time_idx = timeToIndex(time_start);
    expanded_nodes_.insert(cur_node->getIndex(), cur_node->time_idx, cur_node);
    cout << "time start: " << time_start << endl;
  } else {
    // cur_node->time = 0.0;  // useless
    // time_origin_   = 0.0;
    expanded_nodes_.insert(cur_node->getIndex(), cur_node);
  }

  PathNodePtr neighbor       = NULL;
  PathNodePtr terminate_node = NULL;
  bool        init_search    = init;

  while (!open_set_.empty()) {
    cur_node = this->open_set_.top();

    std::cout << "cur_node->state: " << cur_node->state.transpose() << std::endl;
    bool reach_horizon = (cur_node->state.head(3) - start_pt).norm() >= horizon_;
    bool near_end      = abs(cur_node->getIndex(0) - end_index(0)) <= tolerance_ &&
                    abs(cur_node->getIndex(1) - end_index(1)) <= tolerance_ &&
                    abs(cur_node->getIndex(2) - end_index(2)) <= tolerance_;
    bool exceed_time = cur_node->time >= max_tau_;

    /* If ReachGoal(n_c) or AnalyticExpand(n_c_) */
    if (reach_horizon || near_end || exceed_time) {
      terminate_node = cur_node;
      retrievePath(terminate_node);
      std::cout << "retrievePath success " << std::endl;
      if (near_end) {
        // Check whether shot traj exist
        estimateHeuristic(cur_node->state, end_state, time_to_goal);
        computeShotTraj(cur_node->state, end_state, time_to_goal);
        if (init_search) {
          ROS_ERROR("Shot in first search loop!");
        }
      }
    }

    if (reach_horizon) {
      if (is_shot_succ_) {
        std::cout << "reach end" << std::endl;
        return ASTAR_RET::REACH_END;
      } else {
        std::cout << "reach horizon" << std::endl;
        return ASTAR_RET::REACH_HORIZON;
      }
    }

    if (near_end) {
      if (is_shot_succ_) {
        std::cout << "reach end" << std::endl;
        return ASTAR_RET::REACH_END;
      } else if (cur_node->getParent() != NULL) {
        std::cout << "near end" << std::endl;
        return ASTAR_RET::NEAR_END;
      } else {
        std::cout << "no path" << std::endl;
        return ASTAR_RET::NO_PATH;
      }
    }

    if (exceed_time) {
      std::cout << "exceed time" << std::endl;
      return ASTAR_RET::REACH_HORIZON;
    }

    open_set_.pop();
    cur_node->setNodeState(IN_CLOSE_SET);
    iter_num_ += 1;

    /* primitives <- Expand(n_c) */
    double res = 1 / 2.0;
    // double time_res = 1 / 1.0, time_res_init = 1 / 20.0;

    Eigen::Matrix<double, 6, 1>  cur_state = cur_node->state;
    Eigen::Matrix<double, 6, 1>  pro_state;
    std::vector<PathNodePtr>     tmp_expand_nodes;
    Eigen::Vector3d              um;
    double                       pro_t;
    std::vector<Eigen::Vector3d> inputs;
    std::vector<double>          durations;
    if (init_search) { /* init: use start acceleration */
      inputs.push_back(start_acc_);
      // for (double tau = time_res_init * init_max_tau_; tau <= init_max_tau_ + 1e-3;
      //      tau += time_res_init * init_max_tau_)
      //   durations.push_back(tau);
      durations.push_back(time_resolution_);
      init_search = false;
    } else { /* otherwise: sample acceleration to generate motion primitives */
      for (double ax = -max_acc_; ax <= max_acc_ + 1e-3; ax += max_acc_ * res)
        for (double ay = -max_acc_; ay <= max_acc_ + 1e-3; ay += max_acc_ * res)
          for (double az = -0.5 * max_acc_; az <= 0.5 * max_acc_ + 1e-3; az += max_acc_ * res) {
            um << ax, ay, az;
            inputs.push_back(um);
          }
      durations.push_back(time_resolution_);
      // for (double tau = time_res * max_tau_; tau <= max_tau_; tau += time_res * max_tau_)
      //   durations.push_back(tau);
    }

    // cout << "cur state:" << cur_state.head(3).transpose() << endl;
    /* traverse all accelerations and trajectory durations */
    for (unsigned int i = 0; i < inputs.size(); ++i)
      for (unsigned int j = 0; j < durations.size(); ++j) {
        // get the state after applying the primitive
        um         = inputs[i];
        double tau = durations[j];
        stateTransit(cur_state, pro_state, um, tau);
        pro_t = cur_node->time + tau;
        // std::cout << "pro_state: " << pro_state.transpose() << std::endl;
        Eigen::Vector3d pro_pos = pro_state.head(3);
        // Check if in close set
        Eigen::Vector3i pro_id   = posToIndex(pro_pos);
        int             pro_t_id = timeToIndex(pro_t);
        PathNodePtr     pro_node =
            dynamic ? expanded_nodes_.find(pro_id, pro_t_id) : expanded_nodes_.find(pro_id);
        if (pro_node != NULL && pro_node->getNodeState() == IN_CLOSE_SET) {
          if (init_search) std::cout << "close" << std::endl;
          continue;
        }

        // Check maximal velocity
        Eigen::Vector3d pro_v = pro_state.tail(3);
        if (fabs(pro_v(0)) > max_vel_ || fabs(pro_v(1)) > max_vel_ || fabs(pro_v(2)) > max_vel_) {
          if (init_search) std::cout << "vel" << std::endl;
          continue;
        }

        // Check not in the same voxel
        Eigen::Vector3i diff      = pro_id - cur_node->getIndex();
        int             diff_time = pro_t_id - cur_node->time_idx;
        if (diff.norm() == 0 && ((!dynamic) || diff_time == 0)) {
          if (init_search) std::cout << "same" << std::endl;
          continue;
        }

        // Check safety
        Eigen::Vector3d             pos;
        Eigen::Matrix<double, 6, 1> xt;
        bool                        is_occ = false;
        for (int k = 1; k <= check_num_; ++k) {
          double dt = tau * double(k) / double(check_num_);
          stateTransit(cur_state, xt, um, dt);
          pos               = xt.head(3);
          double          t = cur_node->time + dt;
          Eigen::Vector4d obs;
          obs << pos, t;
          // check if in the occupied voxel
          // auto occ_it = std::find_if(
          //     occupied_voxels_.begin(), occupied_voxels_.end(),
          //     [&obs](const Eigen::Vector4d& p) { return std::abs(p.norm() - obs.norm()) < 1e-3;
          //     });
          // if (occ_it != occupied_voxels_.end()) {
          //   is_occ = true;
          //   break;
          // }
          //
          // // check if in the visited voxel
          // auto visited_it = std::find_if(
          //     visited_voxels_.begin(), visited_voxels_.end(),
          //     [&obs](const Eigen::Vector4d& p) { return std::abs(p.norm() - obs.norm()) < 1e-3;
          //     });
          // if (visited_it != visited_voxels_.end()) {
          //   is_occ = false;
          //   continue;
          // }

          // not checked before
          if (grid_map_->getClearOcccupancy(pos, t) != 0) {
            is_occ = true;
            occupied_voxels_.push_back(obs);
            break;
          } else {
            visited_voxels_.push_back(obs);
          }
        }

        if (is_occ) {
          if (init_search) std::cout << "safe" << std::endl;
          continue;
        }

        double time_to_goal, tmp_g_score, tmp_f_score;
        tmp_g_score = (um.squaredNorm() + w_time_) * tau + cur_node->getGScore();
        tmp_f_score =
            tmp_g_score + lambda_heu_ * estimateHeuristic(pro_state, end_state, time_to_goal);

        // Compare nodes expanded from the same parent
        bool prune = false;
        for (int j = 0; j < static_cast<int>(tmp_expand_nodes.size()); ++j) {
          PathNodePtr expand_node = tmp_expand_nodes[j];
          // std::cout << " expand_node: " << expand_node->state.transpose();
          // std::cout << " t: " << pro_t_id << " | " << expand_node->time;
          if ((pro_id - expand_node->getIndex()).norm() == 0 &&
              ((!dynamic) || pro_t_id == expand_node->time_idx)) {
            prune = true;
            if (tmp_f_score < expand_node->getFScore()) {
              expand_node->setFScore(tmp_f_score);
              expand_node->setGScore(tmp_g_score);
              expand_node->state    = pro_state;
              expand_node->input    = um;
              expand_node->duration = tau;
              if (dynamic) expand_node->time = cur_node->time + tau;
            }
            break;
          }
        }
        // std::cout << " | prune: " << prune << std::endl;
        // This node end up in a voxel different from others
        if (!prune) {
          if (pro_node == NULL) {
            /** @details
             * expanded_nodes_对于已经扩展过的节点,不需重复分配内存,直接在哈希表中查找即可.
             */
            pro_node = path_node_pool_[use_node_num_];
            pro_node->setIndex(pro_id);
            pro_node->state = pro_state;
            pro_node->setFScore(tmp_f_score);
            pro_node->setGScore(tmp_g_score);
            pro_node->input    = um;
            pro_node->duration = tau;
            pro_node->setParent(cur_node);
            pro_node->setNodeState(IN_OPEN_SET);
            if (dynamic) {
              pro_node->time     = cur_node->time + tau;
              pro_node->time_idx = timeToIndex(pro_node->time);
            }
            // std::cout << "pro_node->state: " << pro_node->state.transpose() << std::endl;
            open_set_.push(pro_node);

            if (dynamic)
              expanded_nodes_.insert(pro_id, pro_node->time, pro_node);
            else
              expanded_nodes_.insert(pro_id, pro_node);
            /**
             * @details 在hash table中的节点只能插入和查找,无法遍历,所以这里定义一个 vector
             * 来存储所有扩展过的节点,方便剪枝的时候遍历所有节点
             */
            tmp_expand_nodes.push_back(pro_node);

            use_node_num_ += 1;
            if (use_node_num_ == allocate_num_) {
              cout << "run out of memory." << endl;
              return ASTAR_RET::NO_PATH;
            }
          } else if (pro_node->getNodeState() == IN_OPEN_SET) {
            if (tmp_g_score < pro_node->getGScore()) {
              // pro_node->index = pro_id;
              pro_node->state = pro_state;
              pro_node->setFScore(tmp_f_score);
              pro_node->setGScore(tmp_g_score);
              pro_node->input    = um;
              pro_node->duration = tau;
              pro_node->setParent(cur_node);

              if (dynamic) pro_node->time = cur_node->time + tau;
            }
          } else {
            cout << "error type in searching: " << pro_node->getNodeState() << endl;
            return ASTAR_RET::SEARCH_ERR;
          }
        }
      }
    // init_search = false;
  }

  cout << "open set empty, no path!" << endl;
  cout << "use node num: " << use_node_num_ << endl;
  cout << "iter num: " << iter_num_ << endl;
  return ASTAR_RET::NO_PATH;
}

double RiskHybridAstar::estimateHeuristic(Eigen::VectorXd x1,
                                          Eigen::VectorXd x2,
                                          double&         optimal_time) {
  const Vector3d dp = x2.head(3) - x1.head(3);
  const Vector3d v0 = x1.segment(3, 3);
  const Vector3d v1 = x2.segment(3, 3);

  double c1 = -36 * dp.dot(dp);
  double c2 = 24 * (v0 + v1).dot(dp);
  double c3 = -4 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));
  double c4 = 0;
  double c5 = w_time_;

  std::vector<double> ts = quartic(c5, c4, c3, c2, c1);

  double v_max = max_vel_ * 0.5;
  double t_bar = (x1.head(3) - x2.head(3)).lpNorm<Infinity>() / v_max;
  ts.push_back(t_bar);

  double cost = 100000000;
  double t_d  = t_bar;

  for (auto t : ts) {
    if (t < t_bar) continue;
    double c = -c1 / (3 * t * t * t) - c2 / (2 * t * t) - c3 / t + w_time_ * t;
    if (c < cost) {
      cost = c;
      t_d  = t;
    }
  }

  optimal_time = t_d;

  return 1.0 * (1 + tie_breaker_) * cost;
}

bool RiskHybridAstar::computeShotTraj(Eigen::VectorXd state1,
                                      Eigen::VectorXd state2,
                                      double          time_to_goal) {
  /* ---------- get coefficient ---------- */
  const Vector3d p0  = state1.head(3);
  const Vector3d dp  = state2.head(3) - p0;
  const Vector3d v0  = state1.segment(3, 3);
  const Vector3d v1  = state2.segment(3, 3);
  const Vector3d dv  = v1 - v0;
  double         t_d = time_to_goal;
  MatrixXd       coef(3, 4);
  end_vel_ = v1;

  Vector3d a = 1.0 / 6.0 * (-12.0 / (t_d * t_d * t_d) * (dp - v0 * t_d) + 6 / (t_d * t_d) * dv);
  Vector3d b = 0.5 * (6.0 / (t_d * t_d) * (dp - v0 * t_d) - 2 / t_d * dv);
  Vector3d c = v0;
  Vector3d d = p0;

  // 1/6 * alpha * t^3 + 1/2 * beta * t^2 + v0
  // a*t^3 + b*t^2 + v0*t + p0
  coef.col(3) = a, coef.col(2) = b, coef.col(1) = c, coef.col(0) = d;

  Vector3d coord, vel, acc;
  VectorXd poly1d, t, polyv, polya;
  Vector3i index;

  Eigen::MatrixXd Tm(4, 4);
  Tm << 0, 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 3, 0, 0, 0, 0;

  /* ---------- forward checking of trajectory ---------- */
  double t_delta = t_d / 10;

  for (double time = t_delta; time <= t_d; time += t_delta) {
    t = VectorXd::Zero(4);
    for (int j = 0; j < 4; j++) {
      t(j) = pow(time, j);
    }

    for (int dim = 0; dim < 3; dim++) {
      poly1d     = coef.row(dim);
      coord(dim) = poly1d.dot(t);
      vel(dim)   = (Tm * poly1d).dot(t);
      acc(dim)   = (Tm * Tm * poly1d).dot(t);

      if (fabs(vel(dim)) > max_vel_ || fabs(acc(dim)) > max_acc_) {
        // cout << "vel:" << vel(dim) << ", acc:" << acc(dim) << endl;
        // return false;
      }
    }
    std::cout << "coord: " << coord.transpose() << std::endl;
    if (grid_map_->getClearOcccupancy(coord) != 0) {
      // if (grid_map_->getInflateOccupancy(coord) != 0) {
      return false;
    }
  }
  coef_shot_    = coef;
  t_shot_       = t_d;
  is_shot_succ_ = true;
  return true;
}

std::vector<double> RiskHybridAstar::cubic(double a, double b, double c, double d) {
  std::vector<double> dts;

  double a2 = b / a;
  double a1 = c / a;
  double a0 = d / a;

  double Q = (3 * a1 - a2 * a2) / 9;
  double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
  double D = Q * Q * Q + R * R;
  if (D > 0) {
    double S = std::cbrt(R + sqrt(D));
    double T = std::cbrt(R - sqrt(D));
    dts.push_back(-a2 / 3 + (S + T));
    return dts;
  } else if (D == 0) {
    double S = std::cbrt(R);
    dts.push_back(-a2 / 3 + S + S);
    dts.push_back(-a2 / 3 - S);
    return dts;
  } else {
    double theta = acos(R / sqrt(-Q * Q * Q));
    dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
    return dts;
  }
}

std::vector<double> RiskHybridAstar::quartic(double a, double b, double c, double d, double e) {
  std::vector<double> dts;

  double a3 = b / a;
  double a2 = c / a;
  double a1 = d / a;
  double a0 = e / a;

  std::vector<double> ys = cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
  double              y1 = ys.front();
  double              r  = a3 * a3 / 4 - a2 + y1;
  if (r < 0) return dts;

  double R = sqrt(r);
  double D, E;
  if (R != 0) {
    D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 + 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
    E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 - 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
  } else {
    D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
    E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
  }

  if (!std::isnan(D)) {
    dts.push_back(-a3 / 4 + R / 2 + D / 2);
    dts.push_back(-a3 / 4 + R / 2 - D / 2);
  }
  if (!std::isnan(E)) {
    dts.push_back(-a3 / 4 - R / 2 + E / 2);
    dts.push_back(-a3 / 4 - R / 2 - E / 2);
  }

  return dts;
}

/**
 * @brief
 * @param delta_t : sample time
 * @return
 */
std::vector<Eigen::Vector3d> RiskHybridAstar::getPath(double delta_t) {
  std::vector<Eigen::Vector3d> state_list;

  /* ---------- get traj of searching ---------- */
  PathNodePtr                 node = node_path_.back();
  Eigen::Matrix<double, 6, 1> x0, xt;

  // double t_counter = 0;
  double t_node   = 0;        // time to next node
  double t_sample = delta_t;  // time to next sample
  state_list.push_back(node->state.head(3));
  while (node->getParent() != NULL) {
    Eigen::Vector3d ut       = node->input;
    double          duration = node->duration;
    x0                       = node->getParent()->state;
    t_node                   = duration;
    while (true) {
      if (t_sample > t_node) {  // can't sample in this node
        node = node->getParent();
        t_sample -= t_node;
        break;
      }
      t_node -= t_sample;
      stateTransit(x0, xt, ut, t_node);  // backward
      state_list.push_back(xt.head(3));
      t_sample = delta_t;  // update time to next sample
    }
  }
  // while (node->getParent() != NULL) {  // sample from the terminal node
  //   Eigen::Vector3d ut       = node->input;
  //   double          duration = node->duration;
  //   x0                       = node->getParent()->state;
  //   for (double t = duration; t >= 1e-5; t -= delta_t) {
  //     stateTransit(x0, xt, ut, t);
  //     state_list.push_back(xt.head(3));
  //     t_sample = delta_t;
  //     t_counter += delta_t;
  //     std::cout << "t: " << t_counter << " x: " << xt.transpose() << std::endl;
  //   }
  //   node = node->getParent();
  // }
  // state_list.push_back(node->state.head(3));
  reverse(state_list.begin(), state_list.end());
  /* ---------- get traj of one shot ---------- */
  if (is_shot_succ_) {
    Eigen::Vector3d coord;
    Eigen::VectorXd poly1d, time(4);

    for (double t = delta_t; t <= t_shot_; t += delta_t) {
      for (int j = 0; j < 4; j++) {
        time(j) = pow(t, j);
      }

      for (int dim = 0; dim < 3; dim++) {
        poly1d     = coef_shot_.row(dim);
        coord(dim) = poly1d.dot(time);
      }
      state_list.push_back(coord);
    }
  }

  return state_list;
}

/**
 * @brief
 * @param delta_t : sample time
 * @return
 */
std::vector<Eigen::Matrix<double, 6, 1>> RiskHybridAstar::getPathWithVel(double delta_t) {
  /* ---------- get traj of searching ---------- */
  PathNodePtr                 node = node_path_.back();  // TODO 0??
  Eigen::Matrix<double, 6, 1> x0, xt;

  // double t_counter = 0;
  double t_node   = 0;        // time to next node
  double t_sample = delta_t;  // time to next sample

  std::vector<Eigen::Matrix<double, 6, 1>> state_list;
  state_list.reserve(10);
  state_list.push_back(node->state);
  while (node->getParent() != NULL) {
    Eigen::Vector3d ut       = node->input;
    double          duration = node->duration;
    x0                       = node->getParent()->state;
    t_node                   = duration;
    while (true) {
      if (t_sample > t_node) {  // can't sample in this node, skip to next
        node = node->getParent();
        t_sample -= t_node;
        break;
      }
      t_node -= t_sample;
      stateTransit(x0, xt, ut, t_node);  // backward
      state_list.push_back(xt);
      t_sample = delta_t;  // update time to next sample
    }
  }
  reverse(state_list.begin(), state_list.end());
  return state_list;
}

void RiskHybridAstar::getSamples(double&                  ts,
                                 vector<Eigen::Vector3d>& point_set,
                                 vector<Eigen::Vector3d>& start_end_derivatives) {
  /* ---------- path duration ---------- */
  double T_sum = 0.0;
  if (is_shot_succ_) T_sum += t_shot_;
  PathNodePtr node = node_path_.back();
  while (node->getParent() != NULL) {
    T_sum += node->duration;
    node = node->getParent();
  }
  // cout << "duration:" << T_sum << endl;

  // Calculate boundary vel and acc
  Eigen::Vector3d end_vel, end_acc;
  double          t;
  if (is_shot_succ_) {
    t       = t_shot_;
    end_vel = end_vel_;
    for (int dim = 0; dim < 3; ++dim) {
      Vector4d coe = coef_shot_.row(dim);
      end_acc(dim) = 2 * coe(2) + 6 * coe(3) * t_shot_;
    }
  } else {
    t       = node_path_.back()->duration;
    end_vel = node->state.tail(3);
    end_acc = node_path_.back()->input;
  }

  // Get point samples
  int seg_num           = floor(T_sum / ts);
  seg_num               = max(8, seg_num);
  ts                    = T_sum / double(seg_num);
  bool sample_shot_traj = is_shot_succ_;
  node                  = node_path_.back();

  for (double ti = T_sum; ti > -1e-5; ti -= ts) {
    if (sample_shot_traj) {
      // samples on shot traj
      Vector3d coord;
      Vector4d poly1d, time;

      for (int j = 0; j < 4; j++) time(j) = pow(t, j);

      for (int dim = 0; dim < 3; dim++) {
        poly1d     = coef_shot_.row(dim);
        coord(dim) = poly1d.dot(time);
      }

      point_set.push_back(coord);
      t -= ts;

      /* end of segment */
      if (t < -1e-5) {
        sample_shot_traj = false;
        if (node->getParent() != NULL) t += node->duration;
      }
    } else {
      // samples on searched traj
      Eigen::Matrix<double, 6, 1> x0 = node->getParent()->state;
      Eigen::Matrix<double, 6, 1> xt;
      Vector3d                    ut = node->input;

      stateTransit(x0, xt, ut, t);

      point_set.push_back(xt.head(3));
      t -= ts;

      // cout << "t: " << t << ", t acc: " << T_accumulate << endl;
      if (t < -1e-5 && node->getParent()->getParent() != NULL) {
        node = node->getParent();
        t += node->duration;
      }
    }
  }
  reverse(point_set.begin(), point_set.end());

  // calculate start acc
  Eigen::Vector3d start_acc;
  if (node_path_.back()->getParent() == NULL) {
    // no searched traj, calculate by shot traj
    start_acc = 2 * coef_shot_.col(2);
  } else {
    // input of searched traj
    start_acc = node->input;
  }

  start_end_derivatives.push_back(start_vel_);
  start_end_derivatives.push_back(end_vel);
  start_end_derivatives.push_back(start_acc);
  start_end_derivatives.push_back(end_acc);
}

std::vector<PathNodePtr> RiskHybridAstar::getVisitedNodes() {
  vector<PathNodePtr> visited;
  visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);
  return visited;
}

Eigen::Vector3i RiskHybridAstar::posToIndex(Eigen::Vector3d pt) {
  Eigen::Vector3i idx = ((pt - map_center_) * inv_resolution_).array().floor().cast<int>();
  return idx;
}

int RiskHybridAstar::timeToIndex(double time) {
  int idx = static_cast<int>(floor((time - time_origin_) * inv_time_resolution_));
  return idx;
}

/**
 * @brief
 * @param state0: current state
 * @param state1: propagated state (next state)
 * @param um    : control input
 * @param tau   : time duration
 */
void RiskHybridAstar::stateTransit(Eigen::Matrix<double, 6, 1>& state0,
                                   Eigen::Matrix<double, 6, 1>& state1,
                                   Eigen::Vector3d              um,
                                   double                       tau) {
  /* phi_ = [ O | I ]*tau */
  for (int i = 0; i < 3; ++i) phi_(i, i + 3) = tau;

  Eigen::Matrix<double, 6, 1> integral;
  integral.head(3) = 0.5 * pow(tau, 2) * um;
  integral.tail(3) = tau * um;

  state1 = phi_ * state0 + integral;
}

void RiskHybridAstar::retrievePath(PathNodePtr end_node) {
  PathNodePtr cur_node = end_node;
  node_path_.push_back(cur_node);

  while (cur_node->getParent() != NULL) {
    cur_node = cur_node->getParent();
    node_path_.push_back(cur_node);
  }

  reverse(node_path_.begin(), node_path_.end());
}
