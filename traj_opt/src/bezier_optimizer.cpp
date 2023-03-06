/**
 * @file bezier_optimizer.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-09-08
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "bernstein/bezier_optimizer.hpp"
using namespace Bernstein;

namespace traj_opt {

void BezierOpt::setConstraints(const std::vector<PolyhedronH>& constraints) {
  constraints_ = constraints;
  assert(static_cast<int>(constraints_.size()) == M_);
}

void BezierOpt::setTimeAllocation(const std::vector<double>& time_allocation) {
  t_ = time_allocation;
  M_ = t_.size();
}

void BezierOpt::setup(const Eigen::Matrix3d&          start,
                      const Eigen::Matrix3d&          end,
                      const std::vector<double>&      time_allocation,
                      const std::vector<PolyhedronH>& constraints,
                      const double&                   max_vel,
                      const double&                   max_acc) {
  max_vel_ = max_vel;
  max_acc_ = max_acc;
  setTimeAllocation(time_allocation);
  setConstraints(constraints);
  init_ = start;
  goal_ = end;

  // x_.resize(DIM * (4 * M_ + 1), 1);
  DM_ = DIM * M_ * (N_ + 1);
  // Q_.resize(DM_, DM_);
  // Q_.setZero();
  // A_.resize(10, DM_);
  // A_.setZero();
  // q_.resize(DM_);
  // q_.setZero();
  // b_.resize(10);
  // b_.setZero();
  x_.resize(DM_);
  x_.setZero();
  calcMinJerkCost();
  addConstraints();
}

void BezierOpt::reset() {
  idx_ = 0;
  M_   = 0;
  t_.clear();
  constraints_.clear();
  bc_->clear();
}

void BezierOpt::calcCtrlPtsCvtMat() {
  p2v_.resize(DIM * N_, DIM * (N_ + 1));
  v2a_.resize(DIM * (N_ - 1), DIM * N_);
  a2j_.resize(DIM * (N_ - 2), DIM * (N_ - 1));
  p2v_.setZero();
  v2a_.setZero();
  a2j_.setZero();
  for (int i = 0; i < N_; i++) {
    p2v_.block(i * DIM, i * DIM, DIM, DIM)       = -N_ * Eigen::MatrixXd::Identity(DIM, DIM);
    p2v_.block(i * DIM, (i + 1) * DIM, DIM, DIM) = N_ * Eigen::MatrixXd::Identity(DIM, DIM);
  }
  for (int i = 0; i < N_ - 1; i++) {
    v2a_.block(i * DIM, i * DIM, DIM, DIM)       = -(N_ - 1) * Eigen::MatrixXd::Identity(DIM, DIM);
    v2a_.block(i * DIM, (i + 1) * DIM, DIM, DIM) = (N_ - 1) * Eigen::MatrixXd::Identity(DIM, DIM);
  }
  for (int i = 0; i < N_ - 2; i++) {
    a2j_.block(i * DIM, i * DIM, DIM, DIM)       = -(N_ - 2) * Eigen::MatrixXd::Identity(DIM, DIM);
    a2j_.block(i * DIM, (i + 1) * DIM, DIM, DIM) = (N_ - 2) * Eigen::MatrixXd::Identity(DIM, DIM);
  }
}

/**
 * @brief cost = x'Qx = x'P'QPx
 * x are control points
 * P is the control points to jerk conversion matrix
 *
 */
void BezierOpt::calcMinJerkCost() {
  Q_.resize(DM_, DM_);
  Q_.setZero();
  q_.resize(DM_);
  q_.setZero();
  Eigen::Matrix<double, DIM, DIM> I = Eigen::MatrixXd::Identity(DIM, DIM);

  Eigen::MatrixXd p2j = a2j_ * v2a_ * p2v_;
  Eigen::MatrixXd P(DIM * (N_ - 2), DIM * (N_ - 2));
  P << I / 3, I / 6, I / 6, I / 3;
  Eigen::MatrixXd QM = p2j.transpose() * P * p2j;
  for (int i = 0; i < M_; i++) {
    Q_.block(i * DIM * (N_ + 1), i * DIM * (N_ + 1), DIM * (N_ + 1), DIM * (N_ + 1)) = QM;
  }
}

void BezierOpt::addConstraints() {
  int num_const = 0;  // number of constraints
  for (auto& c : constraints_) {
    num_const += c.rows();
  }
  num_const *= N_ + 1;                                    // all points inside the polyhedron
  int num_continuous = (1 + M_) * DIM * 3;                // continuous between segments
  int num_dynamical  = M_ * (DIM * N_ + DIM * (N_ - 1));  // maximum velocity and acceleration
  int num            = num_const + num_continuous + num_dynamical;
  std::cout << "num: " << num_continuous << " | " << num_const << " | " << num_dynamical << " || "
            << num << std::endl;
  A_.resize(num, DM_);
  A_.setZero();
  b_.resize(num);
  b_.setZero();
  lb_.resize(num);
  lb_.setZero();
  idx_ = 0;
  addContinuityConstraints();
  addDynamicalConstraints();
  addSafetyConstraints();
}

/**
 * @brief equality constraints for continuity between segments
 *
 */
void BezierOpt::addContinuityConstraints() {
  double t0 = t_[0], tM = t_[M_ - 1];
  /* position continuity */
  Eigen::Matrix<double, DIM, DIM> I = Eigen::MatrixXd::Identity(DIM, DIM);
  /* initial position */
  A_.block(idx_, 0, DIM, DIM) = I;
  b_.segment(idx_, DIM)       = init_.row(0);
  lb_.segment(idx_, DIM)      = init_.row(0);
  idx_ += DIM;
  for (int i = 1; i < M_; i++) {
    A_.block(idx_, i * DIM * (N_ + 1), DIM, DIM)       = I;
    A_.block(idx_, i * DIM * (N_ + 1) - DIM, DIM, DIM) = -I;
    b_.segment(idx_, DIM)                              = Eigen::Vector3d::Zero();
    lb_.segment(idx_, DIM)                             = Eigen::Vector3d::Zero();
    idx_ += DIM;
  }
  /* final position */
  A_.block(idx_, M_ * DIM * (N_ + 1) - DIM, DIM, DIM) = I;
  b_.segment(idx_, DIM)                               = goal_.row(0);
  lb_.segment(idx_, DIM)                              = goal_.row(0);
  idx_ += DIM;

  /* velocity continuity */
  /* initial velocity */
  A_.block(idx_, 0, DIM, DIM)   = -N_ * I;
  A_.block(idx_, DIM, DIM, DIM) = N_ * I;
  b_.segment(idx_, DIM)         = init_.row(1) * t0;
  lb_.segment(idx_, DIM)        = init_.row(1) * t0;
  idx_ += DIM;
  for (int i = 1; i < M_; i++) {
    double t1  = t_[i];
    double t1_ = t_[i - 1];

    A_.block(idx_, i * DIM * (N_ + 1), DIM, DIM)           = -N_ * I / t1;
    A_.block(idx_, i * DIM * (N_ + 1) + DIM, DIM, DIM)     = N_ * I / t1;
    A_.block(idx_, i * DIM * (N_ + 1) - DIM, DIM, DIM)     = -N_ * I / t1_;
    A_.block(idx_, i * DIM * (N_ + 1) - 2 * DIM, DIM, DIM) = N_ * I / t1_;
    b_.segment(idx_, DIM)                                  = Eigen::Vector3d::Zero();
    lb_.segment(idx_, DIM)                                 = Eigen::Vector3d::Zero();
    idx_ += DIM;
  }
  /* final velocity */
  A_.block(idx_, M_ * DIM * (N_ + 1) - DIM * 2, DIM, DIM) = -N_ * I;
  A_.block(idx_, M_ * DIM * (N_ + 1) - DIM, DIM, DIM)     = N_ * I;
  b_.segment(idx_, DIM)                                   = goal_.row(1) * tM;
  lb_.segment(idx_, DIM)                                  = goal_.row(1) * tM;
  idx_ += DIM;

  /* acceleration continuity */
  constexpr int                    DIM3 = DIM * 3;
  Eigen::Matrix<double, DIM, DIM3> p2a  = (v2a_ * p2v_).block<DIM, DIM3>(0, 0);
  /* initial acceleration */
  A_.block(idx_, 0, DIM, DIM3) = p2a;
  b_.segment(idx_, DIM)        = init_.row(2) * t0 * t0;
  lb_.segment(idx_, DIM)       = init_.row(2) * t0 * t0;
  idx_ += DIM;
  for (int i = 1; i < M_; i++) {
    double t2  = pow(t_[i], 2);
    double t2_ = pow(t_[i - 1], 2);

    A_.block(idx_, i * DIM * (N_ + 1), DIM, DIM3)        = p2a / t2;
    A_.block(idx_, i * DIM * (N_ + 1) - DIM3, DIM, DIM3) = -p2a / t2_;
    b_.segment(idx_, DIM)                                = Eigen::Vector3d::Zero();
    lb_.segment(idx_, DIM)                               = Eigen::Vector3d::Zero();
    idx_ += DIM;
  }
  /* final acceleration */
  A_.block(idx_, M_ * DIM * (N_ + 1) - DIM3, DIM, DIM3) = p2a;
  b_.segment(idx_, DIM)                                 = goal_.row(2) * tM * tM;
  lb_.segment(idx_, DIM)                                = goal_.row(2) * tM * tM;
  idx_ += DIM;

  std::cout << "idx: " << idx_ << std::endl;
}

void BezierOpt::addDynamicalConstraints() {
  constexpr int                    DIM2 = DIM * 2;
  constexpr int                    DIM3 = DIM * 3;
  Eigen::Matrix<double, DIM, DIM2> p2v  = p2v_.block<DIM, DIM2>(0, 0);
  Eigen::Matrix<double, DIM, DIM3> p2a  = (v2a_ * p2v_).block<DIM, DIM3>(0, 0);
  Eigen::Matrix<double, DIM, DIM>  I    = Eigen::MatrixXd::Identity(DIM, DIM);

  Eigen::Vector3d ones = Eigen::Vector3d::Ones();

  /* maximum velocity */
  for (int i = 0; i < M_; i++) {
    double t = t_[i];
    for (int j = 0; j < N_; j++) {
      A_.block(idx_, i * DIM * (N_ + 1) + j * DIM, DIM, DIM2) = p2v;
      b_.segment(idx_, DIM)                                   = max_vel_ * ones * t;
      lb_.segment(idx_, DIM)                                  = -max_vel_ * ones * t;
      idx_ += DIM;
    }
  }

  /* maximum acceleration */
  for (int i = 0; i < M_; i++) {
    double t = t_[i];
    for (int j = 0; j < N_ - 1; j++) {
      A_.block(idx_, i * DIM * (N_ + 1) + j * DIM, DIM, DIM3) = p2a;
      b_.segment(idx_, DIM)                                   = max_acc_ * ones * t * t;
      lb_.segment(idx_, DIM)                                  = -max_acc_ * ones * t * t;
      idx_ += DIM;
    }
  }
  std::cout << "idx: " << idx_ << std::endl;
}

void BezierOpt::addSafetyConstraints() {
  for (int i = 0; i < M_; i++) {
    auto c = constraints_[i];
    for (int j = 0; j < c.rows(); j++) {
      Eigen::Vector4d p = c.row(j);
      for (int n = 0; n < N_ + 1; n++) {
        // std::cout << idx_ << "|" << i * DIM * (N_ + 1) + n * DIM << "|" <<
        // p.head(DIM).transpose()
        //           << std::endl;
        A_.block(idx_, i * DIM * (N_ + 1) + n * DIM, 1, DIM) = p.head(DIM).transpose();

        b_[idx_]  = -p[3];
        lb_[idx_] = -OSQP_INFTY;
        idx_++;
      }
    }
  }
  std::cout << "idx: " << idx_ << std::endl;
}

bool BezierOpt::optimize() {
  IOSQP                       solver;
  Eigen::SparseMatrix<double> Q = Q_.sparseView();
  Eigen::SparseMatrix<double> A = A_.sparseView();

  Eigen::VectorXd lb = Eigen::VectorXd::Constant(x_.size(), -OSQP_INFTY);

  c_int flag = solver.setMats(Q, q_, A, lb_, b_, 1e-3, 1e-3);

  if (flag != 0) {
    std::cout << "Problem non-convex. " << std::endl;
    return false;
  } else {
    solver.solve();
    c_int status = solver.getStatus();
    std::cout << "STATUS: " << status << std::endl;
    x_ = solver.getPrimalSol();
    if (status == 1 || status == 2) {
      return true;
    } else {
      return false;
    }
  }
}

void BezierOpt::calcBezierCurve() {
  Eigen::MatrixXd p = getOptCtrlPtsMat();
  bc_.reset(new BezierCurve(t_, p));
}

// bool BezierOpt::optimizeSDQP() {
//   double sdqp_tol      = 1e-3;
//   double sdqp_max_iter = 1000;

//   double sdqp_rst = sdqp::sdqp<-1>(Q_, q_, A_, b_, x_);
// }

}  // namespace traj_opt