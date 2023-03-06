/**
 * @file bernstein.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-09-06
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "traj_utils/bernstein.hpp"

namespace Bernstein {

/**
 * @brief
 * p(t) = C^T * A * [1, t, t^2, t^3, t^4]^T
 * C: (N+1)x3 control points
 * A: 5x5 coefficient matrix
 * T: 5x1 time vector
 * @param t
 * @return Eigen::Vector3d
 */
Eigen::Vector3d BernsteinPiece::getPos(double t) const {
  double          s = (t - t0_) / t_;
  Eigen::VectorXd S;
  S.resize(N_ + 1);
  S(0) = 1;
  for (int i = 1; i <= N_; i++) {
    S(i) = pow(s, i);
  }
  return cpts_.transpose() * A_ * S;
}

Eigen::Vector3d BernsteinPiece::getVel(double t) const {
  double          s = (t - t0_) / t_;
  Eigen::VectorXd S;
  S.resize(N_ + 1);
  S(0) = 0;
  S(1) = 1;
  for (int i = 2; i <= N_; i++) {
    S(i) = i * pow(s, i - 1);
  }
  return cpts_.transpose() * A_ * S / t_;
}

Eigen::Vector3d BernsteinPiece::getAcc(double t) const {
  double          s = (t - t0_) / t_;
  Eigen::VectorXd S;
  S.resize(N_ + 1);
  S(0) = 0;
  S(1) = 0;
  S(2) = 2;
  for (int i = 3; i <= N_; i++) {
    S(i) = i * (i - 1) * pow(s, i - 2);
  }
  return cpts_.transpose() * A_ * S / pow(t_, 2);
}

void BernsteinPiece::calcCoeffMat() {
  A_.resize(N_ + 1, N_ + 1);
  A_.setZero();
  switch (N_) {
    case 1:
      A_ << 1, -1, 0, 1;
      break;
    case 2:
      A_ << 1, -2, 1, 0, 2, -2, 0, 0, 1;
      break;
    case 3:
      A_ <<
          // clang-format off
      1, -3, 3, 1,
      0, 3, -6, 3,
      0, 0, 3, -3,
      0, 0, 0, 1;
      // clang-format on
      break;
    case 4:
      A_ <<
          // clang-format off
      1, -4,   6,  -4,  1,
      0,  4, -12,  12, -4,
      0,  0,   6, -12,  6,
      0,  0,   0,   4, -4,
      0,  0,   0,   0,  1;
      // clang-format on
      break;
  }
}

void BernsteinPiece::calcCoeffMat(int n, Eigen::MatrixXd &A) {
  A.resize(n + 1, n + 1);
  A.setZero();
  switch (n) {
    case 1:
      A << 1, -1, 0, 1;
      break;
    case 2:
      A << 1, -2, 1, 0, 2, -2, 0, 0, 1;
      break;
    case 3:
      A <<
          // clang-format off
      1, -3, 3, 1,
      0, 3, -6, 3,
      0, 0, 3, -3,
      0, 0, 0, 1;
      // clang-format on
      break;
    case 4:
      A <<
          // clang-format off
      1, -4,   6,  -4,  1,
      0,  4, -12,  12, -4,
      0,  0,   6, -12,  6,
      0,  0,   0,   4, -4,
      0,  0,   0,   0,  1;
      // clang-format on
      break;
  }
}

Eigen::MatrixXd BernsteinPiece::calcDerivativeCtrlPts(const Eigen::MatrixXd &cpts) const {
  int n = cpts.rows() - 1;
  assert(n >= 1);
  Eigen::MatrixXd d_cpts;
  d_cpts.resize(n, 3);
  for (int i = 0; i < n; i++) {
    d_cpts.row(i) = n * (cpts.row(i + 1) - cpts.row(i));
  }
  return d_cpts;
}

/**
 * @brief TODO: check this function
 *
 * @return double
 */
// double BernsteinPiece::getMaxVelRate() const {
//   double max_vel_rate = 0;
//   for (int i = 0; i <= N_; i++) {
//     double vel_rate = cpts_.row(i).norm() / (i + 1);
//     if (vel_rate > max_vel_rate) {
//       max_vel_rate = vel_rate;
//     }
//   }
//   return max_vel_rate;
// }

double BernsteinPiece::getMaxVelRate() const {
  Eigen::MatrixXd d_cpts       = getVelCtrlPts();
  double          max_vel_rate = 0;
  for (int i = 0; i < d_cpts.rows(); i++) {
    double vel_rate = d_cpts.row(i).norm();
    if (vel_rate > max_vel_rate) {
      max_vel_rate = vel_rate;
    }
  }
  return max_vel_rate / t_;
}

double BernsteinPiece::getMaxAccRate() const {
  Eigen::MatrixXd d_cpts       = getAccCtrlPts();
  double          max_acc_rate = 0;
  for (int i = 0; i < d_cpts.rows(); i++) {
    double acc_rate = d_cpts.row(i).norm();
    if (acc_rate > max_acc_rate) {
      max_acc_rate = acc_rate;
    }
  }
  return max_acc_rate / pow(t_, 2);
}

/********** Bezier Curve **********/
void Bezier::setControlPoints(const Eigen::MatrixX3d &cpts) {
  cpts_ = cpts;
  assert(cpts_.rows() == M_ * (N_ + 1) && "Piece number does not match time interval number");
  assert(cpts_.cols() == DIM && "Dimension does not match");
  calcPieces();
}

void Bezier::calcPieces() {
  pieces_.reserve(M_);
  double t = 0;
  for (int i = 0; i < M_; i++) {
    Eigen::MatrixXd cpts;
    cpts.resize(N_ + 1, 3);
    for (int j = 0; j <= N_; j++) {
      cpts.row(j) = cpts_.row(i * (N_+1) + j);
    }
    pieces_.emplace_back(cpts, t, t + t_[i]);
    t += t_[i];
  }
}

double Bezier::getMaxVelRate() const {
  double max_vel_rate = 0;
  for (int i = 0; i < M_; i++) {
    double vel_rate = pieces_[i].getMaxVelRate();
    if (vel_rate > max_vel_rate) {
      max_vel_rate = vel_rate;
    }
  }
  return max_vel_rate;
}

double Bezier::getMaxAccRate() const {
  double max_acc_rate = 0;
  for (int i = 0; i < M_; i++) {
    double acc_rate = pieces_[i].getMaxAccRate();
    if (acc_rate > max_acc_rate) {
      max_acc_rate = acc_rate;
    }
  }
  return max_acc_rate;
}

}  // namespace Bernstein