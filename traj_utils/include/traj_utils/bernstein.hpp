/**
 * @file bernstein.hpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-09-06
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef _BERNSTEIN_H_
#define _BERNSTEIN_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <memory>
#include <vector>

namespace Bernstein {
const int ORDER = 4;  // order of Bezier curve, default 4
const int DIM   = 3;  // dimension of the trajectory

class BernsteinPiece {
 private:
  /** control points for Bernstein with different dimensions.
   * Each row represents one single control point
   * The dimension is determined by column number
   * e.g. Bernstein with N points in 3D space -> Nx3 matrix */
  Eigen::MatrixXd cpts_;     // control points
  Eigen::MatrixXd A_;        // coefficient matrix
  int             N_;        // order
  double          t0_, tf_;  // time interval
  double          t_;        // duration

 public:
  BernsteinPiece() = default;
  BernsteinPiece(const Eigen::MatrixX3d &cpts, const double &t0, const double &tf) {
    cpts_ = cpts;
    N_    = ORDER;
    t0_   = t0;
    tf_   = tf;
    t_    = tf_ - t0_;
    assert(cpts_.rows() == N_ + 1);  // 4th order curve has 5 control points
    calcCoeffMat(N_, A_);
  }
  BernsteinPiece(const Eigen::MatrixX3d &cpts, const double &t) {
    cpts_ = cpts;
    N_    = ORDER;
    t0_   = 0;
    tf_   = t;
    t_    = t;
    assert(cpts_.rows() == N_ + 1);
    calcCoeffMat(N_, A_);
  }

  ~BernsteinPiece() {}
  void setControlPoints(const Eigen::MatrixX3d &cpts) {
    cpts_ = cpts;
    N_    = ORDER;
    assert(cpts_.rows() == N_ + 1);
    calcCoeffMat(N_, A_);
  }
  void setTimeInterval(const double &t0, const double &tf) {
    t0_ = t0;
    tf_ = tf;
    t_  = tf_ - t0_;
  }
  inline int    getDim() const { return DIM; }
  inline int    getOrder() const { return N_; }
  inline double getStartTime() const { return t0_; }
  inline double getEndTime() const { return tf_; }
  inline double getDuration() const { return tf_ - t0_; }

  Eigen::Vector3d getPos(double t) const;
  Eigen::Vector3d getVel(double t) const;
  Eigen::Vector3d getAcc(double t) const;
  Eigen::Vector3d getJrk(double t) const;

  Eigen::MatrixXd getPosCtrlPts() const { return cpts_; }
  Eigen::MatrixXd getVelCtrlPts() const { return calcDerivativeCtrlPts(getPosCtrlPts()); }
  Eigen::MatrixXd getAccCtrlPts() const { return calcDerivativeCtrlPts(getVelCtrlPts()); }
  Eigen::MatrixXd getCoeffMat() const { return A_; }
  double          getMaxVelRate() const;
  double          getMaxAccRate() const;

 private:
  Eigen::MatrixXd calcDerivativeCtrlPts(const Eigen::MatrixXd &cpts) const;

  void calcCoeffMat();
  void calcCoeffMat(int n, Eigen::MatrixXd &A);

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class Bezier {
 private:
  typedef std::vector<BernsteinPiece> Pieces;
  Pieces                              pieces_;
  int                                 N_;     // order
  int                                 M_;     // number of pieces
  double                              T_;     // total time
  std::vector<double>                 t_;     // time interval for each piece
  Eigen::MatrixX3d                    cpts_;  // control points, (M(N+1))x3 matrix

 public:
  Bezier() {}
  Bezier(const double &time) : T_(time) {
    N_ = ORDER;
    M_ = 1;
    t_.push_back(T_);
  }

  Bezier(const std::vector<double> &time) : t_(time) {
    N_ = ORDER;
    M_ = t_.size();
    T_ = 0;
    for (int i = 0; i < M_; i++) {
      T_ += t_[i];
    }
  }

  Bezier(const std::vector<double> &time, const Eigen::MatrixX3d &cpts) : t_(time), cpts_(cpts) {
    N_ = ORDER;
    M_ = t_.size();
    T_ = 0;
    for (int i = 0; i < M_; i++) {
      T_ += t_[i];
    }
    setControlPoints(cpts_);
  }
  ~Bezier() {}

  /* get & set basic info */
  void setOrder(const int &order) { N_ = order; }
  void setTime(const std::vector<double> &t) {
    t_ = t;
    M_ = t_.size();
    T_ = 0;
    for (auto it = t_.begin(); it != t_.end(); it++) {
      T_ += *it;
    }
  }
  void setControlPoints(const Eigen::MatrixX3d &cpts);

  int             getOrder() const { return N_; }
  int             getNumPieces() const { return M_; }
  double          getDuration() const { return T_; }
  void            getCtrlPoints(Eigen::MatrixXd &cpts) const { cpts = cpts_; }
  Eigen::MatrixXd getCtrlPoints() const { return cpts_; }
  Eigen::MatrixXd getVelCtrlPoints(int idx) const { return pieces_[idx].getVelCtrlPts(); }
  Eigen::MatrixXd getAccCtrlPoints(int idx) const { return pieces_[idx].getAccCtrlPts(); }
  void            calcPieces();
  void            clear() {
    t_.clear();
    pieces_.clear();
    cpts_.resize(0, 3);
    T_ = 0;
    M_ = 0;
  }

  inline int locatePiece(double t) const {
    for (int i = 0; i < M_; i++) {
      t -= t_[i];
      if (t < 0) {
        return i;
      }
    }
    return M_ - 1;
  }

  inline Eigen::Vector3d getPos(double t) const {
    int i = locatePiece(t);
    return pieces_[i].getPos(t);
  }

  inline Eigen::Vector3d getVel(double t) const {
    int i = locatePiece(t);
    return pieces_[i].getVel(t);
  }

  inline Eigen::Vector3d getAcc(double t) const {
    int i = locatePiece(t);
    return pieces_[i].getAcc(t);
  }

  double getMaxVelRate() const;
  double getMaxAccRate() const;

  const BernsteinPiece &operator[](int i) const { return pieces_[i]; }
  BernsteinPiece       &operator[](int i) { return pieces_[i]; }

  typedef std::shared_ptr<Bezier> Ptr;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace Bernstein

#endif
