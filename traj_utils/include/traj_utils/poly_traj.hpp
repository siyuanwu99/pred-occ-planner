/**
 * @file mini_snap_utils.hpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-08-05
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef _MINI_SNAP_UTILS_H_
#define _MINI_SNAP_UTILS_H_

#include <stdlib.h>

#include <Eigen/Eigen>
#include <cmath>
#include <memory>
#include <vector>

#include "root_finder.hpp"

// #define ORDER       7  // order of polynomial trajectory
// #define D_ORDER     4  // order of maximum derivative (4 for minisnap)
// #define DIM         3  // number of dimensions in Cspace
// #define N_POLYHEDRA 6  // number of polygons in polyhedra

namespace polynomial {
const int ORDER       = 7;  // order of polynomial trajectory
const int D_ORDER     = 4;  // order of maximum derivative (4 for minisnap)
const int DIM         = 3;  // number of dimensions in Cspace
const int N_POLYHEDRA = 6;  // number of polygons in polyhedra

typedef Eigen::Matrix<double, DIM, ORDER + 1> CoefficientMat;
typedef Eigen::SparseMatrix<double>           SparMat;

class PolyPiece {
 private:
  double _duration;
  /** normalized coefficients
   * c0 * 1 + c1 * t + c2 * t^2 + ... + c7 * t^7
   * coeff: [c0, c1, c2, c3, c4, c5, c6, c7]
   */
  CoefficientMat _coeffs;

 public:
  PolyPiece() = default;
  PolyPiece(double duration, const CoefficientMat& coeffs) : _duration(duration), _coeffs(coeffs) {}

  void setup(const double duration) { _duration = duration; }
  void setup(const CoefficientMat& coeffs) { _coeffs = coeffs; }

  inline int                   getDim() const { return DIM; }
  inline int                   getOrder() const { return ORDER; }
  inline double                getDuration() const { return _duration; }
  inline const CoefficientMat& getCoeffs() const { return _coeffs; }

  /**
   * @brief get position at normalized time t
   * @param nt normalized time
   * @return position at normalized time nt
   */
  inline Eigen::Vector3d getPos(double nt) const {
    Eigen::Vector3d pos(0.0, 0.0, 0.0);
    double          t = 1.0;
    for (int i = 0; i <= ORDER; i++) {
      pos += _coeffs.col(i) * t;
      t *= nt;
    }
    return pos;
  }

  /**
   * @brief get velocity at normalized time t
   * @param nt normalized time
   * @return velocity at normalized time nt
   */
  inline Eigen::Vector3d getVel(double nt) const {
    Eigen::Vector3d vel(0.0, 0.0, 0.0);
    double          t = 1.0;
    for (int i = 1; i <= ORDER; i++) {
      vel += i * _coeffs.col(i) * t;
      t *= nt;
    }
    vel = vel / _duration;  // normalize derivative
    return vel;
  }

  /**
   * @brief get acceleration at normalized time t
   * @param nt normalized time
   * @return acceleration at normalized time nt
   */
  inline Eigen::Vector3d getAcc(double nt) const {
    Eigen::Vector3d acc(0.0, 0.0, 0.0);
    double          t = 1.0;
    for (int i = 2; i <= ORDER; i++) {
      acc += i * (i - 1) * _coeffs.col(i) * t;
      t *= nt;
    }
    acc = acc / (_duration * _duration);  // normalize 2nd-order derivative
    return acc;
  }

  /**
   * @brief get jerk at normalized time t
   * @param nt normalized time
   * @return jerk at normalized time nt
   */
  inline Eigen::Vector3d getJrk(double nt) const {
    Eigen::Vector3d jrk(0.0, 0.0, 0.0);
    double          t = 1.0;
    for (int i = 3; i <= ORDER; i++) {
      jrk += i * (i - 1) * (i - 2) * _coeffs.col(i) * t;
      t *= nt;
    }
    jrk = jrk / (_duration * _duration * _duration);  // normalize 3rd-order derivative
    return jrk;
  }

  inline Eigen::MatrixXd getVelCoeffMat() const {
    Eigen::Matrix<double, DIM, ORDER> velCoeffMat;
    for (int i = 1; i <= ORDER; i++) {
      velCoeffMat.col(i - 1) = i * _coeffs.col(i);
    }
    return velCoeffMat;
  }

  inline Eigen::MatrixXd getAccCeoffMat() const {
    Eigen::Matrix<double, DIM, ORDER - 1> accCoeffMat;
    for (int i = 2; i <= ORDER; i++) {
      accCoeffMat.col(i - 2) = i * (i - 1) * _coeffs.col(i);
    }
    return accCoeffMat;
  }

  // TODO: bug
  inline double getMaxVelRate() const {
    Eigen::MatrixXd velCoeffMat = getVelCoeffMat();
    Eigen::VectorXd coeff       = RootFinder::polySqr(velCoeffMat.row(0).reverse()) +
                            RootFinder::polySqr(velCoeffMat.row(1).reverse()) +
                            RootFinder::polySqr(velCoeffMat.row(2).reverse());
    int N = coeff.size();
    int n = N - 1;
    for (int i = 0; i < N; i++) {
      coeff(i) *= n;
      n--;
    }
    if (coeff.head(N - 1).squaredNorm() < DBL_EPSILON) {
      return 0.0;
    } else {
      double l = -0.0625;
      double r = 1.0625;
      while (fabs(RootFinder::polyVal(coeff.head(N - 1), l)) < DBL_EPSILON) {
        l = 0.5 * l;
      }
      while (fabs(RootFinder::polyVal(coeff.head(N - 1), r)) < DBL_EPSILON) {
        r = 0.5 * (r + 1.0);
      }
      std::set<double> candidates =
          RootFinder::solvePolynomial(coeff.head(N - 1), l, r, FLT_EPSILON / _duration);
      candidates.insert(0.0);
      candidates.insert(1.0);
      double maxVelRateSqr = -INFINITY;
      double tempNormSqr;
      for (std::set<double>::const_iterator it = candidates.begin(); it != candidates.end(); it++) {
        if (0.0 <= *it && 1.0 >= *it) {
          tempNormSqr   = getVel((*it)).squaredNorm();
          maxVelRateSqr = maxVelRateSqr < tempNormSqr ? tempNormSqr : maxVelRateSqr;
        }
      }
      return sqrt(maxVelRateSqr);
    }
  }

  inline double getMaxAccRate() const {
    Eigen::MatrixXd accCoeffMat = getAccCeoffMat();
    Eigen::VectorXd coeff       = RootFinder::polySqr(accCoeffMat.row(0).reverse()) +
                            RootFinder::polySqr(accCoeffMat.row(1).reverse()) +
                            RootFinder::polySqr(accCoeffMat.row(2).reverse());
    int N = coeff.size();
    int n = N - 1;
    for (int i = 0; i < N; i++) {
      coeff(i) *= n;
      n--;
    }
    if (coeff.head(N - 1).squaredNorm() < DBL_EPSILON) {
      return 0.0;
    } else {
      double l = -0.0625;
      double r = 1.0625;
      while (fabs(RootFinder::polyVal(coeff.head(N - 1), l)) < DBL_EPSILON) {
        l = 0.5 * l;
      }
      while (fabs(RootFinder::polyVal(coeff.head(N - 1), r)) < DBL_EPSILON) {
        r = 0.5 * (r + 1.0);
      }
      std::set<double> candidates =
          RootFinder::solvePolynomial(coeff.head(N - 1), l, r, FLT_EPSILON / _duration);
      candidates.insert(0.0);
      candidates.insert(1.0);
      double maxAccRateSqr = -INFINITY;
      double tempNormSqr;
      for (std::set<double>::const_iterator it = candidates.begin(); it != candidates.end(); it++) {
        if (0.0 <= *it && 1.0 >= *it) {
          tempNormSqr   = getAcc(*it).squaredNorm();
          maxAccRateSqr = maxAccRateSqr < tempNormSqr ? tempNormSqr : maxAccRateSqr;
        }
      }
      return sqrt(maxAccRateSqr);
    }
  }

};  // class PolyPiece

class Trajectory {
 private:
  typedef std::vector<PolyPiece> Pieces;
  Pieces                         _pieces;
  int                            _n_pieces;

 public:
  typedef std::shared_ptr<Trajectory> Ptr;
  Trajectory() = default;
  Trajectory(const std::vector<double>& durations, const std::vector<CoefficientMat>& coeffs) {
    int _n_pieces = durations.size();
    for (int i = 0; i < _n_pieces; i++) {
      _pieces.push_back(PolyPiece(durations[i], coeffs[i]));
    }
  }

  void setDuration(const std::vector<double>& durations) {
    _n_pieces = durations.size();
    _pieces.resize(_n_pieces);
    for (int i = 0; i < _n_pieces; i++) {
      _pieces[i].setup(durations[i]);
    }
  }

  void setCoeffs(const Eigen::VectorXd& x) {
    int M = ORDER + 1;
    for (int i = 0; i < _n_pieces; i++) {
      Eigen::Matrix<double, DIM, ORDER + 1> temp;
      temp << x.segment(DIM * M * i, M).transpose(), x.segment(DIM * M * i + M, M).transpose(),
          x.segment(DIM * M * i + 2 * M, M).transpose();
      _pieces[i].setup(temp);
    }
  }
  void setCoeffs(const std::vector<CoefficientMat>& coeffs) {
    _n_pieces = coeffs.size();
    _pieces.resize(_n_pieces);
    for (int i = 0; i < _n_pieces; i++) {
      _pieces[i].setup(coeffs[i]);
    }
  }
  void clear() {
    _pieces.clear();
    _n_pieces = 0;
  }

  inline int    getPieceNum() const { return _n_pieces; }
  inline double getDuration() const {
    double duration = 0.0;
    for (int i = 0; i < _n_pieces; i++) {
      duration += _pieces[i].getDuration();
    }
    return duration;
  }

  /**
   * @brief given absolute timestamp, find out piece index and relative timestamp
   * @param t absolute time stamp
   * @param nt normalized time stamp, return nt \in [0, 1)
   * @param idx
   */
  inline void locatePiece(const double& t, double& nt, int& idx) const {
    idx        = _n_pieces;
    double tmp = t;
    for (int i = 0; i < _n_pieces; i++) {
      double Ti = _pieces[i].getDuration();
      if (tmp > Ti) {
        tmp -= Ti;
      } else {
        idx = i;
        nt  = tmp / Ti;
        break;
      }
    }
    /* if t0 is longer than all durations */
    if (idx == _n_pieces) {
      idx = _n_pieces - 1;
      nt  = 1;
    }
  }

  inline Eigen::Vector3d getPos(double t) const {
    double nt;
    int    idx;
    locatePiece(t, nt, idx);
    return _pieces[idx].getPos(nt);
  }

  inline Eigen::Vector3d getVel(double t) const {
    double nt;
    int    idx;
    locatePiece(t, nt, idx);
    return _pieces[idx].getVel(nt);
  }

  inline Eigen::Vector3d getAcc(double t) const {
    double nt;
    int    idx;
    locatePiece(t, nt, idx);
    return _pieces[idx].getAcc(nt);
  }

  inline Eigen::Vector3d getJrk(double t) const {
    double nt;
    int    idx;
    locatePiece(t, nt, idx);
    return _pieces[idx].getJrk(nt);
  }

  double getMaxVelRate() const {
    double max_vel_rate = -INFINITY;
    for (int i = 0; i < _n_pieces; i++) {
      double tmp   = _pieces[i].getMaxVelRate();
      max_vel_rate = max_vel_rate < tmp ? tmp : max_vel_rate;
    }
    return max_vel_rate;
  }

  double getMaxAccRate() const {
    double max_acc_rate = -INFINITY;
    for (int i = 0; i < _n_pieces; i++) {
      double tmp   = _pieces[i].getMaxAccRate();
      max_acc_rate = max_acc_rate < tmp ? tmp : max_acc_rate;
    }
    return max_acc_rate;
  }

  const PolyPiece& operator[](int i) const { return _pieces[i]; }
  PolyPiece&       operator[](int i) { return _pieces[i]; }
};

}  // namespace polynomial

#endif  // MINISNAP_UTILS_H_