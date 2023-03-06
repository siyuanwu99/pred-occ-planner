/**
 * @file mini_snap.hpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief Solve minimun snap trajectory problem by using iosqp solver
 * @version 1.0
 * @date 2022-08-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef MINI_SNAP_H_
#define MINI_SNAP_H_

#include <stdlib.h>

#include <Eigen/Eigen>
#include <cmath>
#include <vector>

#include "iosqp.hpp"
#include "traj_utils/poly_traj.hpp"

namespace polynomial{
/***************************************/
/********** Corridor MiniSnap **********/
/***************************************/

/** Jing Chen, Tianbo Liu and Shaojie Shen, "Online generation of collision-free
 * trajectories for quadrotor flight in unknown cluttered environments," 2016
 * IEEE International Conference on Robotics and Automation (ICRA), 2016, pp.
 * 1476-1483, doi: 10.1109/ICRA.2016.7487283.
 */
class CorridorMiniSnap {
 private:
  int                                       N;  // number of pieces
  int                                       n_hyperplanes;
  Eigen::Matrix3d                           _headPVA;  // head's pos, vel, acc
  Eigen::Matrix3d                           _tailPVA;  // tail's pos, vel, acc
  Eigen::MatrixXd                           _Q;
  Eigen::MatrixXd                           _A;
  /**
   * @brief solution of the QP problem
   * _x: DIM * (ORDER + 1) * N vector of coefficients
   * [1st piece] [2nd piece] ... [Nth piece]
   * [i-th piece]: [xxxxxxxx] [yyyyyyyy] [zzzzzzzz]
   */
  Eigen::VectorXd                           _x;
  Eigen::VectorXd                           _ub;
  Eigen::VectorXd                           _lb;
  std::vector<double>                       _timeAlloc;
  std::vector<Eigen::Matrix<double, 6, -1>> _Polygons;

 public:
  CorridorMiniSnap() {}
  ~CorridorMiniSnap() {}
  void reset(const Eigen::Matrix3d&                           head,
             const Eigen::Matrix3d&                           tail,
             const std::vector<double>&                       timeAlloc,
             const std::vector<Eigen::Matrix<double, 6, -1>>& corridors);

  void getCustomCostFunc(const std::vector<double>& factors);
  void getMiniSnapCostFunc();
  void getCorridorConstraint();
  void getTransitionConstraint(double delta);
  void getContinuityConstraint();
  void getHeadTailConstraint();

  bool optimize(double delta);
  bool optimizeCustomCostFunc(const std::vector<double>& factors, double delta);
  bool primarySolveQP();
  bool reOptimize();
  // inline bool isCorridorSatisfied(const Eigen::Vector3d & pos, int idx,
  // double t);
  bool   isCorridorSatisfied(Trajectory& traj, double max_vel, double max_acc, double delta);
  void   getTrajectory(Trajectory* traj);
  double getMinimumCost() const;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<CorridorMiniSnap> Ptr;
};

}  // namespace minisnap

#endif  // MINI_SNAP_H_