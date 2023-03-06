/**
 * @file mini_snap.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-08-05
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <iostream>
#include <polynomial/mini_snap.h>

namespace polynomial {
void CorridorMiniSnap::reset(const Eigen::Matrix3d&                           head,
                             const Eigen::Matrix3d&                           tail,
                             const std::vector<double>&                       timeAlloc,
                             const std::vector<Eigen::Matrix<double, 6, -1>>& corridors) {
  // ROS_INFO_STREAM("[TrajOpt] head state:\n" << head);
  // ROS_INFO_STREAM("[TrajOpt] tail state:\n" << tail);
  _headPVA = head;
  _tailPVA = tail;
  // ROS_INFO_STREAM("[TrajOpt] timeAlloc:" << timeAlloc.size());
  _timeAlloc = timeAlloc;
  _Polygons  = corridors;
  N          = timeAlloc.size();
  int S      = N * (ORDER + 1) * DIM;  // number of variables
  _x.resize(S);
  _Q.resize(S, S);
  _Q.setZero();

  n_hyperplanes = 0;
  for (int i = 0; i < corridors.size() - 1; i++) {
    int c_prev = corridors[i].cols();
    int c_next = corridors[i + 1].cols();
    n_hyperplanes += c_prev;
    n_hyperplanes += c_next;
  }

  /**
   * @brief constraint sampled trajectory points
   */
  int M = DIM * 4 * 2 + DIM * 4 * (N - 1) + n_hyperplanes;
  _A.resize(M, S);
  _A.setZero();
  _ub.resize(M);  // inherited b as upper bound
  _ub.setZero();
  _lb.resize(M);  // lower bound
  _lb.setZero();
}

/**
 * @brief
 * _Polygons[j]: 6-by-4 matrix: [direction, point] x 6
 */
void CorridorMiniSnap::getTransitionConstraint(double delta) {
  int SR        = 24;
  int row_index = SR;
  int N_PIECE   = DIM * (ORDER + 1);           // number of coeffs in single piece (8*3=24)
  int C         = (ORDER + 1);                 // number of coeffs in single dimension
  Eigen::Matrix<double, 1, ORDER + 1> pos_1d;  // end position
  pos_1d << 1, 1, 1, 1, 1, 1, 1, 1;

  for (int j = 0; j < N - 1; j++) {
    Eigen::MatrixXd this_corridor = _Polygons[j];
    Eigen::MatrixXd next_corridor = _Polygons[j + 1];
    for (int i = 0; i < this_corridor.cols(); i++) {
      Eigen::VectorXd v = this_corridor.col(i);
      // int row_index = SR + i + 2 * N_POLYHEDRA * j;
      _A.block(row_index, j * N_PIECE + 0, 1, C)     = pos_1d * v(0);
      _A.block(row_index, j * N_PIECE + C, 1, C)     = pos_1d * v(1);
      _A.block(row_index, j * N_PIECE + C * 2, 1, C) = pos_1d * v(2);

      Eigen::Vector3d n_vec(v(0), v(1), v(2)), p(v(3), v(4), v(5));
      p[1] = p[1] - delta * n_vec[1];
      p[2] = p[2] - 0.5 * delta * n_vec[1];

      _ub(row_index) = n_vec(0) * p(0) + n_vec(1) * p(1) + n_vec(2) * p(2);
      _lb(row_index) = -OSQP_INFTY;
      row_index++;
    }
    for (int i = 0; i < next_corridor.cols(); i++) {
      Eigen::VectorXd v                              = next_corridor.col(i);
      _A.block(row_index, j * N_PIECE + 0, 1, C)     = pos_1d * v(0);
      _A.block(row_index, j * N_PIECE + C, 1, C)     = pos_1d * v(1);
      _A.block(row_index, j * N_PIECE + C * 2, 1, C) = pos_1d * v(2);
      Eigen::Vector3d n_vec(v(0), v(1), v(2)), p(v(3), v(4), v(5));
      p[1] = p[1] - delta * n_vec[1];
      p[2] = p[2] - 0.5 * delta * n_vec[1];

      _ub(row_index) = n_vec(0) * p(0) + n_vec(1) * p(1) + n_vec(2) * p(2);
      _lb(row_index) = -OSQP_INFTY;
      row_index++;
    }
  }
}

void CorridorMiniSnap::getContinuityConstraint() {
  int                                 M  = ORDER + 1;
  int                                 K  = DIM * M;             // 3*8
  int                                 SR = 24 + n_hyperplanes;  // start row
  Eigen::Matrix<double, 1, ORDER + 1> pos_1d;
  Eigen::Matrix<double, 1, ORDER + 1> vel_1d;
  Eigen::Matrix<double, 1, ORDER + 1> acc_1d;
  Eigen::Matrix<double, 1, ORDER + 1> jer_1d;
  pos_1d << 1, 1, 1, 1, 1, 1, 1, 1;
  vel_1d << 0, 1, 2, 3, 4, 5, 6, 7;
  acc_1d << 0, 0, 2, 6, 12, 20, 30, 42;
  jer_1d << 0, 0, 0, 6, 24, 60, 120, 210;

  for (int j = 0; j < N - 1; j++) {
    for (int i = 0; i < DIM; i++) {
      _A.block(SR + 0 + 4 * i + 12 * j, i * M + j * K, 1, ORDER + 1) = pos_1d;
      _A.block(SR + 1 + 4 * i + 12 * j, i * M + j * K, 1, ORDER + 1) = vel_1d / _timeAlloc[j];
      _A.block(SR + 2 + 4 * i + 12 * j, i * M + j * K, 1, ORDER + 1) =
          acc_1d / _timeAlloc[j] / _timeAlloc[j];
      _A.block(SR + 3 + 4 * i + 12 * j, i * M + j * K, 1, ORDER + 1) =
          jer_1d / _timeAlloc[j] / _timeAlloc[j] / _timeAlloc[j];

      _A(SR + 0 + 4 * i + 12 * j, 0 + K * (j + 1) + i * M) = -1;
      _A(SR + 1 + 4 * i + 12 * j, 1 + K * (j + 1) + i * M) = -1 / _timeAlloc[j + 1];
      _A(SR + 2 + 4 * i + 12 * j, 2 + K * (j + 1) + i * M) =
          -2 / _timeAlloc[j + 1] / _timeAlloc[j + 1];
      _A(SR + 3 + 4 * i + 12 * j, 3 + K * (j + 1) + i * M) =
          -6 / _timeAlloc[j + 1] / _timeAlloc[j + 1] / _timeAlloc[j + 1];
    }
  }
}

void CorridorMiniSnap::getHeadTailConstraint() {
  /* constraints for starting states*/
  for (int i = 0; i < DIM; i++) {
    _A(0 + 4 * i, 0 + 8 * i) = 1;
    _A(1 + 4 * i, 1 + 8 * i) = 1;
    _A(2 + 4 * i, 2 + 8 * i) = 2;
    _A(3 + 4 * i, 3 + 8 * i) = 6;

    _ub(0 + 4 * i) = _headPVA(i, 0);
    _ub(1 + 4 * i) = _headPVA(i, 1) * _timeAlloc[0];
    _ub(2 + 4 * i) = _headPVA(i, 2) * _timeAlloc[0] * _timeAlloc[0];
    _ub(3 + 4 * i) = 0;
    _lb(0 + 4 * i) = _headPVA(i, 0);
    _lb(1 + 4 * i) = _headPVA(i, 1) * _timeAlloc[0];
    _lb(2 + 4 * i) = _headPVA(i, 2) * _timeAlloc[0] * _timeAlloc[0];
    _lb(3 + 4 * i) = 0;
  }

  /* constraints for end states */
  int                                 M = (N - 1) * (ORDER + 1) * DIM;
  Eigen::Matrix<double, 1, ORDER + 1> pos_1d;
  Eigen::Matrix<double, 1, ORDER + 1> vel_1d;
  Eigen::Matrix<double, 1, ORDER + 1> acc_1d;
  Eigen::Matrix<double, 1, ORDER + 1> jer_1d;
  pos_1d << 1, 1, 1, 1, 1, 1, 1, 1;
  vel_1d << 0, 1, 2, 3, 4, 5, 6, 7;
  acc_1d << 0, 0, 2, 6, 12, 20, 30, 42;
  jer_1d << 0, 0, 0, 6, 24, 60, 120, 210;
  double T = _timeAlloc[N - 1];
  for (int i = 0; i < DIM; i++) {
    _A.block(12 + 0 + 4 * i, M + 8 * i, 1, ORDER + 1) = pos_1d;
    _A.block(12 + 1 + 4 * i, M + 8 * i, 1, ORDER + 1) = vel_1d;
    _A.block(12 + 2 + 4 * i, M + 8 * i, 1, ORDER + 1) = acc_1d;
    _A.block(12 + 3 + 4 * i, M + 8 * i, 1, ORDER + 1) = jer_1d;

    _ub(12 + 0 + 4 * i) = _tailPVA(i, 0);
    _ub(12 + 1 + 4 * i) = _tailPVA(i, 1) * T;
    _ub(12 + 2 + 4 * i) = _tailPVA(i, 2) * T * T;
    _ub(12 + 3 + 4 * i) = 0;
    _lb(12 + 0 + 4 * i) = _tailPVA(i, 0);
    _lb(12 + 1 + 4 * i) = _tailPVA(i, 1) * T;
    _lb(12 + 2 + 4 * i) = _tailPVA(i, 2) * T * T;
    _lb(12 + 3 + 4 * i) = 0;
  }
}

bool CorridorMiniSnap::primarySolveQP() {
  IOSQP solver;
  // ROS_INFO("[TrajOpt] start solving");
  Eigen::VectorXd q;
  q.resize(N * (ORDER + 1) * DIM);
  q.setZero();
  SparMat Q = _Q.sparseView();
  SparMat A = _A.sparseView();
  //  std::cout << "setMats" << std::endl;
  c_int flag = solver.setMats(Q, q, A, _lb, _ub, 1e-3, 1e-3);
  //  std::cout << "FLAG: " << flag << std::endl;
  if (flag != 0) {
    std::cout << "Problem non-convex. " << std::endl;
    return false;
  }
  solver.solve();
  c_int status = solver.getStatus();
  //  std::cout << "STATUS: " << status << std::endl;
  _x = solver.getPrimalSol();
  if (status == 1 || status == 2) {
    return true;
  } else {
    return false;
  }
}

bool CorridorMiniSnap::optimizeCustomCostFunc(const std::vector<double>& factors, double delta) {
  getCustomCostFunc(factors);
  // ROS_INFO("[TrajOpt] Generated Cost Func");
  getHeadTailConstraint();
  // ROS_INFO("[TrajOpt] Generated Head Tail Constraint");
  getTransitionConstraint(delta);
  // ROS_INFO("[TrajOpt] Generated Transitional Constraint");
  getContinuityConstraint();
  // ROS_INFO("[TrajOpt] Generated Continuity Constraint");
  // getCorridorConstraint();
  // std::cout << "Generated Corridor Constraint" << std::endl;
  bool isSuccess = primarySolveQP();
  // ROS_INFO("[TrajOpt] Solved primary QP");
  return isSuccess;
}

bool CorridorMiniSnap::optimize(double delta) {
  getMiniSnapCostFunc();
  // ROS_INFO("[TrajOpt] Generated Cost Func");
  getHeadTailConstraint();
  // ROS_INFO("[TrajOpt] Generated Head Tail Constraint");
  getTransitionConstraint(delta);
  // ROS_INFO("[TrajOpt] Generated Transitional Constraint");
  getContinuityConstraint();
  // ROS_INFO("[TrajOpt] Generated Continuity Constraint");
  // getCorridorConstraint();
  // std::cout << "Generated Corridor Constraint" << std::endl;
  bool isSuccess = primarySolveQP();
  // ROS_INFO("[TrajOpt] Solved primary QP");
  return isSuccess;
}

bool CorridorMiniSnap::reOptimize() {
  bool isSuccess = primarySolveQP();
  // ROS_INFO("[TrajOpt] Solved new QP problem");
  return isSuccess;
}

/**
 * @brief calculate i!/(i-d)!
 *
 * @author Moji Shi
 * @param i
 * @param d
 * @return double
 */
double divided_factorial(int i, int d) {
  double result = 1;
  for (int j = 0; j < d; j++) {
    result *= i - j;
  }
  return result;
}

void CorridorMiniSnap::getCustomCostFunc(const std::vector<double>& factors) {
  /* for single piece, single dimension */
  int                                         D = ORDER + 1;  // size of matrix Q
  Eigen::Matrix<double, ORDER + 1, ORDER + 1> Q;
  Q.setZero();
  for (int d = 0; d <= 4; d++) {
    for (int i = d; i <= ORDER; i++) {
      for (int j = d; j <= ORDER; j++) {
        if (i + j > 2 * d - 1) {
          Q(i, j) += factors[d] *
                     (divided_factorial(i, d) * divided_factorial(j, d) / (i + j - 2 * d + 1));
        }
      }
    }
  }
  // std::cout << Q;
  /* iterate all dimensions and all pieces */
  for (int i = 0; i < N * DIM; i++) {
    _Q.block(i * D, i * D, D, D) = Q;
  }
}

void CorridorMiniSnap::getMiniSnapCostFunc() {
  /* for single piece, single dimension */
  int D = ORDER + 1;  // size of matrix Q
  Eigen::Matrix<double, ORDER + 1, ORDER + 1> Q;
  for (int i = 0; i <= ORDER; i++) {
    for (int j = 0; j <= ORDER; j++) {
      if (i < 4 || j < 4) {
        Q(i, j) = 0;
      }
      if (i + j > ORDER) {
        Q(i, j) = i * (i - 1) * (i - 2) * (i - 3) * j * (j - 1) * (j - 2) *
                  (j - 3) / (i + j - ORDER);
      }
    }
  }
  /* iterate all dimensions and all pieces */
  for (int i = 0; i < N * DIM; i++) {
    _Q.block(i * D, i * D, D, D) = Q;
  }
}

typedef Eigen::Matrix<double, DIM, ORDER + 1> CoeffMatrix;

/**
 * @brief derivative calculate
 *
 * @param coeff
 * @return Eigen::Matrix<double, DIM, ORDER + 1>
 */

Eigen::Matrix<double, DIM, ORDER> derivative(CoeffMatrix coeff) {
  Eigen::Matrix<double, DIM, ORDER> der;
  for (int i = 0; i < DIM; i++) {
    for (int j = 0; j < ORDER; j++) {
      der(i, j) = coeff(i, j + 1) * (j + 1);
    }
  }
  return der;
}
/** TODO: Templates **/
Eigen::Matrix<double, DIM, ORDER - 1> derivative(Eigen::Matrix<double, DIM, ORDER> coeff) {
  Eigen::Matrix<double, DIM, ORDER - 1> der;
  for (int i = 0; i < DIM; i++) {
    for (int j = 0; j < ORDER - 1; j++) {
      der(i, j) = coeff(i, j + 1) * (j + 1);
    }
  }
  return der;
}
Eigen::Matrix<double, DIM, ORDER - 2> derivative(Eigen::Matrix<double, DIM, ORDER - 1> coeff) {
  Eigen::Matrix<double, DIM, ORDER - 2> der;
  for (int i = 0; i < DIM; i++) {
    for (int j = 0; j < ORDER - 2; j++) {
      der(i, j) = coeff(i, j + 1) * (j + 1);
    }
  }
  return der;
}

/**
 * @brief tangent curve based constraint refinement
 *
 * @param traj
 * @return true
 * @return false
 * @author Moji Shi
 */
bool CorridorMiniSnap::isCorridorSatisfied(Trajectory& traj,
                                           double      max_vel,
                                           double      max_acc,
                                           double      delta) {
  bool                         isSatisfied = true;
  std::vector<Eigen::VectorXd> constraints;
  int                          C        = ORDER + 1;
  int                          N_PIECES = DIM * C;
  int                          S        = N * (ORDER + 1) * DIM;

  /* add position constraints */
  for (int idx = 0; idx < N; idx++) { /* for each piece */
    Eigen::MatrixXd                   polyhedra = _Polygons[idx];
    CoeffMatrix                       coeff     = traj[idx].getCoeffs();
    Eigen::Matrix<double, DIM, ORDER> coeff_dot = derivative(coeff);

    for (int i = 0; i < polyhedra.cols(); i++) { /* for each hyperplane */
      Eigen::Vector3d n_vec = polyhedra.block<3, 1>(0, i);
      Eigen::Vector3d p     = polyhedra.block<3, 1>(3, i);
      Eigen::VectorXd coeff_solver_reverse(7), coeff_solver(7);
      coeff_solver_reverse = n_vec.transpose() * coeff_dot;
      for (int j = 0; j < 7; j++) {
        coeff_solver(j) = coeff_solver_reverse(6 - j);
      }

      std::set<double> allRoots = RootFinder::solvePolynomial(coeff_solver, 0, 1, 0.0000001);

      for (auto itr = allRoots.begin(); itr != allRoots.end(); itr++) {
        double t = *itr;

        Eigen::Vector3d pos = traj[idx].getPos(t);

        if (n_vec.dot(p - pos) < 0) {
          isSatisfied = false;
          /* add a single corridor constraint */
          Eigen::Matrix<double, 1, ORDER + 1> d;
          d << 1, t, pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5), pow(t, 6), pow(t, 7);
          Eigen::Matrix<double, 1, 3 * (ORDER + 1)> d3;
          d3 << n_vec(0) * d, n_vec(1) * d, n_vec(2) * d;
          //          p = p - delta * n_vec;  // constraint with boundary margin
          p[1] = p[1] - delta * n_vec[1];
          p[2] = p[2] - 0.5 * delta * n_vec[1];

          Eigen::VectorXd Ab(S + 1);
          Ab.setZero();
          Ab.segment(idx * N_PIECES, N_PIECES) = d3;
          Ab(S) = n_vec(0) * p(0) + n_vec(1) * p(1) + n_vec(2) * p(2);
          constraints.push_back(Ab);
        }
      }
    }

    std::set<double> root_x, root_y, root_z;
    /* add velocity constraints */
    Eigen::Matrix<double, DIM, ORDER - 1> coeff_dot2 = derivative(coeff_dot);
    Eigen::VectorXd                       a_coeff_x  = coeff_dot2.row(0);
    Eigen::VectorXd                       a_coeff_y  = coeff_dot2.row(1);
    Eigen::VectorXd                       a_coeff_z  = coeff_dot2.row(2);
    root_x = RootFinder::solvePolynomial(a_coeff_x.reverse(), 0, 1, 0.0001);
    root_y = RootFinder::solvePolynomial(a_coeff_y.reverse(), 0, 1, 0.0001);
    root_z = RootFinder::solvePolynomial(a_coeff_z.reverse(), 0, 1, 0.0001);
    std::set<double> vx_roots(root_x.begin(), root_x.end());
    std::set<double> vy_roots(root_y.begin(), root_y.end());
    std::set<double> vz_roots(root_z.begin(), root_z.end());

    int idxt = 0;
    for (auto itr = vx_roots.begin(); itr != vx_roots.end(); itr++) {
      double          t   = *itr;
      Eigen::Vector3d vel = traj[idx].getVel(t);
      //      std::cout << idxt << "\033[1;32m t:" << t << "\tvel_x\t\033[0m" << vel(0) <<
      //      std::endl;
      idxt++;
      if (vel(0) > max_vel) {
        isSatisfied = false;
        Eigen::Matrix<double, 1, ORDER + 1> d;
        d << 0, 1, 2 * pow(t, 1), 3 * pow(t, 2), 4 * pow(t, 3), 5 * pow(t, 4), 6 * pow(t, 5),
            7 * pow(t, 6);
        Eigen::VectorXd Ab(S + 1);
        Ab.setZero();
        Ab.segment(idx * N_PIECES, C) = d;
        //        std::cout << "T " << _timeAlloc[idx] << std::endl;
        Ab(S) = max_vel * _timeAlloc[idx];
        constraints.push_back(Ab);
      }
    }
    for (auto itr = vy_roots.begin(); itr != vy_roots.end(); itr++) {
      double          t   = *itr;
      Eigen::Vector3d vel = traj[idx].getVel(t);
      //      std::cout << "\033[1;32m t:" << t << "\tvel_y\t\033[0m" << vel(1) << std::endl;
      if (vel(1) > max_vel) {
        isSatisfied = false;
        //        std::cout << t << "\tvel_y\t" << vel(1) << std::endl;
        Eigen::Matrix<double, 1, ORDER + 1> d;
        d << 0, 1, 2 * pow(t, 1), 3 * pow(t, 2), 4 * pow(t, 3), 5 * pow(t, 4), 6 * pow(t, 5),
            7 * pow(t, 6);
        Eigen::VectorXd Ab(S + 1);
        Ab.setZero();
        Ab.segment(idx * N_PIECES + C, C) = d;
        Ab(S)                             = max_vel * _timeAlloc[idx];
        constraints.push_back(Ab);
      }
    }
    for (auto itr = vz_roots.begin(); itr != vz_roots.end(); itr++) {
      double          t   = *itr;
      Eigen::Vector3d vel = traj[idx].getVel(t);
      //      std::cout << "\033[1;32m t:" << t << "\tvel_z\t\033[0m" << vel(2) << std::endl;

      if (vel(2) > max_vel) {
        isSatisfied = false;
        //        std::cout << t << "\tvel_z\t" << vel(2) << std::endl;
        Eigen::Matrix<double, 1, ORDER + 1> d;
        d << 0, 1, 2 * pow(t, 1), 3 * pow(t, 2), 4 * pow(t, 3), 5 * pow(t, 4), 6 * pow(t, 5),
            7 * pow(t, 6);
        Eigen::VectorXd Ab(S + 1);
        Ab.setZero();
        Ab.segment(idx * N_PIECES + 2 * C, C) = d;
        Ab(S)                                 = max_vel * _timeAlloc[idx];
        constraints.push_back(Ab);
      }
    }

    /* add acceleration constraints */
    Eigen::Matrix<double, DIM, ORDER - 2> coeff_dot3 = derivative(coeff_dot2);
    Eigen::VectorXd                       j_coeff_x  = coeff_dot3.row(0);
    Eigen::VectorXd                       j_coeff_y  = coeff_dot3.row(1);
    Eigen::VectorXd                       j_coeff_z  = coeff_dot3.row(2);
    root_x = RootFinder::solvePolynomial(j_coeff_x.reverse(), 0, 1, 0.0001);
    // root_y = RootFinder::solvePolynomial(j_coeff_y.reverse(), 0, 1, 0.0001);
    // root_z = RootFinder::solvePolynomial(j_coeff_z.reverse(), 0, 1, 0.0001);
    std::set<double> ax_roots(root_x.begin(), root_x.end());
    // std::set<double> ay_roots(root_y.begin(), root_y.end());
    // std::set<double> az_roots(root_z.begin(), root_z.end());

    for (auto itr = ax_roots.begin(); itr != ax_roots.end(); itr++) {
      double          t   = *itr;
      Eigen::Vector3d acc = traj[idx].getAcc(t);
      //      std::cout << "\033[1;32m t:" << t << "\tacc_x\t\033[0m" << acc(0) << std::endl;
      if (acc(0) > max_acc) {
        isSatisfied = false;
        Eigen::Matrix<double, 1, ORDER + 1> d;
        d << 0, 0, 2, 6 * pow(t, 1), 12 * pow(t, 2), 20 * pow(t, 3), 30 * pow(t, 4), 42 * pow(t, 5);
        Eigen::VectorXd Ab(S + 1);
        Ab.setZero();
        Ab.segment(idx * N_PIECES, C) = d;
        Ab(S)                         = max_acc * pow(_timeAlloc[idx], 2);
        constraints.push_back(Ab);
      }
      if (acc(0) < -max_acc) {
        isSatisfied = false;
        Eigen::Matrix<double, 1, ORDER + 1> d;
        d << 0, 0, 2, 6 * pow(t, 1), 12 * pow(t, 2), 20 * pow(t, 3), 30 * pow(t, 4), 42 * pow(t, 5);
        Eigen::VectorXd Ab(S + 1);
        Ab.setZero();
        Ab.segment(idx * N_PIECES, C) = -d;
        Ab(S)                         = max_acc * pow(_timeAlloc[idx], 2);
        constraints.push_back(Ab);
      }
    }
  }

  /* load constraints to A matrix */
  int n   = constraints.size();
  int ROW = _A.rows();
  _A.conservativeResize(ROW + n, Eigen::NoChange);
  _ub.conservativeResize(ROW + n);
  _lb.conservativeResize(ROW + n);
  for (int i = 0; i < n; i++) {
    _A.row(ROW + i) = constraints[i].head(S);
    _ub(ROW + i)    = constraints[i](S);
    _lb(ROW + i)    = -OSQP_INFTY;
  }
  //  std::cout << "\033[42m"
  //            << "Get new constriants:\tttl size: " << _ub.rows() << "\033[0m" << std::endl;
  return isSatisfied;
}

void CorridorMiniSnap::getCorridorConstraint() { std::cout << "TODO" << std::endl; }

void CorridorMiniSnap::getTrajectory(Trajectory* traj) {
  traj->setDuration(_timeAlloc);
  traj->setCoeffs(_x);
  //  std::cout << (*traj)[3].getCoefficient().row(0) << std::endl;
}

double CorridorMiniSnap::getMinimumCost() const { return _x.transpose() * _Q * _x; }
}  // namespace minisnap