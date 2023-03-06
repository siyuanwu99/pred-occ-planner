/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include <iostream>
#include <array>
#include "separator.hpp"

int main()
{
  separator::Separator separator_solver;

  double num_points_set_A = 4;
  double num_points_set_B = 3;

  Eigen::Matrix<double, 3, -1> pointsA(3, 4);  // Each column is a point of the set A
  Eigen::Matrix<double, 3, -1> pointsB(3, 5);  // Each column is a point of the set B

  pointsA.col(0) = Eigen::Vector3d(-3.50, 21, 1.4);
  pointsA.col(1) = Eigen::Vector3d(-2.71, 2.13, 1.6);
  pointsA.col(2) = Eigen::Vector3d(0.53, 0.51, 1.4);
  pointsA.col(3) = Eigen::Vector3d(-3.50, 0.21, 0.3);

  pointsB.col(0) = Eigen::Vector3d(-2.3, 4.69, 6.2);
  pointsB.col(1) = Eigen::Vector3d(3.7, 2.13, 65.6);
  pointsB.col(2) = Eigen::Vector3d(6.5, 2.93, 2.8);
  pointsB.col(3) = Eigen::Vector3d(0.3, 4.8, 9.2);
  pointsB.col(4) = Eigen::Vector3d(1.5, 6.7, 2.9);

  std::cout << "pointsA= \n" << pointsA << std::endl;
  std::cout << "pointsB= \n" << pointsB << std::endl;

  Eigen::Vector3d n;
  double d;
  bool solved = separator_solver.solveModel(n, d, pointsA, pointsB);
  std::cout << "Solved= " << solved << std::endl;
  std::cout << "n= " << n.transpose() << std::endl;
  std::cout << "d= " << d << std::endl;

  return 0;
};
