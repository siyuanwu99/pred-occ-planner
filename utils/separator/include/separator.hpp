/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#pragma once

#include <array>
#include <vector>
#include <Eigen/Dense>
#include <memory>

namespace separator
{
class Separator
{
public:
  Separator();
  ~Separator();

  bool solveModel(Eigen::Vector3d& solution, double& d, const std::vector<Eigen::Vector3d>& pointsA,
                  const std::vector<Eigen::Vector3d>& pointsB);

  bool solveModel(Eigen::Vector3d& solutionN, double& solutionD,
                  const Eigen::Matrix<double, 3, Eigen::Dynamic>& pointsA,
                  const Eigen::Matrix<double, 3, Eigen::Dynamic>& pointsB);

  long int getNumOfLPsRun();

  double meanSolveTimeMs();

private:
  // PImpl idiom, https://www.geeksforgeeks.org/pimpl-idiom-in-c-with-examples/
  struct PImpl;
  std::unique_ptr<PImpl> pm_;  // Opaque pointer
};

}  // namespace separator
