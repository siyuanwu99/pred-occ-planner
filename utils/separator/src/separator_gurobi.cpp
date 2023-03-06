/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include <time.h>
#include <math.h>

#include <chrono>

#include "gurobi_c++.h"

#include "separator.hpp"
#include <iostream>

namespace separator
{
struct Separator::PImpl
{
  GRBEnv* env = new GRBEnv();
  GRBModel model = GRBModel(*env);

  double weight_n0 = 0.0;
  double weight_n1 = 0.0;
  double weight_n2 = 0.0;

  long int num_of_LPs_run = 0;
  double mean_comp_time_ms;

  double epsilon = 1.0;

  GRBVar n0;  // 1st component of the normal of the plane
  GRBVar n1;  // 2nd component of the normal of the plane
  GRBVar n2;  // 3rd component of the normal of the plane
  GRBVar d;   // d component of the plane
};

Separator::~Separator() = default;

Separator::Separator()  // double weight_n1, double weight_n2, double weight_n3
{
  pm_ = std::unique_ptr<PImpl>(new PImpl());
  pm_->n0 = pm_->model.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS);  //, coeff[i] + std::to_string(t)
  pm_->n1 = pm_->model.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS);  //, coeff[i] + std::to_string(t)
  pm_->n2 = pm_->model.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS);  //, coeff[i] + std::to_string(t)
  pm_->d = pm_->model.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS);   //, coeff[i] + std::to_string(t)

  pm_->model.set("OutputFlag", std::to_string(0));  // 1 if you want verbose, 0 if not

  pm_->epsilon = 1.0;
};

long int Separator::getNumOfLPsRun()
{
  return pm_->num_of_LPs_run;
}

double Separator::meanSolveTimeMs()
{
  return pm_->mean_comp_time_ms;
}

bool Separator::solveModel(Eigen::Vector3d& solutionN, double& solutionD, const std::vector<Eigen::Vector3d>& pointsA,
                           const std::vector<Eigen::Vector3d>& pointsB)
{
  Eigen::Matrix<double, 3, Eigen::Dynamic> pointsA_matrix(3, pointsA.size());
  for (int i = 0; i < pointsA.size(); i++)
  {
    pointsA_matrix.col(i) = pointsA[i];
  }

  Eigen::Matrix<double, 3, Eigen::Dynamic> pointsB_matrix(3, pointsB.size());

  for (int i = 0; i < pointsB.size(); i++)
  {
    pointsB_matrix.col(i) = pointsB[i];
  }

  return solveModel(solutionN, solutionD, pointsA_matrix, pointsB_matrix);
}

bool Separator::solveModel(Eigen::Vector3d& solutionN, double& solutionD,
                           const Eigen::Matrix<double, 3, Eigen::Dynamic>& pointsA,
                           const Eigen::Matrix<double, 3, Eigen::Dynamic>& pointsB)
{
  auto start_time = std::chrono::high_resolution_clock::now();
  // std::cout << "pointsA_matrix.cols()=" << pointsA.cols() << std::endl;
  // std::cout << "pointsB_matrix.cols()=" << pointsB.cols() << std::endl;

  ////////////////////////////  RESET (except for the variables)
  ////////////////////////////

  GRBConstr* c = 0;
  c = pm_->model.getConstrs();
  for (int i = 0; i < pm_->model.get(GRB_IntAttr_NumConstrs); ++i)
  {
    pm_->model.remove(c[i]);
  }

  GRBQConstr* cq = 0;
  cq = pm_->model.getQConstrs();
  for (int i = 0; i < pm_->model.get(GRB_IntAttr_NumQConstrs); ++i)
  {
    pm_->model.remove(cq[i]);
  }

  GRBGenConstr* gc = 0;
  gc = pm_->model.getGenConstrs();
  for (int i = 0; i < pm_->model.get(GRB_IntAttr_NumGenConstrs); ++i)
  {
    pm_->model.remove(gc[i]);
  }

  // GRBVar* vars = 0;
  // vars = pm_->model.getVars();
  // for (int i = 0; i < pm_->model.get(GRB_IntAttr_NumVars); ++i)
  // {
  //   pm_->model.remove(vars[i]);
  // }

  pm_->model.reset();  // Note that this function, only by itself, does NOT remove vars or constraints

  ////////////////////////////
  ////////////////////////////

  ////////////////////////////  ADD CONSTRAINTS
  ////////////////////////////

  for (size_t i = 0; i < pointsA.cols(); i++)
  {
    pm_->model.addConstr(pm_->n0 * pointsA(0, i) + pm_->n1 * pointsA(1, i) + pm_->n2 * pointsA(2, i) + pm_->d >=
                         pm_->epsilon);  // n'xA+d >=
                                         // epsilon
  }

  for (size_t i = 0; i < pointsB.cols(); i++)
  {
    pm_->model.addConstr(pm_->n0 * pointsB(0, i) + pm_->n1 * pointsB(1, i) + pm_->n2 * pointsB(2, i) + pm_->d <=
                         -pm_->epsilon);  // n'xB+d
                                          // <=-epsilon
  }
  ////////////////////////////
  ////////////////////////////

  ////////////////////////////  ADD OBJECTIVE
  ////////////////////////////
  pm_->model.setObjective(pm_->weight_n0 * pm_->n0 + pm_->weight_n1 * pm_->n1 + pm_->weight_n2 * pm_->n2, GRB_MINIMIZE);
  ////////////////////////////
  ////////////////////////////

  pm_->model.update();  // needed due to the lazy evaluation
  // pm_->model.write("/home/jtorde/Desktop/ws/src/mader/model.lp");
  pm_->model.optimize();

  int optimstatus = pm_->model.get(GRB_IntAttr_Status);

  pm_->num_of_LPs_run++;  // Now pm_->num_of_LPs_run counts also the last LP run
  double total_time_us =
      (std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time))
          .count();

  // https://math.stackexchange.com/a/22351
  pm_->mean_comp_time_ms =
      pm_->mean_comp_time_ms + (total_time_us / 1e3 - pm_->mean_comp_time_ms) / pm_->num_of_LPs_run;

  // std::cout << "total_time_us LP =" << total_time_us << "us" << std::endl;
  // std::cout << "mean comp time LP =" << pm_->mean_comp_time_ms * 1000 << "us" << std::endl;
  // std::cout << "pm_->num_of_LPs_run LP =" << pm_->num_of_LPs_run << std::endl;

  // int number_of_stored_solutions = pm_->model.get(GRB_IntAttr_SolCount);
  // || optimstatus == GRB_TIME_LIMIT ||
  //       optimstatus == GRB_USER_OBJ_LIMIT ||                                    ///////////////
  //       optimstatus == GRB_ITERATION_LIMIT || optimstatus == GRB_NODE_LIMIT ||  ///////////////
  //       optimstatus == GRB_SOLUTION_LIMIT) &&
  //      number_of_stored_solutions > 0
  if (optimstatus == GRB_OPTIMAL)
  {
    solutionN(0) = pm_->n0.get(GRB_DoubleAttr_X);
    solutionN(1) = pm_->n1.get(GRB_DoubleAttr_X);
    solutionN(2) = pm_->n2.get(GRB_DoubleAttr_X);
    solutionD = pm_->d.get(GRB_DoubleAttr_X);
    return true;
  }
  else
  {
    // std::cout << "Gurobi (LP) failed to find a solution" << std::endl;
    return false;
  }
};

}  // namespace separator
