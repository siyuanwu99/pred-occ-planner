/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include <time.h>
#include <math.h>
#include <glpk.h> /* GNU GLPK linear/mixed integer solver */

#include <chrono>

#include "separator.hpp"
#include <iostream>

namespace separator
{
struct Separator::PImpl
{
  glp_prob* lp;
  int ia[10000], ja[10000];  // TODO
  double ar[10000];          // TODO
  double weight_n0 = 0.0;
  double weight_n1 = 0.0;
  double weight_n2 = 0.0;

  glp_smcp params;

  long int num_of_LPs_run = 0;
  double mean_comp_time_ms;
};

Separator::~Separator() = default;

Separator::Separator()  // double weight_n0, double weight_n1, double weight_n2
{
  pm_ = std::unique_ptr<PImpl>(new PImpl());
  glp_init_smcp(&pm_->params);
  pm_->params.msg_lev = 1;  // 1=no output.  GLP_MSG_ALL;
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

  // std::cout << "Using GLPK" << std::endl;

  // std::cout << "pointsA_matrix.cols()=" << pointsA.cols() << std::endl;
  // std::cout << "pointsB_matrix.cols()=" << pointsB.cols() << std::endl;

  pm_->lp = glp_create_prob();
  glp_set_prob_name(pm_->lp, "separator");
  glp_set_obj_dir(pm_->lp, GLP_MAX);

  /* fill problem */
  glp_add_rows(pm_->lp, pointsA.cols() + pointsB.cols());
  int row = 1;

  // See here why we can use an epsilon of 1.0:
  // http://www.joyofdata.de/blog/testing-linear-separability-linear-programming-r-glpk/
  // This also allows you to avoid checking if norm(n)==0 at the end (see degenerateSolution commented out below)
  double epsilon = 1.0;

  // n (the solution) will point to the pointsA

  for (int i = 0; i < pointsA.cols(); i++)
  {
    // glp_set_row_name(lp, r, "p");
    glp_set_row_bnds(pm_->lp, row, GLP_LO, epsilon, 0.0);  // n'xA+d>=epsilon
    row++;
  }
  //  std::cout << "Using PointsB=" << std::endl;
  for (int i = 0; i < pointsB.cols(); i++)
  {
    // glp_set_row_name(lp, r, "p");
    glp_set_row_bnds(pm_->lp, row, GLP_UP, 0.0, -epsilon);  //<=0.0   n'xB+d <=-epsilon
    row++;
  }

  // glp_set_row_bnds(pm_->lp, row, GLP_UP, 0.0, 3.0);  // n1+n2+n3<=3 //To prevent unbounded solutions
  row++;

  ///
  glp_add_cols(pm_->lp, 4);

  // weights
  glp_set_col_name(pm_->lp, 1, "n1");
  glp_set_col_bnds(pm_->lp, 1, GLP_FR, 0.0, 0.0);  // Free
  glp_set_obj_coef(pm_->lp, 1, pm_->weight_n0);    // weight on n0

  glp_set_col_name(pm_->lp, 2, "n2");
  glp_set_col_bnds(pm_->lp, 2, GLP_FR, 0.0, 0.0);  // Free
  glp_set_obj_coef(pm_->lp, 2, pm_->weight_n1);    // weight on n1

  glp_set_col_name(pm_->lp, 3, "n3");
  glp_set_col_bnds(pm_->lp, 3, GLP_FR, 0.0, 0.0);  // Free
  glp_set_obj_coef(pm_->lp, 3, pm_->weight_n2);    // weight on n2

  glp_set_col_name(pm_->lp, 4, "d");
  glp_set_col_bnds(pm_->lp, 4, GLP_FR, 0.0, 0.0);  // Free
  glp_set_obj_coef(pm_->lp, 4, 0.0);               // weight on d

  int r = 1;
  row = 1;
  for (int i = 0; i < pointsA.cols(); i++)
  {
    pm_->ia[r] = row, pm_->ja[r] = 1, pm_->ar[r] = pointsA(0, i); /* a[1,1] = 1 */
    r++;
    pm_->ia[r] = row, pm_->ja[r] = 2, pm_->ar[r] = pointsA(1, i);  // a[1,2] = 2
    r++;
    pm_->ia[r] = row, pm_->ja[r] = 3, pm_->ar[r] = pointsA(2, i);  // a[1,3] = 2
    r++;
    pm_->ia[r] = row, pm_->ja[r] = 4, pm_->ar[r] = 1.0;  // a[1,4] = 1
    r++;
    row++;
  }

  for (int i = 0; i < pointsB.cols(); i++)
  {
    pm_->ia[r] = row, pm_->ja[r] = 1, pm_->ar[r] = pointsB(0, i);  // a[1,1] = 1
    r++;
    pm_->ia[r] = row, pm_->ja[r] = 2, pm_->ar[r] = pointsB(1, i);  // a[1,2] = 2
    r++;
    pm_->ia[r] = row, pm_->ja[r] = 3, pm_->ar[r] = pointsB(2, i);  // a[1,3] = 2
    r++;
    pm_->ia[r] = row, pm_->ja[r] = 4, pm_->ar[r] = 1.0;  // a[1,4] = 2
    r++;
    row++;
  }

  glp_load_matrix(pm_->lp, r - 1, pm_->ia, pm_->ja,
                  pm_->ar);  // need r-1 to substract from r++ in the last iteration of the previous loop
  // glp_write_lp(pm_->lp, NULL, "/home/jtorde/Desktop/ws/src/faster/faster/my_model.txt");

  /* solve problem */

  glp_simplex(pm_->lp, &pm_->params);

  /* recover and display results */
  double z = glp_get_obj_val(pm_->lp);
  solutionN(0) = glp_get_col_prim(pm_->lp, 1);
  solutionN(1) = glp_get_col_prim(pm_->lp, 2);
  solutionN(2) = glp_get_col_prim(pm_->lp, 3);
  solutionD = glp_get_col_prim(pm_->lp, 4);
  // std::cout << "solutionD= " << solutionD << std::endl;
  // printf("z = %g; n1 = %g; n2 = %g; n3 = %g\n", z, n1, n2, n3);

  int status = glp_get_status(pm_->lp);

  // std::cout << "status= " << status << std::endl;

  // /*  GLP_OPT — solution is optimal;
  //   GLP_FEAS — solution is feasible;
  //   GLP_INFEAS — solution is infeasible;
  //   GLP_NOFEAS — problem has no feasible solution;
  //   GLP_UNBND — problem has unbounded solution;
  //   GLP_UNDEF — solution is undefined.*/

  // switch (status)
  // {
  //   case GLP_OPT:
  //     std::cout << "status = GLP_OPT" << std::endl;
  //     break;
  //   case GLP_FEAS:
  //     std::cout << "status = GLP_FEAS" << std::endl;
  //     break;
  //   case GLP_INFEAS:
  //     std::cout << "status = GLP_INFEAS" << std::endl;
  //     break;
  //   case GLP_NOFEAS:
  //     std::cout << "status = GLP_NOFEAS" << std::endl;
  //     break;
  //   case GLP_UNBND:
  //     std::cout << "status = GLP_UNBND" << std::endl;
  //     break;
  //   case GLP_UNDEF:
  //     std::cout << "status = GLP_UNDEF" << std::endl;
  //     break;
  //   default:
  //     std::cout << "This code doesn't exist!!" << std::endl;
  // }

  /*  if ((status != GLP_OPT) && (status != GLP_FEAS))
    {
      glp_write_lp(pm_->lp, NULL, "/home/jtorde/Desktop/ws/src/faster/faster/my_model2.txt");
    }*/

  glp_delete_prob(pm_->lp);
  glp_free_env();

  // std::cout << "solutionN.norm()=" << solutionN.norm() << std::endl;

  // bool degenerateSolution = (solutionN.norm() < 0.000001);  // solution is [0 0 0]
  pm_->num_of_LPs_run++;  // Now pm_->num_of_LPs_run counts also the last LP run
  double total_time_us =
      (std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time))
          .count();

  // std::cout << "total_time_us LP =" << total_time_us << "us" << std::endl;
  // std::cout << "mean comp time LP =" << pm_->mean_comp_time_ms * 1000 << "us" << std::endl;
  // std::cout << "pm_->num_of_LPs_run LP =" << pm_->num_of_LPs_run << std::endl;

  // https://math.stackexchange.com/a/22351
  pm_->mean_comp_time_ms =
      pm_->mean_comp_time_ms + (total_time_us / 1e3 - pm_->mean_comp_time_ms) / pm_->num_of_LPs_run;

  if ((status == GLP_OPT || status == GLP_FEAS))  //&& !degenerateSolution
  {
    return true;
  }

  return false;
};

}  // namespace separator
