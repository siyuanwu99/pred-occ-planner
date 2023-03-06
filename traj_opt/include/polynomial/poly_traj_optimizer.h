#ifndef _POLY_TRAJ_OPTIMIZER_H_
#define _POLY_TRAJ_OPTIMIZER_H_

#include <path_searching/dyn_a_star.h>
#include <plan_env/grid_map.h>
#include <ros/ros.h>

#include <Eigen/Eigen>
#include "plan_container.hpp"

#include "lbfgs.hpp"
#include "polynomial/poly_traj_utils.hpp"

namespace ego_planner {

class ConstraintPoints {
 public:
  int             cp_size;  // deformation points
  Eigen::MatrixXd points;
  std::vector<std::vector<Eigen::Vector3d>> base_point;  // The point at the statrt of the direction vector (collision point)
  std::vector<std::vector<Eigen::Vector3d>> direction;  // Direction vector, must be normalized.
  std::vector<bool>
      flag_temp;  // A flag that used in many places. Initialize it everytime before using it.

  void resize_cp(const int size_set) {
    cp_size = size_set;

    base_point.clear();
    direction.clear();
    flag_temp.clear();

    points.resize(3, size_set);
    base_point.resize(cp_size);
    direction.resize(cp_size);
    flag_temp.resize(cp_size);
  }

  void segment(ConstraintPoints &buf, const int start, const int end) {
    if (start < 0 || end >= cp_size || points.rows() != 3) {
      ROS_ERROR("Wrong segment index! start=%d, end=%d", start, end);
      return;
    }

    buf.resize_cp(end - start + 1);
    buf.points  = points.block(0, start, 3, end - start + 1);
    buf.cp_size = end - start + 1;
    for (int i = start; i <= end; i++) {
      buf.base_point[i - start] = base_point[i];
      buf.direction[i - start]  = direction[i];
    }
  }

  static inline int two_thirds_id(Eigen::MatrixXd &points, const bool touch_goal) {
    return touch_goal ? points.cols() - 1 : points.cols() - 1 - (points.cols() - 2) / 3;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

class PolyTrajOptimizer {
 private:
  GridMap::Ptr          grid_map_;
  AStar::Ptr            a_star_;
  poly_traj::MinJerkOpt jerkOpt_;
  SwarmTrajData *       swarm_trajs_{NULL};  // Can not use shared_ptr and no need to free
  // ConstraintPoints cps_;
  // PtsChk_t pts_check_;

  std::vector<int> closeSet_;

  int    drone_id_;
  int    cps_num_prePiece_;  // number of distinctive constraint points each piece
  int    variable_num_;      // optimization variables
  int    piece_num_;         // poly traj piece numbers
  int    iter_num_;          // iteration of the solver
  double min_ellip_dist2_;   // min trajectory distance in swarm
  bool   touch_goal_;

  std::vector<int> piece_nums_;
  int              central_num_;

  enum FORCE_STOP_OPTIMIZE_TYPE { DONT_STOP, STOP_FOR_REBOUND, STOP_FOR_ERROR } force_stop_type_;

  /* optimization parameters */
  double wei_obs_, wei_obs_soft_;                                // obstacle weight
  double wei_swarm_;                                             // swarm weight
  double wei_feas_;                                              // feasibility weight
  double wei_sqrvar_;                                            // squared variance weight
  double wei_time_;                                              // time weight
  double obs_clearance_, obs_clearance_soft_, swarm_clearance_;  // safe distance
  double max_vel_, max_acc_;                                     // dynamic limits

  double t_now_;

 public:
  PolyTrajOptimizer() {}
  ~PolyTrajOptimizer() {}

  enum CHK_RET { OBS_FREE, ERR, FINISH };

  /* set variables */
  void setParam(ros::NodeHandle &nh);
  void setEnvironment(const GridMap::Ptr &map);
  void setControlPoints(const Eigen::MatrixXd &points);
  void setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr);
  void setDroneId(const int drone_id);
  void setIfTouchGoal(const bool touch_goal);
  void setConstraintPoints(ConstraintPoints cps);

  bool resizeTrajPara(int size) {
    central_num_ = size;
    jerkOpts_.clear();
    piece_nums_.clear();
    cpss_.clear();
    jerkOpts_.resize(size);
    piece_nums_.resize(size);
    cpss_.resize(size);
    return true;
  }

  /* helper functions */
  inline const ConstraintPoints &     getControlPoints(void) { return cps_; }
  inline const poly_traj::MinJerkOpt &getMinJerkOpt(void) { return jerkOpt_; }
  inline int                          get_cps_num_prePiece_(void) { return cps_num_prePiece_; }
  inline double                       get_swarm_clearance_(void) { return swarm_clearance_; }

  /* ecbs central optimization */
  bool ecbsOptTraj(const std::vector<Eigen::Matrix<double, 3, 3>> &iniStates,
                   const std::vector<Eigen::Matrix<double, 3, 3>> &finStates,
                   const std::vector<Eigen::MatrixXd> &            initInnerPtss,
                   const std::vector<Eigen::VectorXd> &            initTs,
                   std::vector<Eigen::MatrixXd> &                  optimal_points,
                   double &                                        final_cost,
                   const std::vector<int> &                        closeSet);

  /* main planning API */
  bool optimizeTrajectory(const Eigen::MatrixXd &iniState,
                          const Eigen::MatrixXd &finState,
                          const Eigen::MatrixXd &initInnerPts,
                          const Eigen::VectorXd &initT,
                          Eigen::MatrixXd &      optimal_points,
                          double &               final_cost);

  bool computePointsToCheck(poly_traj::Trajectory &traj, int id_end, PtsChk_t &pts_check);

  std::vector<std::pair<int, int>> finelyCheckConstraintPointsOnly(Eigen::MatrixXd &init_points);

  CHK_RET finelyCheckAndSetConstraintPoints(ConstraintPoints &                cps,
                                            std::vector<std::pair<int, int>> &segments,
                                            const poly_traj::MinJerkOpt &     pt_data,
                                            const bool flag_first_init /*= true*/);

  bool roughlyCheckConstraintPoints(ConstraintPoints &cps);

  /* multi-topo support */
  std::vector<ConstraintPoints> distinctiveTrajs(vector<std::pair<int, int>> segments);

  double computePerturedCost(Eigen::VectorXd &x_var);
  double computePerturedSmoothCost(Eigen::VectorXd &                   x_var,
                                   std::vector<poly_traj::MinJerkOpt> &jerkOpts_pert);
  // double computePerturedCost(Eigen::VectorXd &x_var, std::vector<poly_traj::MinJerkOpt>
  // &jerkOpts_pert, std::vector<ConstraintPoints> &cpss_pert);

 private:
  /* ECBS callbacks by the L-BFGS optimizer */
  static double ecbsCostFuncCallback(void *func_data, const double *x, double *grad, const int n);

  /* callbacks by the L-BFGS optimizer */
  static double costFunctionCallback(void *func_data, const double *x, double *grad, const int n);

  static int earlyExitCallback(void *        func_data,
                               const double *x,
                               const double *g,
                               const double  fx,
                               const double  xnorm,
                               const double  gnorm,
                               const double  step,
                               int           n,
                               int           k,
                               int           ls);

  /* mappings between real world time and unconstrained virtual time */
  template <typename EIGENVEC>
  void RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT);

  template <typename EIGENVEC>
  void VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT);

  template <typename EIGENVEC, typename EIGENVECGD>
  void VirtualTGradCost(const Eigen::VectorXd &RT,
                        const EIGENVEC &       VT,
                        const Eigen::VectorXd &gdRT,
                        EIGENVECGD &           gdVT,
                        double &               costT);

  /* gradient and cost evaluation functions */
  template <typename EIGENVEC>
  void initAndGetSmoothnessGradCost2PT(poly_traj::MinJerkOpt &jerkOpt, EIGENVEC &gdT, double &cost);

  template <typename EIGENVEC>
  void addPVAGradCost2CT(poly_traj::MinJerkOpt &jerkOpt,
                         ConstraintPoints &     cps,
                         EIGENVEC &             gdT,
                         Eigen::VectorXd &      costs,
                         const int &            K);

  template <typename EIGENVEC>
  void cenAddPVAGradCost2CT(int                           id,
                            std::vector<Eigen::VectorXd> &Ts,
                            ConstraintPoints &            cps,
                            std::vector<EIGENVEC> &       gradTs,
                            Eigen::VectorXd &             costs,
                            const int &                   K);

  bool obstacleGradCostP(ConstraintPoints &     cps,
                         const int              i_dp,
                         const Eigen::Vector3d &p,
                         Eigen::Vector3d &      gradp,
                         double &               costp);

  bool swarmGradCostP(ConstraintPoints &     cps,
                      const int              i_dp,
                      const double           t,
                      const Eigen::Vector3d &p,
                      const Eigen::Vector3d &v,
                      Eigen::Vector3d &      gradp,
                      double &               gradt,
                      double &               grad_prev_t,
                      double &               costp);

  bool cenSwarmGradCostP(int                           i_cur,
                         double &                      omg,
                         double &                      step,
                         std::vector<Eigen::VectorXd> &gradTs,
                         std::vector<Eigen::VectorXd> &Ts,
                         ConstraintPoints &            cps,
                         const int                     i_dp,
                         const double                  t,
                         const Eigen::Vector3d &       p,
                         const Eigen::Vector3d &       v,
                         Eigen::Vector3d &             gradp,
                         double &                      gradt,
                         double &                      grad_prev_t,
                         double &                      costp);

  bool feasibilityGradCostV(const Eigen::Vector3d &v, Eigen::Vector3d &gradv, double &costv);

  bool feasibilityGradCostA(const Eigen::Vector3d &a, Eigen::Vector3d &grada, double &costa);

  void distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                         Eigen::MatrixXd &      gdp,
                                         double &               var);

  void lengthVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                    const int              n,
                                    Eigen::MatrixXd &      gdp,
                                    double &               var);

 public:
  ConstraintPoints                      cps_;
  std::vector<ConstraintPoints>         cpss_;
  std::vector<poly_traj::MinJerkOpt>    jerkOpts_;
  typedef unique_ptr<PolyTrajOptimizer> Ptr;
};

}  // namespace ego_planner
#endif