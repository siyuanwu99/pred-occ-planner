#ifndef _BSPLINE_OPTIMIZER_H_
#define _BSPLINE_OPTIMIZER_H_

#include <bspline_opt/uniform_bspline.h>
#include <path_searching/dyn_a_star.h>
#include <ros/ros.h>

#include <Eigen/Eigen>
#include <traj_opt/plan_container.hpp>

#include "bspline_opt/lbfgs.hpp"

// Gradient and elasitc band optimization

// Input: a signed distance field and a sequence of points
// Output: the optimized sequence of points
// The format of points: N x 3 matrix, each row is a point
namespace traj_opt {

class ControlPoints {
 public:
  double          clearance;
  int             size;
  Eigen::MatrixXd points;

  /* The point at the statrt of the direction vector (collision point) */
  std::vector<std::vector<Eigen::Vector3d>> base_point;
  /* The direction vector, must be normalized*/
  std::vector<std::vector<Eigen::Vector3d>> direction;
  /* A flag that used in many places. Initialize it everytime before using it. */
  std::vector<bool> flag_temp;
  // std::vector<bool> occupancy;

  void resize(const int size_set) {
    size = size_set;

    base_point.clear();
    direction.clear();
    flag_temp.clear();
    // occupancy.clear();

    points.resize(3, size_set);
    base_point.resize(size);
    direction.resize(size);
    flag_temp.resize(size);
    // occupancy.resize(size);
  }

  void segment(ControlPoints& buf, const int start, const int end) {
    if (start < 0 || end >= size || points.rows() != 3) {
      ROS_ERROR("Wrong segment index! start=%d, end=%d", start, end);
      return;
    }

    buf.resize(end - start + 1);
    buf.points    = points.block(0, start, 3, end - start + 1);
    buf.clearance = clearance;
    buf.size      = end - start + 1;
    for (int i = start; i <= end; i++) {
      buf.base_point[i - start] = base_point[i];
      buf.direction[i - start]  = direction[i];

      // if ( buf.base_point[i - start].size() > 1 )
      // {
      //   ROS_ERROR("buf.base_point[i - start].size()=%d, base_point[i].size()=%d",
      //   buf.base_point[i - start].size(), base_point[i].size());
      // }
    }

    // cout << "RichInfoOneSeg_temp, insede" << endl;
    // for ( int k=0; k<buf.size; k++ )
    //   if ( buf.base_point[k].size() > 0 )
    //   {
    //     cout << "###" << buf.points.col(k).transpose() << endl;
    //     for (int k2 = 0; k2 < buf.base_point[k].size(); k2++)
    //     {
    //       cout << "      " << buf.base_point[k][k2].transpose() << " @ " <<
    //       buf.direction[k][k2].transpose() << endl;
    //     }
    //   }
  }
};

/**
 * @brief B spline optimizer with hard constraints
 */
ConstrainedBsplineOpt {
 public:
  ConstrainedBsplineOpt() {}
  ~ConstrainedBsplineOpt() {}

  /* main API */
  void            setParam(ros::NodeHandle & nh);
  void            setConstraints(const std::vector<Eigen::MatrixXd>& constraints);
  Eigen::MatrixXd BsplineOptimizeTraj(const Eigen::MatrixXd& points, const double& ts,
                                      const int& cost_function, int max_num_id, int max_time_id);

  /* required inputs */
  void setControlPoints(const Eigen::MatrixXd& points);
  void setBsplineInterval(const double& ts);
  void setDroneId(const int drone_id) { _drone_id = drone_id; }

  /* optional inputs */
  void setGuidePath(const vector<Eigen::Vector3d>& guide_pt);
  void setWaypoints(const vector<Eigen::Vector3d>& waypts,
                    const vector<int>&             waypt_idx);  // N-2 constraints at most
  void setLocalTargetPt(const Eigen::Vector3d local_target_pt) {
    local_target_pt_ = local_target_pt;
  }
  inline int    getOrder(void) { return order_; }
  ControlPoints getControlPoints() { return cps_; }

  void optimize();

  std::vector<Eigen::Vector3d> ref_pts_;

  std::vector<ControlPoints>       distinctiveTrajs(vector<std::pair<int, int>> segments);
  std::vector<std::pair<int, int>> initControlPoints(Eigen::MatrixXd & init_points,
                                                     bool flag_first_init = true);

  bool BsplineOptimizeTrajRebound(Eigen::MatrixXd & optimal_points,
                                  double ts);  // must be called after initControlPoints()
  bool BsplineOptimizeTrajRebound(Eigen::MatrixXd & optimal_points, double& final_cost,
                                  const ControlPoints& control_points, double ts);
  bool BsplineOptimizeTrajRefine(const Eigen::MatrixXd& init_points, const double ts,
                                 Eigen::MatrixXd& optimal_points);

 private:
  int drone_id_;
  enum FORCE_STOP_OPTIMIZE_TYPE { DONT_STOP, STOP_FOR_REBOUND, STOP_FOR_ERROR } force_stop_type_;
  ControlPoints   cps_;               // control points of the bspline
  double          bspline_interval_;  // B-spline knot span
  Eigen::Vector3d end_pt_;            // end of the trajectory
  int             dim_;               // dimension of the B-spline

  std::vector<Eigen::Vector3d> guide_pts_;  // geometric guiding path points, N-6
  std::vector<int>             waypt_idx_;  // waypoints constraints index
  std::vector<Eigen::Vector3d> waypoints_;  // waypoints constraints
                                            //
  int    max_num_id_, max_time_id_;         // stopping criteria
  int    cost_function_;                    // used to determine objective function
  double start_time_;                       // global time for moving obstacles

  /* optimization parameters */
  int    order_;                  // bspline degree
  double lambda1_;                // jerk smoothness weight
  double lambda2_, new_lambda2_;  // distance weight
  double lambda3_;                // feasibility weight
  double lambda4_;                // curve fitting

  int a;
  //
  double dist0_, swarm_clearance_;  // safe distance
  double max_vel_, max_acc_;        // dynamic limits

  int             variable_num_;   // optimization variables
  int             iter_num_;       // iteration of the solver
  Eigen::VectorXd best_variable_;  //
  double          min_cost_;       //

  Eigen::Vector3d local_target_pt_;

  /* cost function */
  /* calculate each part of cost function with control points q as input */

  static double costFunction(const std::vector<double>& x, std::vector<double>& grad,
                             void* func_data);
  void          combineCost(const std::vector<double>& x, vector<double>& grad, double& cost);

  // q contains all control points
  void calcSmoothnessCost(const Eigen::MatrixXd& q, double& cost, Eigen::MatrixXd& gradient,
                          bool falg_use_jerk = true);
  void calcFeasibilityCost(const Eigen::MatrixXd& q, double& cost, Eigen::MatrixXd& gradient);
  void calcTerminalCost(const Eigen::MatrixXd& q, double& cost, Eigen::MatrixXd& gradient);
  void calcDistanceCostRebound(const Eigen::MatrixXd& q, double& cost, Eigen::MatrixXd& gradient,
                               int iter_num, double smoothness_cost);
  void calcFitnessCost(const Eigen::MatrixXd& q, double& cost, Eigen::MatrixXd& gradient);
  bool check_collision_and_rebound(void);
  static int    earlyExit(void* func_data, const double* x, const double* g, const double fx,
                          const double xnorm, const double gnorm, const double step, int n, int k,
                          int ls);
  static double costFunctionRebound(void* func_data, const double* x, double* grad, const int n);
  static double costFunctionRefine(void* func_data, const double* x, double* grad, const int n);

 public:
  typedef std::unique_ptr<ConstrainedBsplineOpt> Ptr;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}

class BsplineOptimizer {
 public:
  static const int SMOOTHNESS;
  static const int DISTANCE;
  static const int FEASIBILITY;
  static const int ENDPOINT;
  static const int GUIDE;
  static const int WAYPOINTS;

  static const int GUIDE_PHASE;
  static const int NORMAL_PHASE;

  BsplineOptimizer() {}
  ~BsplineOptimizer() {}

  /* main API */
  void            setEnvironment(const EDTEnvironment::Ptr& env);
  void            setParam(ros::NodeHandle& nh);
  Eigen::MatrixXd BsplineOptimizeTraj(const Eigen::MatrixXd& points,
                                      const double&          ts,
                                      const int&             cost_function,
                                      int                    max_num_id,
                                      int                    max_time_id);

  /* helper function */

  // required inputs
  void setControlPoints(const Eigen::MatrixXd& points);
  void setBsplineInterval(const double& ts);
  void setCostFunction(const int& cost_function);
  void setTerminateCond(const int& max_num_id, const int& max_time_id);

  // optional inputs
  void setGuidePath(const vector<Eigen::Vector3d>& guide_pt);
  void setWaypoints(const vector<Eigen::Vector3d>& waypts,
                    const vector<int>&             waypt_idx);  // N-2 constraints at most

  void optimize();

  Eigen::MatrixXd         getControlPoints();
  vector<Eigen::Vector3d> matrixToVectors(const Eigen::MatrixXd& ctrl_pts);

 private:
  EDTEnvironment::Ptr edt_environment_;

  // main input
  Eigen::MatrixXd control_points_;     // B-spline control points, N x dim
  double          bspline_interval_;   // B-spline knot span
  Eigen::Vector3d end_pt_;             // end of the trajectory
  int             dim_;                // dimension of the B-spline
                                       //
  vector<Eigen::Vector3d> guide_pts_;  // geometric guiding path points, N-6
  vector<Eigen::Vector3d> waypoints_;  // waypts constraints
  vector<int>             waypt_idx_;  // waypts constraints index
                                       //
  int    max_num_id_, max_time_id_;    // stopping criteria
  int    cost_function_;               // used to determine objective function
  bool   dynamic_;                     // moving obstacles ?
  double start_time_;                  // global time for moving obstacles

  /* optimization parameters */
  int    order_;                  // bspline degree
  double lambda1_;                // jerk smoothness weight
  double lambda2_;                // distance weight
  double lambda3_;                // feasibility weight
  double lambda4_;                // end point weight
  double lambda5_;                // guide cost weight
  double lambda6_;                // visibility cost weight
  double lambda7_;                // waypoints cost weight
  double lambda8_;                // acc smoothness
                                  //
  double dist0_;                  // safe distance
  double max_vel_, max_acc_;      // dynamic limits
  double visib_min_;              // threshold of visibility
  double wnl_;                    //
  double dlmin_;                  //
                                  //
  int    algorithm1_;             // optimization algorithms for quadratic cost
  int    algorithm2_;             // optimization algorithms for general cost
  int    max_iteration_num_[4];   // stopping criteria that can be used
  double max_iteration_time_[4];  // stopping criteria that can be used

  /* intermediate variables */
  /* buffer for gradient of cost function, to avoid repeated allocation and
   * release of memory */
  vector<Eigen::Vector3d> g_q_;
  vector<Eigen::Vector3d> g_smoothness_;
  vector<Eigen::Vector3d> g_distance_;
  vector<Eigen::Vector3d> g_feasibility_;
  vector<Eigen::Vector3d> g_endpoint_;
  vector<Eigen::Vector3d> g_guide_;
  vector<Eigen::Vector3d> g_waypoints_;

  int                 variable_num_;   // optimization variables
  int                 iter_num_;       // iteration of the solver
  std::vector<double> best_variable_;  //
  double              min_cost_;       //

  vector<Eigen::Vector3d> block_pts_;  // blocking points to compute visibility

  /* cost function */
  /* calculate each part of cost function with control points q as input */

  static double costFunction(const std::vector<double>& x,
                             std::vector<double>&       grad,
                             void*                      func_data);
  void          combineCost(const std::vector<double>& x, vector<double>& grad, double& cost);

  // q contains all control points
  void calcSmoothnessCost(const vector<Eigen::Vector3d>& q,
                          double&                        cost,
                          vector<Eigen::Vector3d>&       gradient);
  void calcDistanceCost(const vector<Eigen::Vector3d>& q,
                        double&                        cost,
                        vector<Eigen::Vector3d>&       gradient);
  void calcFeasibilityCost(const vector<Eigen::Vector3d>& q,
                           double&                        cost,
                           vector<Eigen::Vector3d>&       gradient);
  void calcEndpointCost(const vector<Eigen::Vector3d>& q,
                        double&                        cost,
                        vector<Eigen::Vector3d>&       gradient);
  void calcGuideCost(const vector<Eigen::Vector3d>& q,
                     double&                        cost,
                     vector<Eigen::Vector3d>&       gradient);
  void calcVisibilityCost(const vector<Eigen::Vector3d>& q,
                          double&                        cost,
                          vector<Eigen::Vector3d>&       gradient);
  void calcWaypointsCost(const vector<Eigen::Vector3d>& q,
                         double&                        cost,
                         vector<Eigen::Vector3d>&       gradient);
  void calcViewCost(const vector<Eigen::Vector3d>& q,
                    double&                        cost,
                    vector<Eigen::Vector3d>&       gradient);
  bool isQuadratic();

  /* for benckmark evaluation only */
 public:
  vector<double> vec_cost_;
  vector<double> vec_time_;
  ros::Time      time_start_;

  void getCostCurve(vector<double>& cost, vector<double>& time) {
    cost = vec_cost_;
    time = vec_time_;
  }

  typedef unique_ptr<BsplineOptimizer> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace traj_opt
#endif