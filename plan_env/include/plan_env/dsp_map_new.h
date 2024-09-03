/**
 * @file dsp_map_new.h
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-10-04
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef _DSP_MAP_H_
#define _DSP_MAP_H_

#include <cmath>
#include <ctime>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <thread>

#include <Eigen/Eigen>

#include "munkres.h"

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

/* -------------- Parameters for the map ------------------ */
#define MAP_LENGTH_VOXEL_NUM   65  // 33//29  //odd
#define MAP_WIDTH_VOXEL_NUM    65  // 33//29   //odd
#define MAP_HEIGHT_VOXEL_NUM   35  // 9 //21, 9 //odd
#define ANGLE_RESOLUTION       3
#define MAX_PARTICLE_NUM_VOXEL 18  // 18 //80
#define VOXEL_RESOLUTION       0.15

/// Note: RISK_MAP_NUMBER * RISK_MAP_PREDICTION_TIME = PREDICTION_TIMES. RISK_MAP_PREDICTION_TIMES
/// items in _prediction_future_time should be within time a_star_search_time_step
#define PREDICTION_TIMES          9
#define RISK_MAP_PREDICTION_TIMES 3

static const float _prediction_future_time[PREDICTION_TIMES] = {
    0.1f, 0.3f, 0.5f, 0.7f, 0.9f, 1.1f, 1.3f, 1.5f, 1.8f};  // unit: second
// static const float _prediction_future_time[PREDICTION_TIMES] = {0.1f, 0.2f, 0.4f, 0.6f,
// 0.8f, 1.f, 1.2f, 1.4f, 1.6f, 1.8f, 2.f}; //unit: second

const int half_fov_h = 48;  // can be divided by ANGLE_RESOLUTION. If not, modify ANGLE_RESOLUTION
                            // or make half_fov_h a smaller value than the real FOV angle
const int half_fov_v = 36;  // can be divided by ANGLE_RESOLUTION. If not, modify ANGLE_RESOLUTION
                            // or make half_fov_h a smaller value than the real FOV angle

/*--------------- END ---------------*/

static const int VOXEL_NUM   = MAP_LENGTH_VOXEL_NUM * MAP_WIDTH_VOXEL_NUM * MAP_HEIGHT_VOXEL_NUM;
static const int PYRAMID_NUM = 360 * 180 / ANGLE_RESOLUTION / ANGLE_RESOLUTION;

static const int observation_pyramid_num_h = (int)half_fov_h * 2 / ANGLE_RESOLUTION;
static const int observation_pyramid_num_v = (int)half_fov_v * 2 / ANGLE_RESOLUTION;
static const int observation_pyramid_num   = observation_pyramid_num_h * observation_pyramid_num_v;

static const int SAFE_PARTICLE_NUM         = VOXEL_NUM * MAX_PARTICLE_NUM_VOXEL + 1e5;
static const int SAFE_PARTICLE_NUM_VOXEL   = MAX_PARTICLE_NUM_VOXEL * 2;
static const int SAFE_PARTICLE_NUM_PYRAMID = SAFE_PARTICLE_NUM / PYRAMID_NUM * 2;

// An estimated number. If the observation points are too dense (over 100 points in one pyramid),
// the overflowed points will be ignored. It is suggested to use a voxel filter to the original
// point cloud.
static const int   observation_max_points_num_one_pyramid = 100;
static const float obstacle_thickness_for_occlusion       = 0.3;

// int velocity_estimation_error_occurred = 0;

#define GAUSSIAN_RANDOMS_NUM 1000000

#define O_MAKE_VALID   1  // use |= operator
#define O_MAKE_INVALID 0  // use &= operator

using namespace std;

namespace dsp_map {

// flag value 0: invalid, value 1: valid but not newborn, value 3: valid newborn 7: Recently
// predicted
/// Container for voxels h particles
//  1.flag 2.vx 3.vy 4.vz 5.px 6.py 7.pz
//  8.weight 9.update time
static float _voxels_with_particle[VOXEL_NUM][SAFE_PARTICLE_NUM_VOXEL][9];

// 1. objects number; 2-4. Avg vx, vy, vz; 5-. Future objects number
static const int _voxels_objects_number_dimension = 4 + PREDICTION_TIMES;
static float     _voxels_objects_number[VOXEL_NUM][_voxels_objects_number_dimension];

/// Container for pyramids
// 0.flag 1.particle voxel index  2.particle index inside voxel
static int _pyramids_in_fov[observation_pyramid_num][SAFE_PARTICLE_NUM_PYRAMID][3];

// 1.neighbors num 2-10:neighbor indexes
static int _observation_pyramid_neighbors[observation_pyramid_num][10]{};

/// Variables for velocity estimation
// pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud_in_current_view_rotated(
//     new pcl::PointCloud<pcl::PointXYZ>());

static float _current_position[3]          = {0.f, 0.f, 0.f};
static float _voxel_filtered_resolution    = 0.15;
static float _delt_t_from_last_observation = 0.f;

// pcl::PointCloud<pcl::PointXYZINormal>::Ptr _input_cloud_with_velocity(
//     new pcl::PointCloud<pcl::PointXYZINormal>());

/** Storage for Gaussian randoms and Gaussian PDF**/
static float _standard_gaussian_pdf[20000];

/** Struct for an individual particle**/
struct Particle {
  float px;
  float py;
  float pz;
  float vx;
  float vy;
  float vz;
  float weight;
  int   voxel_index;
};

struct ClusterFeature {
  float center_x          = 0.f;
  float center_y          = 0.f;
  float center_z          = 0.f;
  int   point_num         = 0;
  int   match_cluster_seq = -1;
  float vx                = -10000.f;
  float vy                = -10000.f;
  float vz                = -10000.f;
  float v                 = 0.f;
  float intensity         = 0.f;
};

struct MappingParameters {
  /* map properties */
  int   n_risk_map_;                /* number of risk maps */
  int   n_prediction_per_risk_map_; /* number of predictions per risk map */
  int   voxel_size_x_, voxel_size_y_, voxel_size_z_;
  float map_size_x_, map_size_y_, map_size_z_;
  float half_map_size_x_, half_map_size_y_, half_map_size_z_;
  float resolution_, resolution_inv_;

  int n_voxel_;   /* number of voxels */
  int n_pyramid_; /* number of pyramids */
  int n_pyramid_obsrv_h_, n_pyramid_obsrv_v_, n_pyramid_obsrv_;

  Eigen::Vector3f map_origin_;
  Eigen::Vector3f map_min_boundary_, map_max_boundary_;  // map range in pos
  Eigen::Vector3f local_update_range_;
  float           obstacles_inflation_;
  string          frame_id_;

  /* field of view */
  int   angle_resolution_;
  int   half_fov_h_, half_fov_v_; /* degree, should be devide by angle_resolution_ */
  float angle_resolution_rad_;

  /* time out */
  float odom_depth_timeout_;

  /* local map update and clear */
  int local_map_margin_;

  /* visualization and computation time display */
  float visualization_truncate_height_, virtual_ceil_height_, ground_height_, virtual_ceil_yp_,
      virtual_ceil_yn_;
  bool show_occ_time_;

  /* active mapping */
  float unknown_flag_;

  /* particles */
  int n_particles_max_;
  int n_particles_max_per_voxel_;
  int n_particles_max_per_pyramid_;

  /* new born particles */
  int   newborn_particles_per_point_;
  float newborn_particles_weight_;
  float newborn_objects_weight_;
  float newborn_objects_expected_;

  /* other mapping variables */
  float kappa_;
  float sigma_update_;
  float sigma_obsrv_, sigma_loc_;                 /* observation & localization */
  float stddev_pos_predict_, stddev_vel_predict_; /* prediction variance */

  /* others */
  bool is_csv_output_; /* output particles as csv file or not */
};

struct MappingData {
  /** main map data, point cloud and corresponding value in pyramid representation
   * point_cloud[index of pyramid][index of point in pyramid]
   */
  std::vector<std::vector<Eigen::Vector3f>> point_cloud_;
  std::vector<std::vector<float>>           point_cloud_val_;
  std::vector<float> max_length_each_pyramid_; /* max length of each pyramid */
  std::vector<float> n_obs_each_pyramid_;      /* points in each pyramid */

  /* gaussian random buffers */
  std::vector<float> pos_gaussian_randoms_;
  std::vector<float> vel_gaussian_randoms_;
  std::vector<float> loc_gaussian_randoms_; /* localization */

  /* camera position and pose data */
  Eigen::Vector3f    camera_pos_, last_camera_pos_;
  Eigen::Quaternionf camera_pose_, last_camera_pose_;
  Eigen::Matrix4d    cam2body_;

  /* flags of map state */
  bool occ_need_update_, local_updated_;
  bool has_first_depth_;
  bool has_odom_, has_cloud_;

  /* odom_depth_timeout_ */
  ros::Time last_occ_update_time_;
  bool      flag_depth_odom_timeout_;
  bool      flag_use_depth_fusion;

  /* range of updating grid map */
  Eigen::Vector3i local_bound_min_, local_bound_max_;

  /* computation time */
  double total_time_;
  float  update_time_;
  float  record_time_;
  int    idx_update_;  // number of updates

  /* index */
  int vel_gaussian_idx_;
  int pos_gaussian_idx_;
  int loc_gaussian_idx_;  // localization

  float P_detection_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class DSPMapStaticV2 {
 private:
  /** ROS **/
  ros::NodeHandle nh_;

  MappingParameters mp_; /** Map Parameters */
  MappingData       md_; /** Map Data */

  /* TODO: merge to md_ */
  pcl::PointCloud<pcl::PointXYZ>::Ptr        _cloud_in_current_view_rotated;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr _input_cloud_with_velocity;

  /** Parameters **/
  float record_time;

  // 1.px, 2.py, 3.pz 4.acc 5.length for later usage
  // float point_cloud_[observation_pyramid_num][observation_max_points_num_one_pyramid][5];

  // 1.point_num
  // int observation_num_each_pyramid[observation_pyramid_num]{};

  // Normal vectors for pyramids boundary planes when sensor has no rotation
  float pyramid_BPnorm_params_ori_h[observation_pyramid_num_h + 1][3];  // x, y, z
  float pyramid_BPnorm_params_ori_v[observation_pyramid_num_v + 1][3];

  // Normal vectors for pyramids boundary planes when sensor rotated
  float pyramid_BPnorm_params_h[observation_pyramid_num_h + 1][3];
  float pyramid_BPnorm_params_v[observation_pyramid_num_v + 1][3];

  // Max length, used to judge if occlusion happens
  // float point_cloud_max_length[observation_pyramid_num];

  double       total_time;
  unsigned int update_times;

 public:
  typedef std::shared_ptr<DSPMapStaticV2> Ptr;
  DSPMapStaticV2(int init_particle_num = 0, float init_weight = 0.01f) {
    addRandomParticles(init_particle_num, init_weight);

    std::cout << "Map is ready to update!" << endl;
  }

  ~DSPMapStaticV2() { std::cout << "\n See you ;)" << endl; }

  void initMap(MappingParameters mp);

  int update(int                      point_cloud_num,
             int                      size_of_one_point,
             float *                  point_cloud_ptr,
             const Eigen::Vector3f    sensor_pos,
             const Eigen::Quaternionf sensor_quaternion,
             double time_stamp_second);  /// also requires point cloud and velocity

  void setPredictionVariance(float p_stddev, float v_stddev);
  void setObservationStdDev(float ob_stddev);

  void setLocalizationStdDev(float lo_stddev);

  void setNewBornParticleWeight(float weight) { mp_.newborn_particles_weight_ = weight; }

  void setNewBornParticleNumberofEachPoint(int num) { mp_.newborn_particles_per_point_ = num; }

  /// record_particle_flag O: don't record; -1 or other negative value: record all; positive value:
  /// record a time
  void setParticleRecordFlag(bool record_particle_flag, float record_csv_time = 1.f);

  static void setOriginalVoxelFilterResolution(float res) { _voxel_filtered_resolution = res; }
  inline void getMapCenter(Eigen::Vector3f &center) { center = md_.camera_pos_; }
  inline void getMapSize(Eigen::Vector3f &size) {
    size << mp_.voxel_size_x_, mp_.voxel_size_y_, mp_.voxel_size_z_;
  }
  void getOccupancyMap(int &                           obstacles_num,
                       pcl::PointCloud<pcl::PointXYZ> &cloud,
                       const float                     threshold = 0.7);

  void getOccupancyMapWithVelocity(int &                              obstacles_num,
                                   std::vector<float> &               weights,
                                   pcl::PointCloud<pcl::PointNormal> &cloud,
                                   const float                        threshold = 0.7);

  void getOccupancyMapWithFutureStatus(int &                           obstacles_num,
                                       pcl::PointCloud<pcl::PointXYZ> &cloud,
                                       float *                         future_status,
                                       const float                     threshold = 0.7);

  void getOccupancyMapWithRiskMaps(int &                           obstacles_num,
                                   pcl::PointCloud<pcl::PointXYZ> &cloud,
                                   float *                         risk_maps,
                                   const float                     threshold = 0.7);
  void getObstaclePoints(int &                         obstacles_num,
                         std::vector<Eigen::Vector3d> &points,
                         const float                   threshold,
                         const float                   clearance = 0.5);
  /// NOTE: If you don't want to use any visualization functions like "getOccupancyMap"
  ///      or "getOccupancyMapWithVelocity", you must call this function after update process.
  void clearOccupancyMapPrediction();

  /// Get clustered result for visualization
  void getKMClusterResult(pcl::PointCloud<pcl::PointXYZINormal> &cluster_cloud);
  void mapAddNewBornParticlesByObservation();

  inline bool isInMap(const Particle &p) const;
  inline bool isInMap(const float &px, const float &py, const float &pz) const;
  inline bool ifInPyramidsArea(float &x, float &y, float &z);

 private:
  void setInitParameters();

  void addRandomParticles(int particle_num, float avg_weight);
  void mapPrediction(const Eigen::Vector3f &dp, float dt);

  void mapUpdate();

 private: /*** Some specific functions ***/
  void mapOccupancyCalculationAndResample();
  int  getParticleVoxelsIndex(const Particle &p, int &index);

  int getParticleVoxelsIndex(const float &px, const float &py, const float &pz, int &index);

  void getVoxelPositionFromIndex(const int &index, float &px, float &py, float &pz) const;

  inline void rotateVectorByQuaternion(const float *             ori_vector,
                                       const Eigen::Quaternionf &q,
                                       float *                   rotated_vector);

  inline float vectorMultiply(float &x1, float &y1, float &z1, float &x2, float &y2, float &z2);

  static void findPyramidNeighborIndexInFOV(const int &index_ori,
                                            int &      neighbor_spaces_num,
                                            int *      neighbor_spaces_index) {
    int h_index_ori = index_ori / observation_pyramid_num_v;
    int v_index_ori = index_ori % observation_pyramid_num_v;

    neighbor_spaces_num = 0;

    for (int i = -1; i <= 1; ++i) {
      for (int j = -1; j <= 1; ++j) {
        int h = h_index_ori + i;
        int v = v_index_ori + j;
        if (h >= 0 && h < observation_pyramid_num_h && v >= 0 && v < observation_pyramid_num_v) {
          *(neighbor_spaces_index + neighbor_spaces_num) = h * observation_pyramid_num_v + v;
          ++neighbor_spaces_num;
        }
      }
    }
  }

  void  generateGaussianRandomsVectorZeroCenter();
  float getPositionGaussianZeroCenter();
  float getLocalizationGaussianZeroCenter();
  float getVelocityGaussianZeroCenter();
  /***
   * Return 0 if move operation fails, otherwise return 1.
   * **/
  int addAParticle(const Particle &p, const int &voxel_index) const;

  /***
   * Return 0 if move operation fails, otherwise return 1.
   * **/
  int moveParticle(const int &new_voxel_index,
                   const int &current_v_index,
                   const int &current_v_inner_index,
                   float *    ori_particle_flag_ptr);

  static void removeParticle(float *ori_particle_flag_ptr) { *ori_particle_flag_ptr = 0.f; }

  static float standardNormalPDF(float value) {
    float fx = (1.f / (sqrtf(2.f * M_PI_2f32))) * expf(-powf(value, 2) / (2));
    return fx;
  }

  static void calculateNormalPDFBuffer() {
    for (int i = 0; i < 20000; ++i) {
      _standard_gaussian_pdf[i] =
          standardNormalPDF((float)(i - 10000) * 0.001f);  // range[-10, 10]; 10 sigma
    }
  }

  static float queryNormalPDF(float &x, float &mu, float &sigma) {
    float corrected_x = (x - mu) / sigma;
    if (corrected_x > 9.9f)
      corrected_x = 9.9f;
    else if (corrected_x < -9.9f)
      corrected_x = -9.9f;

    return _standard_gaussian_pdf[(int)(corrected_x * 1000 + 10000)];
  }

  //    static void vectorCOut(float *a){
  //        cout<<"("<< *a <<","<< *(a+1)<<","<< *(a+2)<<") ";
  //    }

  int findPointPyramidHorizontalIndex(float &x, float &y, float &z);
  int findPointPyramidVerticalIndex(float &x, float &y, float &z);

  static float clusterDistance(ClusterFeature &c1, ClusterFeature &c2) {
    float square_distance = (c1.center_x - c2.center_x) * (c1.center_x - c2.center_x) +
                            (c1.center_y - c2.center_y) * (c1.center_y - c2.center_y) +
                            (c1.center_z - c2.center_z) * (c1.center_z - c2.center_z);
    return sqrtf(square_distance);
  }

  void velocityEstimationThread();

  /*** For test ***/
 public:
  static float generateRandomFloat(float min, float max) {
    return min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
  }

  void getVoxelPositionFromIndexPublic(const int &index, float &px, float &py, float &pz) const;

  bool getPointVoxelsIndexPublic(const float &px, const float &py, const float &pz, int &index);
};

/* ----- Definition of Inline Functions ----- */

inline bool DSPMapStaticV2::isInMap(const Particle &p) const {
  return (p.px >= mp_.half_map_size_x_ || p.px <= -mp_.half_map_size_x_ ||
          p.py >= mp_.half_map_size_y_ || p.py <= -mp_.half_map_size_y_ ||
          p.pz >= mp_.half_map_size_z_ || p.pz <= -mp_.half_map_size_z_)
             ? false
             : true;
}

inline bool DSPMapStaticV2::isInMap(const float &px, const float &py, const float &pz) const {
  return (px >= mp_.half_map_size_x_ || px <= -mp_.half_map_size_x_ || py >= mp_.half_map_size_y_ ||
          py <= -mp_.half_map_size_y_ || pz >= mp_.half_map_size_z_ || pz <= -mp_.half_map_size_z_)
             ? false
             : true;
}

inline float DSPMapStaticV2::vectorMultiply(
    float &x1, float &y1, float &z1, float &x2, float &y2, float &z2) {
  return x1 * x2 + y1 * y2 + z1 * z2;
}

inline bool DSPMapStaticV2::ifInPyramidsArea(float &x, float &y, float &z) {
  //        vectorCOut(&pyramid_BPnorm_params_v[mp_.n_pyramid_obsrv_v_][0]);
  return vectorMultiply(x, y, z, pyramid_BPnorm_params_h[0][0], pyramid_BPnorm_params_h[0][1],
                        pyramid_BPnorm_params_h[0][2]) >= 0.F &&
         vectorMultiply(x, y, z, pyramid_BPnorm_params_h[mp_.n_pyramid_obsrv_h_][0],
                        pyramid_BPnorm_params_h[mp_.n_pyramid_obsrv_h_][1],
                        pyramid_BPnorm_params_h[mp_.n_pyramid_obsrv_h_][2]) <= 0.F &&
         vectorMultiply(x, y, z, pyramid_BPnorm_params_v[0][0], pyramid_BPnorm_params_v[0][1],
                        pyramid_BPnorm_params_v[0][2]) <= 0.F &&
         vectorMultiply(x, y, z, pyramid_BPnorm_params_v[mp_.n_pyramid_obsrv_v_][0],
                        pyramid_BPnorm_params_v[mp_.n_pyramid_obsrv_v_][1],
                        pyramid_BPnorm_params_v[mp_.n_pyramid_obsrv_v_][2]) >= 0.F;
}

inline void DSPMapStaticV2::rotateVectorByQuaternion(const float *             ori_vector,
                                                     const Eigen::Quaternionf &q,
                                                     float *                   rotated_vector) {
  // Lazy. Use Eigen directly
  Eigen::Quaternionf ori_vector_quaternion, vector_quaternion;
  ori_vector_quaternion.w() = 0;
  ori_vector_quaternion.x() = *ori_vector;
  ori_vector_quaternion.y() = *(ori_vector + 1);
  ori_vector_quaternion.z() = *(ori_vector + 2);

  vector_quaternion     = q * ori_vector_quaternion * q.inverse();
  *rotated_vector       = vector_quaternion.x();
  *(rotated_vector + 1) = vector_quaternion.y();
  *(rotated_vector + 2) = vector_quaternion.z();
}

}  // namespace dsp_map

#endif  // _DSP_MAP_H_