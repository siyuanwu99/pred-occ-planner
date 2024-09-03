/**
 * @file dsp_map_new.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-10-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "plan_env/dsp_map_new.h"

namespace dsp_map {

void DSPMapStaticV2::initMap(MappingParameters mp) {
  mp_ = mp;

  /** @brief Force to use pre-defined parameters
   * Dynamic memory allocation is not possible since required memory exceed maximum size of a single array
   */
  mp_.voxel_size_x_     = MAP_LENGTH_VOXEL_NUM;
  mp_.voxel_size_y_     = MAP_WIDTH_VOXEL_NUM;
  mp_.voxel_size_z_     = MAP_HEIGHT_VOXEL_NUM;
  mp_.n_voxel_          = VOXEL_NUM;
  mp_.resolution_       = VOXEL_RESOLUTION;
  mp_.angle_resolution_ = ANGLE_RESOLUTION;
  mp_.half_fov_h_       = half_fov_h;
  mp_.half_fov_v_       = half_fov_v;

  if (mp_.virtual_ceil_height_ - mp_.ground_height_ > mp_.map_size_z_) {
    mp_.virtual_ceil_height_ = mp_.ground_height_ + mp_.map_size_z_;
  }

  mp_.resolution_inv_ = 1 / mp_.resolution_;

  /* use map size when voxel size is unknown */
  // if (mp_.voxel_size_x_ < 0 || mp_.voxel_size_y_ < 0 || mp_.voxel_size_z_ < 0) {
  //   mp_.voxel_size_x_ = static_cast<int>(mp_.map_size_x_ * mp_.resolution_inv_);
  //   mp_.voxel_size_y_ = static_cast<int>(mp_.map_size_y_ * mp_.resolution_inv_);
  //   mp_.voxel_size_z_ = static_cast<int>(mp_.map_size_z_ * mp_.resolution_inv_);
  // } else {
  //   mp_.map_size_x_ = mp_.voxel_size_x_ * mp_.resolution_;
  //   mp_.map_size_y_ = mp_.voxel_size_y_ * mp_.resolution_;
  //   mp_.map_size_z_ = mp_.voxel_size_z_ * mp_.resolution_;
  // }

  mp_.map_size_x_ = mp_.voxel_size_x_ * mp_.resolution_;
  mp_.map_size_y_ = mp_.voxel_size_y_ * mp_.resolution_;
  mp_.map_size_z_ = mp_.voxel_size_z_ * mp_.resolution_;

  mp_.half_map_size_x_ = mp_.map_size_x_ * 0.5F;
  mp_.half_map_size_y_ = mp_.map_size_y_ * 0.5F;
  mp_.half_map_size_z_ = mp_.map_size_z_ * 0.5F;

  /* set initial parameters */
  mp_.angle_resolution_rad_ = static_cast<float>(mp_.angle_resolution_) / 180.F * M_PIf32;

  ROS_INFO("voxel size: %i %i %i", mp_.voxel_size_x_, mp_.voxel_size_y_, mp_.voxel_size_z_);
  // mp_.n_voxel_           = mp_.voxel_size_x_ * mp_.voxel_size_y_ * mp_.voxel_size_z_;
  mp_.n_pyramid_         = 360 * 180 / mp_.angle_resolution_ / mp_.angle_resolution_;
  mp_.n_pyramid_obsrv_h_ = static_cast<int>(mp_.half_fov_h_ * 2 / mp_.angle_resolution_);
  mp_.n_pyramid_obsrv_v_ = static_cast<int>(mp_.half_fov_v_ * 2 / mp_.angle_resolution_);
  mp_.n_pyramid_obsrv_   = mp_.n_pyramid_obsrv_h_ * mp_.n_pyramid_obsrv_v_;

  /* initialize index */
  md_.vel_gaussian_idx_ = 0;
  md_.pos_gaussian_idx_ = 0;
  md_.loc_gaussian_idx_ = 0;

  mp_.sigma_update_ = mp_.sigma_obsrv_;
  mp_.kappa_        = 0.01F;

  md_.P_detection_ = 0.95F;
  md_.update_time_ = 0.F;
  md_.idx_update_  = 0;
  md_.record_time_ = 1.F;
  md_.total_time_  = 0.0;

  /* initialize map data */
  md_.point_cloud_.resize(mp_.n_pyramid_obsrv_);
  md_.point_cloud_val_.resize(mp_.n_pyramid_obsrv_);
  for (int i = 0; i < mp_.n_pyramid_obsrv_; i++) {
    md_.point_cloud_[i].resize(mp_.n_particles_max_per_pyramid_);
    md_.point_cloud_val_[i].resize(mp_.n_particles_max_per_pyramid_);
  }  // TODO(siyuan): it this the best way to store data?
  md_.n_obs_each_pyramid_.resize(mp_.n_pyramid_obsrv_);
  md_.max_length_each_pyramid_.resize(mp_.n_pyramid_obsrv_);

  /* initialize gaussian randoms */
  md_.pos_gaussian_randoms_.reserve(GAUSSIAN_RANDOMS_NUM);
  md_.vel_gaussian_randoms_.reserve(GAUSSIAN_RANDOMS_NUM);
  md_.loc_gaussian_randoms_.reserve(GAUSSIAN_RANDOMS_NUM);

  /* initialize position data */
  md_.last_camera_pos_  = Eigen::Vector3f::Zero();
  md_.last_camera_pose_ = Eigen::Quaternionf::Identity();

  // Initialize voxels
  for (auto &i : _voxels_with_particle) {
    for (auto &j : i) {
      for (float &k : j) {
        k = 0.F;
      }
    }
  }

  // Initialize pyramids
  for (auto &pyramid : _pyramids_in_fov) {
    for (auto &p : pyramid) {
      p[0] = 0;
      p[1] = 0;
    }
  }

  /* Initialize point clouds */
  _cloud_in_current_view_rotated.reset(new pcl::PointCloud<pcl::PointXYZ>);
  _input_cloud_with_velocity.reset(new pcl::PointCloud<pcl::PointXYZINormal>);

  /// New: set pyramid plane initial parameters
  int h_start_seq = -mp_.half_fov_h_ / mp_.angle_resolution_;
  int h_end_seq   = -h_start_seq;
  for (int i = h_start_seq; i <= h_end_seq; i++) {
    pyramid_BPnorm_params_ori_h[i + h_end_seq][0] =
        -sin((float)i * mp_.angle_resolution_rad_);                                             // x
    pyramid_BPnorm_params_ori_h[i + h_end_seq][1] = cos((float)i * mp_.angle_resolution_rad_);  // y
    pyramid_BPnorm_params_ori_h[i + h_end_seq][2] = 0.F;                                        // z
  }

  int v_start_seq = -mp_.half_fov_v_ / mp_.angle_resolution_;
  int v_end_seq   = -v_start_seq;
  for (int i = v_start_seq; i <= v_end_seq; i++) {
    pyramid_BPnorm_params_ori_v[i + v_end_seq][0] = sin((float)i * mp_.angle_resolution_rad_);  // x
    pyramid_BPnorm_params_ori_v[i + v_end_seq][1] = 0.F;                                        // y
    pyramid_BPnorm_params_ori_v[i + v_end_seq][2] = cos((float)i * mp_.angle_resolution_rad_);  // z
  }

  // Find neighborhood pyramids' indexes for observation pyramids
  for (int i = 0; i < mp_.n_pyramid_obsrv_; i++) {  // Initialize point num in the storage
    findPyramidNeighborIndexInFOV(i, _observation_pyramid_neighbors[i][0],
                                  &_observation_pyramid_neighbors[i][1]);
  }

  // Generate Gaussian randoms.
  srand(static_cast<unsigned>(time(0)));  // TEST
  generateGaussianRandomsVectorZeroCenter();
  calculateNormalPDFBuffer();
}

int DSPMapStaticV2::update(int                      point_cloud_num,
                           int                      size_of_one_point,
                           float *                  point_cloud_ptr,
                           const Eigen::Vector3f    sensor_pos,
                           const Eigen::Quaternionf sensor_quat,
                           double                   time_stamp_second) {
  /** Get delt p **/
  // md_.last_camera_pos_                 = sensor_pos;

  static double time_stamp_second_last = time_stamp_second;

  // Check if the odometry data is invalid
  if (fabs(sensor_quat.x()) > 1.001F || fabs(sensor_quat.w()) > 1.001F ||
      fabs(sensor_quat.y()) > 1.001F || fabs(sensor_quat.z()) > 1.001F) {
    cout << "Invalid quaternion." << endl;
    return 0;
  }
  Eigen::Vector3f odom_delt_p = sensor_pos - md_.last_camera_pos_;

  auto delt_t = (float)(time_stamp_second - time_stamp_second_last);

  /* avoid rapid change */
  if (fabs(odom_delt_p.x()) > 10.F || fabs(odom_delt_p.y()) > 10.F ||
      fabs(odom_delt_p.z()) > 10.F || delt_t < 0.F || delt_t > 10.F) {
    cout << "!!! delt_t = " << delt_t << endl;
    cout << "!!! sensor_px_last = " << md_.last_camera_pos_.x() << "sensor_px = " << sensor_pos.x()
         << " odom_delt_px=" << odom_delt_p.x() << endl;
    cout << "!!! sensor_py_last = " << md_.last_camera_pos_.y() << "sensor_py = " << sensor_pos.y()
         << " odom_delt_py=" << odom_delt_p.y() << endl;
    return 0;
  }

  // clock_t start11, finish11;
  // start11 = clock();
  md_.camera_pos_        = sensor_pos;
  md_.last_camera_pos_   = md_.camera_pos_;
  _current_position[0]   = sensor_pos.x();
  _current_position[1]   = sensor_pos.y();
  _current_position[2]   = sensor_pos.z();
  time_stamp_second_last = time_stamp_second;

  _delt_t_from_last_observation = delt_t;

  /** Update pyramid boundary planes' normal vectors' parameters **/
  md_.camera_pose_ = sensor_quat;

  for (int i = 0; i < mp_.n_pyramid_obsrv_h_ + 1; i++) {
    rotateVectorByQuaternion(&pyramid_BPnorm_params_ori_h[i][0], md_.camera_pose_,
                             &pyramid_BPnorm_params_h[i][0]);
  }

  for (int j = 0; j < mp_.n_pyramid_obsrv_v_ + 1; j++) {
    rotateVectorByQuaternion(&pyramid_BPnorm_params_ori_v[j][0], md_.camera_pose_,
                             &pyramid_BPnorm_params_v[j][0]);
  }

  /** Insert point cloud to observation storage **/
  std::fill(md_.n_obs_each_pyramid_.begin(), md_.n_obs_each_pyramid_.end(), 0);
  std::fill(md_.max_length_each_pyramid_.begin(), md_.max_length_each_pyramid_.end(), -1.F);

  _cloud_in_current_view_rotated->clear();  // Clear first

  int iter_num =
      0;  // This is defined because one point has more than one float values. like px, py, pz
  int valid_points = 0;  // NOTE: the number of valid points might be different from input points
                         // number is the real FOV is larger than the defined FOV. However, if the
                         // point is outside of the map but is still in FOV, it will be counted.
  for (int p_seq = 0; p_seq < point_cloud_num; ++p_seq) {
    float rotated_point_this[3];
    rotateVectorByQuaternion(&point_cloud_ptr[iter_num], md_.camera_pose_, rotated_point_this);

    // Store in pcl point cloud for velocity estimation of new born particles
    pcl::PointXYZ p_this;
    p_this.x = rotated_point_this[0];
    p_this.y = rotated_point_this[1];
    p_this.z = rotated_point_this[2];
    _cloud_in_current_view_rotated->push_back(p_this);

    // Store in pyramids for update
    if (ifInPyramidsArea(rotated_point_this[0], rotated_point_this[1], rotated_point_this[2])) {
      int pyramid_index_h, pyramid_index_v;
      pyramid_index_h = findPointPyramidHorizontalIndex(
          rotated_point_this[0], rotated_point_this[1], rotated_point_this[2]);
      pyramid_index_v = findPointPyramidVerticalIndex(rotated_point_this[0], rotated_point_this[1],
                                                      rotated_point_this[2]);

      int pyramid_index         = pyramid_index_h * mp_.n_pyramid_obsrv_v_ + pyramid_index_v;
      int observation_inner_seq = md_.n_obs_each_pyramid_[pyramid_index];

      float length = sqrtf(rotated_point_this[0] * rotated_point_this[0] +
                           rotated_point_this[1] * rotated_point_this[1] +
                           rotated_point_this[2] * rotated_point_this[2]);

      // get point cloud position in global coordinate
      Eigen::Vector3f rp(rotated_point_this[0], rotated_point_this[1], rotated_point_this[2]);
      md_.point_cloud_[pyramid_index][observation_inner_seq]     = rp;
      md_.point_cloud_val_[pyramid_index][observation_inner_seq] = 0.F;
      // point_cloud_[pyramid_index][observation_inner_seq][4] = length;

      if (md_.max_length_each_pyramid_[pyramid_index] <
          length) {  // to be used to judge if a particle is occluded
        md_.max_length_each_pyramid_[pyramid_index] = length;
      }

      md_.n_obs_each_pyramid_[pyramid_index] += 1;

      // Omit the overflowed observation points. It is suggested to used a voxel filter on the input
      // point clouds to avoid overflow.
      if (md_.n_obs_each_pyramid_[pyramid_index] >= observation_max_points_num_one_pyramid) {
        md_.n_obs_each_pyramid_[pyramid_index] = observation_max_points_num_one_pyramid - 1;
      }

      ++valid_points;
    }

    iter_num += size_of_one_point;
  }

  mp_.newborn_objects_expected_ =
      mp_.newborn_particles_weight_ * (float)valid_points * (float)mp_.newborn_particles_per_point_;
  mp_.newborn_objects_weight_ =
      mp_.newborn_particles_weight_ * (float)mp_.newborn_particles_per_point_;

  /// Start a new thread for velocity estimation
  std::thread velocity_estimation(&DSPMapStaticV2::velocityEstimationThread, this);

  /*** Prediction ***/
  //        clock_t start7, finish7;
  //        start7 = clock();
  // TODO: slow
  /* Particles move in the opposite of the robot moving direction */
  mapPrediction(odom_delt_p, delt_t);
  //        finish7 = clock();
  //        double duration7= (double)(finish7 - start7) / CLOCKS_PER_SEC;
  //        printf( "****** Prediction time %f seconds\n", duration7 );

  /*** Update ***/
  //        clock_t start8, finish8;
  //        start8 = clock();
  if (point_cloud_num >= 0) {
    mapUpdate();  // TODO: too slow
  } else {
    cout << "No points to update." << endl;
  }

  //        finish8 = clock();
  //        double duration8= (double)(finish8 - start8) / CLOCKS_PER_SEC;
  //        printf( "****** Update time %f seconds\n", duration8 );

  /** Wait until optical flow calculation is finished **/
  velocity_estimation.join();

  /** Add updated new born particles ***/
  //        clock_t start1, finish1;
  //        start1 = clock();

  if (point_cloud_num >= 0) {
    mapAddNewBornParticlesByObservation();
  }

  //        finish1 = clock();
  //        double duration1 = (double)(finish1 - start1) / CLOCKS_PER_SEC;
  //        printf( "****** new born time %f seconds\n", duration1);

  /** Calculate object number and Resample **/
  //        clock_t start9, finish9;
  //        start9 = clock();
  /// NOTE in this step the flag which is set to be 7.F in prediction step will be changed to 1.F or
  /// 0.6F. Removing this step will make prediction malfunction unless the flag is reset somewhere
  /// else.
  mapOccupancyCalculationAndResample();  // TODO: slow

  //        finish9 = clock();
  //        double duration9 = (double)(finish9 - start9) / CLOCKS_PER_SEC;
  //        printf( "****** Resample time %f seconds\n", duration9);

  // finish11 = clock();
  // double duration11= (double)(finish11 - start11) / CLOCKS_PER_SEC;
  // printf( "****** Update time %f seconds\n", duration11 );

  // md_.total_time_ += duration11;
  // ++ s;
  // printf( "****** Average update time = %lf seconds\n", md_.total_time_ / double(update_times));

  //        printf( "****** Total time %f seconds\n", duration1 + duration7 + duration8 + duration9
  //        + duration11); printf("############################## \n \n");

  /*** Record particles for analysis  ***/
  static int recorded_once_flag = 0;

  if (mp_.is_csv_output_) {
    if (mp_.is_csv_output_ < 0 || (md_.update_time_ > md_.record_time_ && !recorded_once_flag)) {
      recorded_once_flag = 1;

      ofstream particle_log_writer;
      string   file_name = "/home/clarence/particles_update_t_" + to_string(md_.idx_update_) + "_" +
                         to_string((int)(md_.update_time_ * 1000)) + ".csv";
      particle_log_writer.open(file_name, ios::out | ios::trunc);

      for (int i = 0; i < mp_.n_voxel_; i++) {
        for (int j = 0; j < SAFE_PARTICLE_NUM_VOXEL; j++) {
          if (_voxels_with_particle[i][j][0] > 0.1F) {
            for (int k = 0; k < 8; k++) {
              //  1.Flag 2.vx 3.vy 4.vz 5.px 6.py 7.pz
              //  8.weight 9.update time
              particle_log_writer << _voxels_with_particle[i][j][k] << ",";
            }
            particle_log_writer << i << "\n";
          }
        }
      }
      particle_log_writer.close();
    }
  }

  return 1;
}

void DSPMapStaticV2::mapPrediction(const Eigen::Vector3f &dp, float dt) {
  // int operation_counter         = 0;
  // int exist_particles           = 0;
  // int voxel_full_remove_counter = 0, pyramid_full_remove_counter = 0;
  int moves_out_counter = 0;

  md_.update_time_ += dt;
  md_.idx_update_ += 1;

  /// Clear pyramids first
  for (auto &j : _pyramids_in_fov) {
    for (auto &i : j) {
      i[0] &= O_MAKE_INVALID;
    }
  }

  /// Update Particles' state and index in both voxels and pyramids
  for (int v_index = 0; v_index < VOXEL_NUM; ++v_index) {
    auto *v = _voxels_with_particle[v_index];
    for (int p = 0; p < SAFE_PARTICLE_NUM_VOXEL; p++) {
      auto *ptc  = v[p];
      float ptcf = ptc[0];
      if (ptcf > 0.1F && ptcf < 6.F) {  /// exit, but not new moved
        ptcf = 1.F;                     // If valid, remove resample flag.
        // ++operation_counter;

        if (fabs(ptc[1] * ptc[2] * ptc[3]) < 1e-6) {
          // keep small, for static obstacles
        } else {
          ptc[1] += getVelocityGaussianZeroCenter();  // vx
          ptc[2] += getVelocityGaussianZeroCenter();  // vy
          ptc[3] += getVelocityGaussianZeroCenter();  // vz
        }

        ptc[4] += dt * ptc[1] + dp.x() + getLocalizationGaussianZeroCenter();
        ptc[5] += dt * ptc[2] + dp.y() + getLocalizationGaussianZeroCenter();
        ptc[6] += dt * ptc[3] + dp.z() + getLocalizationGaussianZeroCenter();
        int particle_voxel_index_new;
        if (getParticleVoxelsIndex(ptc[4], ptc[5], ptc[6], particle_voxel_index_new)) {
          // move particle. If moved, the flag turns to 7.F. If should move but failed because
          // target voxel is full, delete the voxel.
          int move_flag = moveParticle(particle_voxel_index_new, v_index, p, &ptc[0]);
          if (move_flag == -2) {
            // Move the particle, if fails, "moveParticleByVoxel" will delete the particle
            // ++pyramid_full_remove_counter;
            continue;
          }

          if (move_flag == -1) {
            // ++voxel_full_remove_counter;
            continue;
          }
          // ++exist_particles;

        } else {
          /// Particle moves out
          removeParticle(&ptc[0]);
          ++moves_out_counter;
        }
      }
    }
  }

  //        cout << "exist_particles=" << exist_particles<<endl;
  //        cout << "voxel_full_remove_counter="<<voxel_full_remove_counter<<endl;
  //        cout << "pyramid_full_remove_counter="<<pyramid_full_remove_counter<<endl;
  //        cout << "moves_out_counter="<<moves_out_counter<<endl;
  //        cout << "operation_counter_prediction="<<operation_counter<<endl;

  if (moves_out_counter > 10000) {
    cout << "!!!!! An error occured! delt_t = " << dt << endl;
    cout << "odom_delt_px = " << dp.x() << " odom_delt_py = " << dp.y() << "odom_delt_pz=" << dp.z()
         << endl;
  }
}

void DSPMapStaticV2::mapUpdate() {
  // int operation_counter_update = 0;

  /// Calculate Ck + kappa first
  for (int i = 0; i < mp_.n_pyramid_obsrv_; ++i) {
    for (int j = 0; j < md_.n_obs_each_pyramid_[i]; ++j) {
      Eigen::Vector3f pt     = md_.point_cloud_[i][j];
      double          pt_val = mp_.newborn_objects_expected_ + mp_.kappa_;
      // Iteration of z
      for (int n_seq = 0; n_seq < _observation_pyramid_neighbors[i][0]; ++n_seq) {
        int pyramid_check_index =
            _observation_pyramid_neighbors[i][n_seq + 1];  // 0.neighbors num 1-9:neighbor indexes

        for (int particle_seq = 0; particle_seq < SAFE_PARTICLE_NUM_PYRAMID; ++particle_seq) {
          auto *particle = _pyramids_in_fov[pyramid_check_index][particle_seq];
          if (_pyramids_in_fov[pyramid_check_index][particle_seq][0] &
              O_MAKE_VALID) {  // Check only valid particles
            int particle_voxel_index       = particle[1];
            int particle_voxel_inner_index = particle[2];

            //                            cout<<"pyramid_check_index="<<pyramid_check_index<<",
            //                            particle_v_inner_index="<<particle_v_inner_index<<",
            //                            point_cloud[i][j][0]="<<point_cloud[i][j][0]<<endl;
            auto vx = _voxels_with_particle[particle_voxel_index][particle_voxel_inner_index];

            float gk = queryNormalPDF(vx[4], pt[0], mp_.sigma_update_) *
                       queryNormalPDF(vx[5], pt[1], mp_.sigma_update_) *
                       queryNormalPDF(vx[6], pt[2], mp_.sigma_update_);

            pt_val += md_.P_detection_ * vx[7] * gk;
          }
        }
      }
      /// add weight for new born particles
      md_.point_cloud_val_[i][j] = pt_val;
    }
  }

  /// Update weight for each particle in view
  for (int i = 0; i < mp_.n_pyramid_obsrv_; i++) {
    int current_pyramid_index = i;

    auto *current_pyramid = _pyramids_in_fov[current_pyramid_index];
    int   neighbor_num    = _observation_pyramid_neighbors[current_pyramid_index][0];
    for (int inner_seq = 0; inner_seq < SAFE_PARTICLE_NUM_PYRAMID; inner_seq++) {
      // Iteration of particles
      if (current_pyramid[inner_seq][0] & O_MAKE_VALID) {
        // update only valid particle

        int   particle_voxel_index       = current_pyramid[inner_seq][1];
        int   particle_voxel_inner_index = current_pyramid[inner_seq][2];
        auto *vpt = _voxels_with_particle[particle_voxel_index][particle_voxel_inner_index];

        float px = vpt[4];
        float py = vpt[5];
        float pz = vpt[6];

        float particle_dist_length = sqrtf(px * px + py * py + pz * pz);
        // Update only particles that are not occluded, use voxel_resolution as the distance metric.
        if (md_.max_length_each_pyramid_[i] > 0.F &&
            particle_dist_length > md_.max_length_each_pyramid_[i] + mp_.obstacles_inflation_) {
          // occluded
          continue;
        }

        float sum_by_zk = 0.F;
        for (int neighbor_seq = 0; neighbor_seq < neighbor_num; ++neighbor_seq) {
          int neighbor_index =
              _observation_pyramid_neighbors[current_pyramid_index][neighbor_seq + 1];
          for (int z_seq = 0; z_seq < md_.n_obs_each_pyramid_[neighbor_index];
               ++z_seq) {  // for all observation points in a neighbor pyramid
            Eigen::Vector3f pt     = md_.point_cloud_[neighbor_index][z_seq];
            float           pt_val = md_.point_cloud_val_[neighbor_index][z_seq];
            float           gk     = queryNormalPDF(px, pt[0], mp_.sigma_update_) *
                       queryNormalPDF(py, pt[1], mp_.sigma_update_) *
                       queryNormalPDF(pz, pt[2], mp_.sigma_update_);

            sum_by_zk += md_.P_detection_ * gk / pt_val;
            // ++operation_counter_update;
          }
        }

        vpt[7] *= ((1 - md_.P_detection_) + sum_by_zk);
        vpt[8] = md_.update_time_;
      }
    }
  }
  //        cout << "operation_counter_update=" << operation_counter_update <<endl;
}

void DSPMapStaticV2::mapOccupancyCalculationAndResample() {
  int removed_particle_counter                = 0;
  int particle_num_after_resampling_should_be = 0;

  for (int v_index = 0; v_index < VOXEL_NUM; ++v_index) {
    // Calculate estimated object number in each voxel
    // static float weight_sum_voxel, vx_sum_voxel, vy_sum_voxel, vz_sum_voxel;
    float weight_sum_voxel       = 0.F;
    float vx_sum_voxel           = 0.F;
    float vy_sum_voxel           = 0.F;
    float vz_sum_voxel           = 0.F;
    int   particle_num_voxel     = 0;
    int   old_particle_num_voxel = 0;

    auto *vptc = _voxels_with_particle[v_index];
    auto *vobj = _voxels_objects_number[v_index];
    for (int p = 0; p < SAFE_PARTICLE_NUM_VOXEL; p++) {
      auto ptkl = vptc[p];
      if (ptkl[0] > 0.1F) {
        if (ptkl[7] < 1e-3) {
          // Remove the particle directly if the weight is too small
          ptkl[0] = 0.F;
        } else {
          if (vptc[p][0] < 10.F) {  // exclude new-born particles
            ++old_particle_num_voxel;
            vx_sum_voxel += ptkl[1];
            vy_sum_voxel += ptkl[2];
            vz_sum_voxel += ptkl[3];

            /*** Future status prediction ***/
            for (int i = 0; i < PREDICTION_TIMES; ++i) {
              float prediction_time = _prediction_future_time[i];
              float px_future       = ptkl[4] + ptkl[1] * prediction_time;
              float py_future       = ptkl[5] + ptkl[2] * prediction_time;
              float pz_future       = ptkl[6] + ptkl[3] * prediction_time;

              int prediction_index;
              if (getParticleVoxelsIndex(px_future, py_future, pz_future, prediction_index)) {
                _voxels_objects_number[prediction_index][4 + i] += ptkl[7];  // weight
              }
            }
            /**** End of prediction ****/
          }

          _voxels_with_particle[v_index][p][0] =
              1.F;  // Remove newborn flag and moved flag in prediction
          ++particle_num_voxel;
          weight_sum_voxel += ptkl[7];
        }
      }
    }
    vobj[0] = weight_sum_voxel;
    if (old_particle_num_voxel > 0) {
      vobj[1] = vx_sum_voxel / (float)old_particle_num_voxel;
      vobj[2] = vy_sum_voxel / (float)old_particle_num_voxel;
      vobj[3] = vz_sum_voxel / (float)old_particle_num_voxel;
    } else {
      vobj[1] = 0.F;
      vobj[2] = 0.F;
      vobj[3] = 0.F;
    }

    if (particle_num_voxel < 5) {  // Too few particles, no need to resample.
      particle_num_after_resampling_should_be += particle_num_voxel;  // for test
      continue;
    }

    // Calculate desired particle number after resampling
    int particle_num_voxel_after_resample;
    if (particle_num_voxel > mp_.n_particles_max_per_voxel_) {
      particle_num_voxel_after_resample = mp_.n_particles_max_per_voxel_;
    } else {
      particle_num_voxel_after_resample = particle_num_voxel;
    }

    float weight_after_resample = weight_sum_voxel / (float)particle_num_voxel_after_resample;
    particle_num_after_resampling_should_be += particle_num_voxel_after_resample;
    // Resample
    float acc_ori_weight = 0.F;
    float acc_new_weight = weight_after_resample * 0.5F;
    for (int p = 0; p < SAFE_PARTICLE_NUM_VOXEL; ++p) {
      auto ptcl = vptc[p];
      if (ptcl[0] > 0.7F) {  // exclude invalid and newly_added_by_resampling particles
        float ori_particle_weight = ptcl[7];
        acc_ori_weight += ori_particle_weight;

        if (acc_ori_weight > acc_new_weight) {
          ptcl[7] = weight_after_resample;  // keep the particle but change weight
          acc_new_weight += weight_after_resample;

          bool if_space_is_currently_full = false;
          /** copy particles that have a very large weight **/
          int p_i = 0;

          while (acc_ori_weight >
                 acc_new_weight) {  // copy the particle if the original weight is very large
            bool if_found_position_in_voxel = false;
            if (!if_space_is_currently_full) {
              for (; p_i < SAFE_PARTICLE_NUM_VOXEL; ++p_i) {
                if (vptc[p_i][0] < 0.1F) {  // find an empty position in voxel
                  // Now copy the particle
                  vptc[p_i][0] = 0.6F;  // Flag: newly_added_by_resampling
                  for (int k = 1; k < 9; k++) {
                    vptc[p_i][k] = ptcl[k];
                  }
                  if_found_position_in_voxel = true;
                  break;
                }
              }
            }

            if (!if_found_position_in_voxel) {
              // If the particle should be copied but no space left in either voxel or pyramid,
              // add the weight of the original particle to keep the total weight unchanged.
              vptc[p][7] += weight_after_resample;
              if_space_is_currently_full = true;
            }

            acc_new_weight += weight_after_resample;
          }

        } else {
          // Remove the particle
          vptc[p][0] = 0.F;
          removed_particle_counter++;
        }
      }
    }
  }

  //        // Resampling result analysis
  //        int particle_num_after_resampling = 0;
  //
  //        for(int v_index=0; v_index<VOXEL_NUM; ++v_index)
  //        {
  //            for(int i=0; i<SAFE_PARTICLE_NUM_VOXEL; i++){
  //                if(_voxels_with_particle[v_index][i][0] > 0.1F){
  //                    ++ particle_num_after_resampling;
  //                }
  //            }
  //        }
  //        cout
  //        <<"particle_num_after_resampling_should_be="<<particle_num_after_resampling_should_be<<endl;
  //        cout <<"particle_num_after_resampling="<<particle_num_after_resampling<<endl;
  //        cout <<"removed_particle_counter="<<removed_particle_counter<<endl;
}

void DSPMapStaticV2::setPredictionVariance(float p_stddev, float v_stddev) {
  mp_.stddev_pos_predict_ = p_stddev;
  mp_.stddev_vel_predict_ = v_stddev;
  // regenerate randoms
  generateGaussianRandomsVectorZeroCenter();
}

void DSPMapStaticV2::setObservationStdDev(float ob_stddev) {
  mp_.sigma_obsrv_  = ob_stddev;
  mp_.sigma_update_ = mp_.sigma_obsrv_;

  cout << "Observation stddev changed to " << mp_.sigma_obsrv_ << endl;
}

void DSPMapStaticV2::setLocalizationStdDev(float lo_stddev) {
  float tuned_scale_factor = 0.1F;
  mp_.sigma_loc_           = lo_stddev * tuned_scale_factor;
  cout << "Localization stddev changed to " << mp_.sigma_loc_ << endl;
  // regenerate randoms
  generateGaussianRandomsVectorZeroCenter();
}

void DSPMapStaticV2::setParticleRecordFlag(bool record_particle_flag, float record_csv_time) {
  mp_.is_csv_output_ = record_particle_flag;
  md_.record_time_   = record_csv_time;
}

void DSPMapStaticV2::getObstaclePoints(int &                         obstacles_num,
                                       std::vector<Eigen::Vector3d> &points,
                                       const float                   threshold,
                                       const float                   clearance) {
  obstacles_num = 0;
  for (int i = 0; i < mp_.n_voxel_; i++) {
    if (_voxels_objects_number[i][0] > threshold) {
      Eigen::Vector3f pointf;
      getVoxelPositionFromIndex(i, pointf[0], pointf[1], pointf[2]);
      pointf                 = pointf + md_.camera_pos_;
      Eigen::Vector3d pointd = pointf.cast<double>();
      points.push_back(pointd);
      /* add inflated corner points */
      Eigen::Vector3f pointf_inflated;
      pointf_inflated = pointf + Eigen::Vector3f(clearance, clearance, 0);
      points.push_back(pointf_inflated.cast<double>());
      pointf_inflated = pointf + Eigen::Vector3f(clearance, -clearance, 0);
      points.push_back(pointf_inflated.cast<double>());
      pointf_inflated = pointf + Eigen::Vector3f(-clearance, clearance, 0);
      points.push_back(pointf_inflated.cast<double>());
      pointf_inflated = pointf + Eigen::Vector3f(-clearance, -clearance, 0);
      points.push_back(pointf_inflated.cast<double>());
      pointf_inflated = pointf + Eigen::Vector3f(clearance, 0, 0);
      points.push_back(pointf_inflated.cast<double>());
      pointf_inflated = pointf + Eigen::Vector3f(-clearance, 0, 0);
      points.push_back(pointf_inflated.cast<double>());
      pointf_inflated = pointf + Eigen::Vector3f(0, clearance, 0);
      points.push_back(pointf_inflated.cast<double>());
      pointf_inflated = pointf + Eigen::Vector3f(0, -clearance, 0);
      ++obstacles_num;
    }
  }
}

void DSPMapStaticV2::getOccupancyMap(int &                           obstacles_num,
                                     pcl::PointCloud<pcl::PointXYZ> &cloud,
                                     const float                     threshold) {
  obstacles_num = 0;
  for (int i = 0; i < mp_.n_voxel_; i++) {
    if (_voxels_objects_number[i][0] > threshold) {
      pcl::PointXYZ pcl_point;
      getVoxelPositionFromIndex(i, pcl_point.x, pcl_point.y, pcl_point.z);
      pcl_point.x += md_.camera_pos_.x();
      pcl_point.y += md_.camera_pos_.y();
      pcl_point.z += md_.camera_pos_.z();
      cloud.push_back(pcl_point);

      ++obstacles_num;
    }

    /// Clear weights for next prediction
    for (int j = 4; j < _voxels_objects_number_dimension; ++j) {
      _voxels_objects_number[i][j] = 0.F;
    }
  }
}

void DSPMapStaticV2::getOccupancyMapWithVelocity(int &                              obstacles_num,
                                                 std::vector<float> &               weights,
                                                 pcl::PointCloud<pcl::PointNormal> &cloud,
                                                 const float                        threshold) {
  obstacles_num = 0;
  for (int i = 0; i < mp_.n_voxel_; i++) {
    if (_voxels_objects_number[i][0] > threshold) {
      pcl::PointNormal pcl_point;
      pcl_point.normal_x = _voxels_objects_number[i][1];  // vx
      pcl_point.normal_y = _voxels_objects_number[i][2];  // vy
      pcl_point.normal_z = _voxels_objects_number[i][3];  // vz
      getVoxelPositionFromIndex(i, pcl_point.x, pcl_point.y, pcl_point.z);
      pcl_point.x += md_.camera_pos_.x();
      pcl_point.y += md_.camera_pos_.y();
      pcl_point.z += md_.camera_pos_.z();
      cloud.push_back(pcl_point);
      weights.push_back(_voxels_objects_number[i][0]);
      ++obstacles_num;
    }

    /// Clear weights for next prediction
    for (int j = 4; j < _voxels_objects_number_dimension; ++j) {
      _voxels_objects_number[i][j] = 0.F;
    }
  }
}

void DSPMapStaticV2::getOccupancyMapWithFutureStatus(int &                           obstacles_num,
                                                     pcl::PointCloud<pcl::PointXYZ> &cloud,
                                                     float *                         future_status,
                                                     const float                     threshold) {
  obstacles_num = 0;
  for (int i = 0; i < mp_.n_voxel_; i++) {
    if (_voxels_objects_number[i][0] > threshold) {
      pcl::PointXYZ pcl_point;
      getVoxelPositionFromIndex(i, pcl_point.x, pcl_point.y, pcl_point.z);
      pcl_point.x += md_.camera_pos_.x();
      pcl_point.y += md_.camera_pos_.y();
      pcl_point.z += md_.camera_pos_.z();
      cloud.push_back(pcl_point);

      ++obstacles_num;
    }

    for (int n = 0; n < PREDICTION_TIMES; ++n) {  // Set future weights
      *(future_status + i * PREDICTION_TIMES + n) = _voxels_objects_number[i][n + 4];
    }

    /// Clear weights for next prediction
    for (int j = 4; j < _voxels_objects_number_dimension; ++j) {
      _voxels_objects_number[i][j] = 0.F;
    }
  }
}

void DSPMapStaticV2::getOccupancyMapWithRiskMaps(int &                           obstacles_num,
                                                 pcl::PointCloud<pcl::PointXYZ> &cloud,
                                                 float *                         risk_maps,
                                                 const float                     threshold) {
  obstacles_num = 0;
  for (int i = 0; i < mp_.n_voxel_; i++) {
    if (_voxels_objects_number[i][0] > threshold) {
      pcl::PointXYZ pcl_point;
      getVoxelPositionFromIndex(i, pcl_point.x, pcl_point.y, pcl_point.z);
      pcl_point.x += md_.camera_pos_.x();
      pcl_point.y += md_.camera_pos_.y();
      pcl_point.z += md_.camera_pos_.z();
      cloud.push_back(pcl_point);

      ++obstacles_num;
    }

    for (int n = 0; n < mp_.n_risk_map_; ++n) {  // Set future weights
      float temp_risk = 0.F;
      for (int m = 0; m < mp_.n_prediction_per_risk_map_; ++m) {
        temp_risk += _voxels_objects_number[i][4 + n * mp_.n_prediction_per_risk_map_ + m];
      }
      *(risk_maps + i * mp_.n_risk_map_ + n) = temp_risk;
    }

    /// Clear weights for next prediction
    for (int j = 4; j < _voxels_objects_number_dimension; ++j) {
      _voxels_objects_number[i][j] = 0.F;
    }
  }
}

void DSPMapStaticV2::clearOccupancyMapPrediction() {
  for (int i = 0; i < mp_.n_voxel_; i++) {
    for (int j = 4; j < _voxels_objects_number_dimension; ++j) {
      _voxels_objects_number[i][j] = 0.F;
    }
  }
}

void DSPMapStaticV2::getKMClusterResult(pcl::PointCloud<pcl::PointXYZINormal> &cluster_cloud) {
  for (auto &point : *_input_cloud_with_velocity) {
    cluster_cloud.push_back(point);
  }
}

void DSPMapStaticV2::addRandomParticles(int particle_num, float avg_weight) {
  /*** Initialize some particles ***/
  int successfully_added_num = 0;
  int voxel_overflow_num     = 0;

  for (int i = 0; i < particle_num; i++) {
    std::shared_ptr<Particle> particle_ptr{new Particle};

    particle_ptr->px     = generateRandomFloat(-mp_.half_map_size_x_, mp_.half_map_size_x_);
    particle_ptr->py     = generateRandomFloat(-mp_.half_map_size_y_, mp_.half_map_size_y_);
    particle_ptr->pz     = generateRandomFloat(-mp_.half_map_size_z_, mp_.half_map_size_z_);
    particle_ptr->vx     = generateRandomFloat(-1.F, 1.F);
    particle_ptr->vy     = generateRandomFloat(-1.F, 1.F);
    particle_ptr->vz     = generateRandomFloat(-1.F, 1.F);
    particle_ptr->weight = avg_weight;

    if (getParticleVoxelsIndex(*particle_ptr, particle_ptr->voxel_index)) {
      int test = addAParticle(*particle_ptr, particle_ptr->voxel_index);
      if (test > 0) {
        successfully_added_num++;
      } else {
        voxel_overflow_num++;
      }
    }
  }

  //        cout << "successfully_added_num="<<successfully_added_num<<endl;
  //        cout << "voxel_overflow_num="<<voxel_overflow_num<<endl;
}

void DSPMapStaticV2::mapAddNewBornParticlesByObservation() {
  /** Calculate normalization coefficient first **/
  float normalization_coefficient = 0.F;
  for (int i = 0; i < mp_.n_pyramid_obsrv_; i++) {
    for (int j = 0; j < md_.n_obs_each_pyramid_[i]; j++) {
      double val = md_.point_cloud_val_[i][j];
      normalization_coefficient += 1.F / val;
    }
  }
  float updated_weight_new_born = mp_.newborn_particles_weight_ * normalization_coefficient;

  /** Add new born particles **/
  static int min_static_new_born_particle_number_each_point =
      (int)((float)mp_.newborn_particles_per_point_ * 0.15F);
  static int static_new_born_particle_number_each_point =
      (int)((float)mp_.newborn_particles_per_point_ * 0.4F);  // static points takes 3 in 10
  static int pf_derive_new_born_particle_number_each_point =
      (int)((float)mp_.newborn_particles_per_point_ * 0.5F);  // Derived takes half
  static const int model_generated_particle_number_each_point =
      (int)((float)mp_.newborn_particles_per_point_ * 0.8F);

  int successfully_born_particles = 0;
  /// TODO: Improve efficiency in this new born procedure
  for (auto &point : *_input_cloud_with_velocity) {
    pcl::PointXYZ p_corrected;
    p_corrected.x = point.x - _current_position[0];
    p_corrected.y = point.y - _current_position[1];
    p_corrected.z = point.z - _current_position[2];

    int   point_voxel_index;
    float static_particle_weight_sum   = 0.F;
    float dynamic_particle_weight_sum  = 0.F;
    float static_or_dynamic_weight_sum = 0.F;

    if (getParticleVoxelsIndex(p_corrected.x, p_corrected.y, p_corrected.z, point_voxel_index)) {
      // This condition should always be true because the point cloud outside of the map should be
      // omitted in the first place. Just an insurance.
      for (int kk = 0; kk < SAFE_PARTICLE_NUM_VOXEL; ++kk) {
        if (_voxels_with_particle[point_voxel_index][kk][0] > 0.9F &&
            _voxels_with_particle[point_voxel_index][kk][0] < 14.F) {  // not new born
          float v_abs = fabs(_voxels_with_particle[point_voxel_index][kk][1]) +
                        fabs(_voxels_with_particle[point_voxel_index][kk][2]) +
                        fabs(_voxels_with_particle[point_voxel_index][kk][3]);
          if (v_abs < 0.1F) {
            // Static
            static_particle_weight_sum += _voxels_with_particle[point_voxel_index][kk][7];
          } else if (v_abs < 0.5F) {
            // Static or dynamic
            static_or_dynamic_weight_sum += _voxels_with_particle[point_voxel_index][kk][7];
          } else {
            // Dynamic
            dynamic_particle_weight_sum += _voxels_with_particle[point_voxel_index][kk][7];
          }
        }
      }
    } else {
      continue;
    }

    // Dempster-Shafer Theory
    float total_weight_voxel =
        static_particle_weight_sum + dynamic_particle_weight_sum + static_or_dynamic_weight_sum;
    float m_static            = static_particle_weight_sum / total_weight_voxel;
    float m_dynamic           = dynamic_particle_weight_sum / total_weight_voxel;
    float m_static_or_dynamic = static_or_dynamic_weight_sum / total_weight_voxel;

    float p_static             = (m_static + m_static + m_static_or_dynamic) * 0.5F;
    float p_dynamic            = (m_dynamic + m_dynamic + m_static_or_dynamic) * 0.5F;
    float normalization_p      = p_static + p_dynamic;
    float p_static_normalized  = p_static / normalization_p;
    float p_dynamic_normalized = p_dynamic / normalization_p;

    static_new_born_particle_number_each_point =
        (int)((float)model_generated_particle_number_each_point * p_static_normalized);
    pf_derive_new_born_particle_number_each_point =
        model_generated_particle_number_each_point - static_new_born_particle_number_each_point;

    // Set a minimum number of static particles
    static_new_born_particle_number_each_point = max(min_static_new_born_particle_number_each_point,
                                                     static_new_born_particle_number_each_point);

    for (int p = 0; p < mp_.newborn_particles_per_point_; p++) {
      std::shared_ptr<Particle> particle_ptr{new Particle};

      particle_ptr->px = p_corrected.x + getPositionGaussianZeroCenter();
      particle_ptr->py = p_corrected.y + getPositionGaussianZeroCenter();
      particle_ptr->pz = p_corrected.z + getPositionGaussianZeroCenter();

      if (getParticleVoxelsIndex(*particle_ptr, particle_ptr->voxel_index)) {
        // Particle index might be different from the point index because a random Gaussian is
        // added.
        if (p < static_new_born_particle_number_each_point) {  // add static points
          particle_ptr->vx = 0.F;
          particle_ptr->vy = 0.F;
          particle_ptr->vz = 0.F;
        } else if (
            point.normal_x > -100.F &&
            p < model_generated_particle_number_each_point) {  // p <
                                                               // pf_derive_new_born_particle_number_each_point
                                                               // +
                                                               // static_new_born_particle_number_each_point){
          /// Use estimated velocity to generate new particles
          if (point.intensity > 0.01F) {
            particle_ptr->vx = point.normal_x + 4 * getVelocityGaussianZeroCenter();
            particle_ptr->vy = point.normal_y + 4 * getVelocityGaussianZeroCenter();
            particle_ptr->vz = point.normal_z + 4 * getVelocityGaussianZeroCenter();
          } else {  // static points like ground
            particle_ptr->vx = 0.F;
            particle_ptr->vy = 0.F;
            particle_ptr->vz = 0.F;
          }
        } else {  /// Considering Random Noise
          if (point.intensity > 0.01F) {
            particle_ptr->vx = generateRandomFloat(-1.5F, 1.5F);
            particle_ptr->vy = generateRandomFloat(-1.5F, 1.5F);
            particle_ptr->vz = generateRandomFloat(-0.5F, 0.5F);
          } else {  // static points like ground
            particle_ptr->vx = 0.F;
            particle_ptr->vy = 0.F;
            particle_ptr->vz = 0.F;
          }
        }
        particle_ptr->weight = updated_weight_new_born;

        int test = addAParticle(*particle_ptr, particle_ptr->voxel_index);
        if (test > 0) {
          ++successfully_born_particles;
        }
      }
    }
  }

  //        cout << "successfully_born_particles = "<<successfully_born_particles<<endl;
}

int DSPMapStaticV2::getParticleVoxelsIndex(const Particle &p, int &index) {
  if (!isInMap(p)) {
    return 0;
  }
  auto x = (int)((p.px + mp_.half_map_size_x_) / mp_.resolution_);
  auto y = (int)((p.py + mp_.half_map_size_y_) / mp_.resolution_);
  auto z = (int)((p.pz + mp_.half_map_size_z_) / mp_.resolution_);
  index  = z * mp_.voxel_size_y_ * mp_.voxel_size_x_ + y * mp_.voxel_size_x_ + x;

  if (index < 0 || index >= VOXEL_NUM) {
    return 0;
  }

  return 1;
}

int DSPMapStaticV2::getParticleVoxelsIndex(const float &px,
                                           const float &py,
                                           const float &pz,
                                           int &        index) {
  if (!isInMap(px, py, pz)) {
    return 0;
  }
  auto x = (int)((px + mp_.half_map_size_x_) / mp_.resolution_);
  auto y = (int)((py + mp_.half_map_size_y_) / mp_.resolution_);
  auto z = (int)((pz + mp_.half_map_size_z_) / mp_.resolution_);
  index  = z * mp_.voxel_size_y_ * mp_.voxel_size_x_ + y * mp_.voxel_size_x_ + x;

  if (index < 0 || index >= VOXEL_NUM) {
    return 0;
  }

  return 1;
}

void DSPMapStaticV2::getVoxelPositionFromIndex(const int &index,
                                               float &    px,
                                               float &    py,
                                               float &    pz) const {
  static const int z_change_storage_taken = mp_.voxel_size_y_ * mp_.voxel_size_x_;
  static const int y_change_storage_taken = mp_.voxel_size_x_;

  int z_index         = index / z_change_storage_taken;
  int yx_left_indexes = index - z_index * z_change_storage_taken;
  int y_index         = yx_left_indexes / y_change_storage_taken;
  int x_index         = yx_left_indexes - y_index * y_change_storage_taken;

  static const float correction_x = -mp_.half_map_size_x_ + mp_.resolution_ * 0.5F;
  static const float correction_y = -mp_.half_map_size_y_ + mp_.resolution_ * 0.5F;
  static const float correction_z = -mp_.half_map_size_z_ + mp_.resolution_ * 0.5F;

  px = (float)x_index * mp_.resolution_ + correction_x;
  py = (float)y_index * mp_.resolution_ + correction_y;
  pz = (float)z_index * mp_.resolution_ + correction_z;
}

void DSPMapStaticV2::generateGaussianRandomsVectorZeroCenter() {
  std::default_random_engine      random(time(NULL));
  std::normal_distribution<float> n1(0, mp_.stddev_pos_predict_);
  std::normal_distribution<float> n2(0, mp_.stddev_vel_predict_);
  std::normal_distribution<float> n3(0, mp_.sigma_loc_);

  for (int i = 0; i < GAUSSIAN_RANDOMS_NUM; i++) {
    md_.pos_gaussian_randoms_.emplace_back(n1(random));
    md_.vel_gaussian_randoms_.emplace_back(n2(random));
    md_.loc_gaussian_randoms_.emplace_back(n3(random));
  }
}

float DSPMapStaticV2::getPositionGaussianZeroCenter() {
  float delt_p = md_.pos_gaussian_randoms_[md_.pos_gaussian_idx_];
  md_.pos_gaussian_idx_ += 1;
  if (md_.pos_gaussian_idx_ >= GAUSSIAN_RANDOMS_NUM) {
    md_.pos_gaussian_idx_ = 0;
  }
  return delt_p;
}

float DSPMapStaticV2::getLocalizationGaussianZeroCenter() {
  float delt_p = md_.loc_gaussian_randoms_[md_.loc_gaussian_idx_];
  md_.loc_gaussian_idx_ += 1;
  if (md_.loc_gaussian_idx_ >= GAUSSIAN_RANDOMS_NUM) {
    md_.loc_gaussian_idx_ = 0;
  }
  return delt_p;
}

float DSPMapStaticV2::getVelocityGaussianZeroCenter() {
  float delt_v = md_.vel_gaussian_randoms_[md_.vel_gaussian_idx_];
  md_.vel_gaussian_idx_ += 1;
  if (md_.vel_gaussian_idx_ >= GAUSSIAN_RANDOMS_NUM) {
    md_.vel_gaussian_idx_ = 0;
  }
  return delt_v;
}

int DSPMapStaticV2::addAParticle(const Particle &p, const int &voxel_index) const {
  for (int i = 0; i < SAFE_PARTICLE_NUM_VOXEL; i++) {
    if (_voxels_with_particle[voxel_index][i][0] < 0.1F) {  // found an empty particle position
      _voxels_with_particle[voxel_index][i][0] = 15.F;      // New born flag
      _voxels_with_particle[voxel_index][i][1] = p.vx;
      _voxels_with_particle[voxel_index][i][2] = p.vy;
      _voxels_with_particle[voxel_index][i][3] = p.vz;
      _voxels_with_particle[voxel_index][i][4] = p.px;
      _voxels_with_particle[voxel_index][i][5] = p.py;
      _voxels_with_particle[voxel_index][i][6] = p.pz;
      _voxels_with_particle[voxel_index][i][7] = p.weight;
      _voxels_with_particle[voxel_index][i][8] = md_.update_time_;

      return 1;
    }
  }  /// If no space. Omit this particle in voxel

  return 0;
}

int DSPMapStaticV2::moveParticle(const int &new_voxel_index,
                                 const int &current_v_index,
                                 const int &current_v_inner_index,
                                 float *    ori_particle_flag_ptr) {
  int new_voxel_inner_index = current_v_inner_index;
  if (new_voxel_index != current_v_index) {
    *ori_particle_flag_ptr = 0.F;  // Remove from ori voxel first

    /// Find a space in the new voxel and then pyramid. If no space in either voxel or pyramid.
    /// This particle would vanish.
    int successfully_moved_by_voxel = 0;
    for (int i = 0; i < SAFE_PARTICLE_NUM_VOXEL; ++i) {
      if (_voxels_with_particle[new_voxel_index][i][0] < 0.1F) {  // empty
        new_voxel_inner_index       = i;
        successfully_moved_by_voxel = 1;

        _voxels_with_particle[new_voxel_index][i][0] = 7.F;  // newly moved flag
        for (int k = 1; k < 9; ++k) {                        // set v, p, weight, update time
          _voxels_with_particle[new_voxel_index][i][k] = *(ori_particle_flag_ptr + k);
        }
        break;  /// Important
      }
    }

    if (!successfully_moved_by_voxel) {  /// let the particle vanish
      return -1;
    }
  }

  // Now check pyramid, pyramids are cleared first so the particle in FOV must be added unless
  // full.
  if (ifInPyramidsArea(_voxels_with_particle[new_voxel_index][new_voxel_inner_index][4],
                       _voxels_with_particle[new_voxel_index][new_voxel_inner_index][5],
                       _voxels_with_particle[new_voxel_index][new_voxel_inner_index][6])) {
    int h_index = findPointPyramidHorizontalIndex(
        _voxels_with_particle[new_voxel_index][new_voxel_inner_index][4],
        _voxels_with_particle[new_voxel_index][new_voxel_inner_index][5],
        _voxels_with_particle[new_voxel_index][new_voxel_inner_index][6]);

    int v_index = findPointPyramidVerticalIndex(
        _voxels_with_particle[new_voxel_index][new_voxel_inner_index][4],
        _voxels_with_particle[new_voxel_index][new_voxel_inner_index][5],
        _voxels_with_particle[new_voxel_index][new_voxel_inner_index][6]);

    int particle_pyramid_index_new = h_index * mp_.n_pyramid_obsrv_v_ + v_index;

    int successfully_moved_by_pyramid = 0;
    for (int j = 0; j < SAFE_PARTICLE_NUM_PYRAMID; j++) {
      if (_pyramids_in_fov[particle_pyramid_index_new][j][0] == 0) {
        _pyramids_in_fov[particle_pyramid_index_new][j][0] |= O_MAKE_VALID;
        _pyramids_in_fov[particle_pyramid_index_new][j][1] = new_voxel_index;
        _pyramids_in_fov[particle_pyramid_index_new][j][2] = new_voxel_inner_index;
        successfully_moved_by_pyramid                      = 1;
        break;
      }
    }

    if (!successfully_moved_by_pyramid) {  /// let the particle vanish
      _voxels_with_particle[new_voxel_index][new_voxel_inner_index][0] = 0.F;  /// vanish
      return -2;
    }

    /// Add Gaussian randoms to velocities of particles inside FOV
    if (fabs(_voxels_with_particle[new_voxel_index][new_voxel_inner_index][1] *
             _voxels_with_particle[new_voxel_index][new_voxel_inner_index][2] *
             _voxels_with_particle[new_voxel_index][new_voxel_inner_index][3]) < 1e-6) {
      // keep small, for static obstacles
      //                cout << "keeped"<<endl;
    } else {
      _voxels_with_particle[new_voxel_index][new_voxel_inner_index][1] +=
          getVelocityGaussianZeroCenter();  // vx
      _voxels_with_particle[new_voxel_index][new_voxel_inner_index][2] +=
          getVelocityGaussianZeroCenter();  // vy
      _voxels_with_particle[new_voxel_index][new_voxel_inner_index][3] =
          0.F;  //+= getVelocityGaussianZeroCenter();  //vz
    }

  }  // else we don't need to consider pyramids

  return 1;
}

int DSPMapStaticV2::findPointPyramidHorizontalIndex(
    float &x, float &y, float &z) {  /// The point should already be inside of Pyramids Area
  float last_dot_multiply =
      1.F;  // for horizontal direction, if the point is inside of Pyramids Area. The symbol of
            // the first dot multiplication should be positive
  for (int i = 0; i < mp_.n_pyramid_obsrv_h_; i++) {
    float this_dot_multiply =
        vectorMultiply(x, y, z, pyramid_BPnorm_params_h[i + 1][0],
                       pyramid_BPnorm_params_h[i + 1][1], pyramid_BPnorm_params_h[i + 1][2]);
    if (last_dot_multiply * this_dot_multiply <= 0.F) {
      return i;
    }
    last_dot_multiply = this_dot_multiply;
  }

  cout << "!!!!!! Please use Function ifInPyramidsArea() to filter the points first before using "
          "findPointPyramidHorizontalIndex()"
       << endl;
  return -1;  // This should not happen if the function is used properly
}

int DSPMapStaticV2::findPointPyramidVerticalIndex(
    float &x, float &y, float &z) {  /// The point should already be inside of Pyramids Area
  float last_dot_multiply =
      -1.F;  // for vertical direction, if the point is inside of Pyramids Area. The symbol of the
             // first dot multiplication should be negative
  for (int j = 0; j < mp_.n_pyramid_obsrv_v_; j++) {
    float this_dot_multiply =
        vectorMultiply(x, y, z, pyramid_BPnorm_params_v[j + 1][0],
                       pyramid_BPnorm_params_v[j + 1][1], pyramid_BPnorm_params_v[j + 1][2]);
    if (last_dot_multiply * this_dot_multiply <= 0.F) {
      return j;
    }
    last_dot_multiply = this_dot_multiply;
  }

  cout << "!!!!!! Please use Function ifInPyramidsArea() to filter the points first before using "
          "findPyramidVerticalIndex()"
       << endl;
  return -1;  // This should not happen if the function is used properly
}

void DSPMapStaticV2::velocityEstimationThread() {
  if (_cloud_in_current_view_rotated->points.empty()) return;

  _input_cloud_with_velocity->clear();

  /// Remove ground and transform data
  pcl::PointCloud<pcl::PointXYZ>::Ptr static_points(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_points(new pcl::PointCloud<pcl::PointXYZ>());

  for (auto &p : _cloud_in_current_view_rotated->points) {
    pcl::PointXYZ transformed_p;
    transformed_p.x = p.x + _current_position[0];
    transformed_p.y = p.y + _current_position[1];
    transformed_p.z = p.z + _current_position[2];

    if (transformed_p.z > _voxel_filtered_resolution) {
      non_ground_points->points.push_back(transformed_p);
    } else {
      static_points->points.push_back(transformed_p);
    }
  }

  /// Cluster
  static std::vector<ClusterFeature> clusters_feature_vector_dynamic_last;
  std::vector<ClusterFeature>        clusters_feature_vector_dynamic;
  std::vector<pcl::PointIndices>     cluster_indices;
  vector<bool>                       cluster_possibly_dynamic;

  if (!non_ground_points->empty()) {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(non_ground_points);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(2 * _voxel_filtered_resolution);
    ec.setMinClusterSize(5);
    ec.setMaxClusterSize(10000);

    ec.setSearchMethod(tree);
    ec.setInputCloud(non_ground_points);
    ec.extract(cluster_indices);

    for (const auto &cluster_indice : cluster_indices) {
      ClusterFeature cluster_this;
      float          intensity = generateRandomFloat(0.F, 1.F);
      cluster_this.intensity   = intensity;  // For visualization

      for (int indice : cluster_indice.indices) {
        cluster_this.center_x += (*non_ground_points)[indice].x;  // sum
        cluster_this.center_y += (*non_ground_points)[indice].y;
        cluster_this.center_z += (*non_ground_points)[indice].z;
        ++cluster_this.point_num;
      }

      // average
      cluster_this.center_x /= (float)cluster_this.point_num;
      cluster_this.center_y /= (float)cluster_this.point_num;
      cluster_this.center_z /= (float)cluster_this.point_num;

      if (cluster_indice.indices.size() > 150 ||
          cluster_this.center_z > 1.4) {  // filter static points  //400, 1.5
        // Static
        for (int indice : cluster_indice.indices) {
          static_points->push_back((*non_ground_points)[indice]);
        }
        cluster_possibly_dynamic.push_back(false);
      } else {
        // Possibly dynamic
        clusters_feature_vector_dynamic.push_back(cluster_this);
        cluster_possibly_dynamic.push_back(true);
      }
    }

    static float distance_gate    = 1.5F;
    static int   point_num_gate   = 100;
    static float maximum_velocity = 5.F;

    /// Move last feature vector d and match by KM algorithm
    if (!clusters_feature_vector_dynamic_last.empty() && !clusters_feature_vector_dynamic.empty()) {
      if (_delt_t_from_last_observation > 0.00001 && _delt_t_from_last_observation < 10.0) {
        Matrix<float> matrix_cost(
            clusters_feature_vector_dynamic.size(),
            clusters_feature_vector_dynamic_last.size());  // This is a Matrix defined in munkres.h
        Matrix<float> matrix_gate(clusters_feature_vector_dynamic.size(),
                                  clusters_feature_vector_dynamic_last.size());

        for (int row = 0; row < clusters_feature_vector_dynamic.size(); ++row) {
          for (int col = 0; col < clusters_feature_vector_dynamic_last.size(); ++col) {
            float cluster_distance_this = clusterDistance(
                clusters_feature_vector_dynamic[row], clusters_feature_vector_dynamic_last[col]);
            if (abs(clusters_feature_vector_dynamic[row].point_num -
                    clusters_feature_vector_dynamic_last[col].point_num) > point_num_gate ||
                cluster_distance_this >= distance_gate) {
              matrix_gate(row, col) = 0.F;
              matrix_cost(row, col) = distance_gate * 5000.F;
            } else {
              matrix_gate(row, col) = 1.F;
              matrix_cost(row, col) = cluster_distance_this / distance_gate * 1000.F;
            }
          }
        }

        Munkres<float> munkres_solver;
        munkres_solver.solve(matrix_cost);

        for (int row = 0; row < clusters_feature_vector_dynamic.size(); ++row) {
          for (int col = 0; col < clusters_feature_vector_dynamic_last.size(); ++col) {
            if (matrix_cost(row, col) == 0.F && matrix_gate(row, col) > 0.01F) {  // Found a match
              clusters_feature_vector_dynamic[row].match_cluster_seq = col;
              clusters_feature_vector_dynamic[row].vx =
                  (clusters_feature_vector_dynamic[row].center_x -
                   clusters_feature_vector_dynamic_last[col].center_x) /
                  _delt_t_from_last_observation;
              clusters_feature_vector_dynamic[row].vy =
                  (clusters_feature_vector_dynamic[row].center_y -
                   clusters_feature_vector_dynamic_last[col].center_y) /
                  _delt_t_from_last_observation;
              clusters_feature_vector_dynamic[row].vz =
                  (clusters_feature_vector_dynamic[row].center_z -
                   clusters_feature_vector_dynamic_last[col].center_z) /
                  _delt_t_from_last_observation;
              //                        cout << "v=("<<clusters_feature_vector_dynamic[row].vx<<",
              //                        " << clusters_feature_vector_dynamic[row].vy <<",
              //                        "<<clusters_feature_vector_dynamic[row].vz << ")" << endl;
              clusters_feature_vector_dynamic[row].v =
                  sqrtf(clusters_feature_vector_dynamic[row].vx *
                            clusters_feature_vector_dynamic[row].vx +
                        clusters_feature_vector_dynamic[row].vy *
                            clusters_feature_vector_dynamic[row].vy +
                        clusters_feature_vector_dynamic[row].vz *
                            clusters_feature_vector_dynamic[row].vz);
              clusters_feature_vector_dynamic[row].intensity =
                  clusters_feature_vector_dynamic_last[col].intensity;  // for visualization

              if (clusters_feature_vector_dynamic[row].v > maximum_velocity) {
                clusters_feature_vector_dynamic[row].v  = 0.F;
                clusters_feature_vector_dynamic[row].vx = clusters_feature_vector_dynamic[row].vy =
                    clusters_feature_vector_dynamic[row].vz = 0.F;
              }

              break;
            }
            /// If no match is found. The cluster velocity is given by struct initialization
            /// (v=-1000).
          }
        }
      }
    }

    /// Velocity Allocation to Points
    // Use normal to store velocity
    int cluster_indice_seq         = 0;
    int cluster_dynamic_vector_seq = 0;
    for (const auto &cluster_indice : cluster_indices) {
      if (cluster_possibly_dynamic[cluster_indice_seq]) {
        for (int indice : cluster_indice.indices) {
          pcl::PointXYZINormal p;
          p.x        = (*non_ground_points)[indice].x;
          p.y        = (*non_ground_points)[indice].y;
          p.z        = (*non_ground_points)[indice].z;
          p.normal_x = clusters_feature_vector_dynamic[cluster_dynamic_vector_seq]
                           .vx;  // Use color to store velocity
          p.normal_y  = clusters_feature_vector_dynamic[cluster_dynamic_vector_seq].vy;
          p.normal_z  = clusters_feature_vector_dynamic[cluster_dynamic_vector_seq].vz;
          p.intensity = clusters_feature_vector_dynamic[cluster_dynamic_vector_seq]
                            .intensity;  // For visualization. //
                                         // clusters_feature_vector_dynamic[cluster_indice_seq].v
                                         // / maximum_velocity;
          _input_cloud_with_velocity->push_back(p);
        }
        ++cluster_dynamic_vector_seq;
      }

      ++cluster_indice_seq;
    }
  }

  for (auto &static_point : static_points->points) {
    pcl::PointXYZINormal p;
    p.x         = static_point.x;
    p.y         = static_point.y;
    p.z         = static_point.z;
    p.normal_x  = 0.F;
    p.normal_y  = 0.F;
    p.normal_z  = 0.F;
    p.intensity = 0.F;
    _input_cloud_with_velocity->push_back(p);
  }

  clusters_feature_vector_dynamic_last = clusters_feature_vector_dynamic;
  // cout << "Velocity estimation done" << endl;
}

void DSPMapStaticV2::getVoxelPositionFromIndexPublic(const int &index,
                                                     float &    px,
                                                     float &    py,
                                                     float &    pz) const {
  static const int z_change_storage_taken = mp_.voxel_size_y_ * mp_.voxel_size_x_;
  static const int y_change_storage_taken = mp_.voxel_size_x_;

  int z_index         = index / z_change_storage_taken;
  int yx_left_indexes = index - z_index * z_change_storage_taken;
  int y_index         = yx_left_indexes / y_change_storage_taken;
  int x_index         = yx_left_indexes - y_index * y_change_storage_taken;

  static const float correction_x = -mp_.half_map_size_x_ + mp_.resolution_ * 0.5F;
  static const float correction_y = -mp_.half_map_size_y_ + mp_.resolution_ * 0.5F;
  static const float correction_z = -mp_.half_map_size_z_ + mp_.resolution_ * 0.5F;

  px = (float)x_index * mp_.resolution_ + correction_x;
  py = (float)y_index * mp_.resolution_ + correction_y;
  pz = (float)z_index * mp_.resolution_ + correction_z;
}

bool DSPMapStaticV2::getPointVoxelsIndexPublic(const float &px,
                                               const float &py,
                                               const float &pz,
                                               int &        index) {
  if (!isInMap(px, py, pz)) {
    return false;
  }
  auto x = (int)((px + mp_.half_map_size_x_) / mp_.resolution_);
  auto y = (int)((py + mp_.half_map_size_y_) / mp_.resolution_);
  auto z = (int)((pz + mp_.half_map_size_z_) / mp_.resolution_);
  index  = z * mp_.voxel_size_y_ * mp_.voxel_size_x_ + y * mp_.voxel_size_x_ + x;
  if (index < 0 || index >= VOXEL_NUM) {
    return false;
  }
  return true;
}

}  // namespace dsp_map