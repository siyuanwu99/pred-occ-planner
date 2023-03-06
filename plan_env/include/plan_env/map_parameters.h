#ifndef DSP_DYNAMIC_PARMETERS_H
#define DSP_DYNAMIC_PARMETERS_H

/** Parameters for the map **/
#define MAP_LENGTH_VOXEL_NUM       66
#define MAP_WIDTH_VOXEL_NUM        66
#define MAP_HEIGHT_VOXEL_NUM       40
#define VOXEL_RESOLUTION           0.15
#define ANGLE_RESOLUTION           3
#define MAX_PARTICLE_NUM_VOXEL     9  // 20
#define LIMIT_MOVEMENT_IN_XY_PLANE 1

#define PREDICTION_TIMES 9
/* static const float prediction_future_time[PREDICTION_TIMES] = {
    0.05f, 0.2f, 0.5f, 1.f,
    1.5f,  2.f};  // unit: second. The first value is used to compensate the delay caused by the */
// map.

static const float prediction_future_time[PREDICTION_TIMES] = {0.2f, 0.4f, 0.6f, 0.8f, 1.0f,
                                                               1.2f, 1.4f, 1.6f, 1.8f};
const int half_fov_h = 42;  // can be divided by ANGLE_RESOLUTION. If not, modify ANGLE_RESOLUTION
                            // or make half_fov_h a smaller value than the real FOV angle
const int half_fov_v = 24;  // can be divided by ANGLE_RESOLUTION. If not, modify ANGLE_RESOLUTION
                            // or make half_fov_h a smaller value than the real FOV angle

#define DYNAMIC_CLUSTER_MAX_POINT_NUM 200
// Pre-velocity estimation parameter. Cluster with too many points will be allocated with a
// zero velocity.
#define DYNAMIC_CLUSTER_MAX_CENTER_HEIGHT 1.5
// Pre-velocity estimation parameter. Cluster with too high center will be allocated with a
// zero velocity.

#define GAUSSIAN_RANDOMS_NUM 10000000

#define O_MAKE_VALID   1  // use |= operator
#define O_MAKE_INVALID 0  // use &= operator
/** END **/

static const int observation_pyramid_num_h = (int)half_fov_h * 2 / ANGLE_RESOLUTION;
static const int observation_pyramid_num_v = (int)half_fov_v * 2 / ANGLE_RESOLUTION;
static const int observation_pyramid_num   = observation_pyramid_num_h * observation_pyramid_num_v;

static const int VOXEL_NUM   = MAP_LENGTH_VOXEL_NUM * MAP_WIDTH_VOXEL_NUM * MAP_HEIGHT_VOXEL_NUM;
static const int PYRAMID_NUM = 360 * 180 / ANGLE_RESOLUTION / ANGLE_RESOLUTION;
static const int SAFE_PARTICLE_NUM         = VOXEL_NUM * MAX_PARTICLE_NUM_VOXEL + 1e5;
static const int SAFE_PARTICLE_NUM_VOXEL   = MAX_PARTICLE_NUM_VOXEL * 2;
static const int SAFE_PARTICLE_NUM_PYRAMID = SAFE_PARTICLE_NUM / PYRAMID_NUM * 2;

// An estimated number. If the observation points are too dense (over 100 points in one pyramid),
// the overflowed points will be ignored. It is suggested to use a voxel filter to the original
// point cloud.
static const int   observation_max_points_num_one_pyramid = 100;
static const float obstacle_thickness_for_occlusion       = 0.3;

#endif  // DSP_DYNAMIC_PARMETERS_H
