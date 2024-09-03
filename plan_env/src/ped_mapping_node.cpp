/**
 * @file ped_voxel.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief map pedestrians to occupancy prediction map (voxel grid) using optitrack odometry
 *
 *  input: /pedestrian1/optitrack/pose
 *         /drone1/local_position/pose
 *  output: /map/occupancy_inflated
 *          /map/future_risk
 *          /map/time
 *
 * @version 1.0
 * @date 2022-10-23
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <plan_env/map_parameters.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <deque>
#include <vector>
#include "gazebo_msgs/ModelStates.h"
#include "std_msgs/Float32MultiArray.h"

constexpr double LOCAL_RANGE_X = MAP_LENGTH_VOXEL_NUM / 2.f * VOXEL_RESOLUTION;
constexpr double LOCAL_RANGE_Y = MAP_WIDTH_VOXEL_NUM / 2.f * VOXEL_RESOLUTION;
constexpr double LOCAL_RANGE_Z = MAP_HEIGHT_VOXEL_NUM / 2.f * VOXEL_RESOLUTION;

/** parameters */
/**
 * @brief read parameters from launch file
 *
 * @param nh node handle
 */
struct Params {
  bool        is_pose_sub_;
  std::string map_frame_id_;
  std::string ped_occu_file_name_;
  explicit Params(const ros::NodeHandle &nh) {
    nh.param("map/booleans/sub_pose", is_pose_sub_, false);
    nh.param("map/frame_id", map_frame_id_, std::string("world"));
    nh.param("map/ped_voxel_file_name", ped_occu_file_name_, std::string("pedstrians.yaml"));
  }
};

class PedestriansMapper {
 private:
  /** map */
  float risk_maps_[VOXEL_NUM][PREDICTION_TIMES];

  Eigen::Vector3f    pose_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f    vel_  = Eigen::Vector3f::Zero();
  Eigen::Quaternionf q_    = Eigen::Quaternionf::Identity();

  /** pedestrian state */
  Eigen::Matrix3f ped1_state_ = Eigen::Matrix3f::Zero();  // [pos; vel; acc]
  Eigen::Matrix3f ped2_state_ = Eigen::Matrix3f::Zero();  // [pos; vel; acc]
  Eigen::Matrix3f ped3_state_ = Eigen::Matrix3f::Zero();  // [pos; vel; acc]
  std::deque<std::pair<Eigen::Vector3f, double>> ped1_pos_buffer_;
  std::deque<std::pair<Eigen::Vector3f, double>> ped2_pos_buffer_;
  std::deque<std::pair<Eigen::Vector3f, double>> ped3_pos_buffer_;
  std::vector<Eigen::Vector3f>                   ped1_occu_voxels_;
  std::vector<Eigen::Vector3f>                   ped2_occu_voxels_;
  std::vector<Eigen::Vector3f>                   ped3_occu_voxels_;
  std::vector<Eigen::Vector3f>                   other_occu_voxels_;
  /** ROS */
  ros::NodeHandle nh1_, nh2_;
  ros::Publisher  cloud_pub_, future_risk_pub_, risk_2dmap_pub_, time_pub_;
  ros::Subscriber predestrian1_sub_, predestrian2_sub_, predestrian3_sub_, pose_sub_, odom_sub_;
  ros::Timer      map_update_timer_;
  ros::Time       last_update_time_;

  /** Parameters */
  Params params_;

 public:
  PedestriansMapper(const ros::NodeHandle &nh1, const ros::NodeHandle &nh2, Params params)
      : nh1_(nh1), nh2_(nh2), params_(params) {}

  /**
   * @brief loal occupancy from yaml file
   *
   * @param ped_name file name of yaml file
   * @return std::vector<Eigen::Vector3f> occupancy voxels
   */
  std::vector<Eigen::Vector3f> loadOccupancyFromYAML(const std::string &ped_name) {
    std::vector<Eigen::Vector3f> particles;
    ROS_INFO("Loading particles from %s", ped_name.c_str());

    XmlRpc::XmlRpcValue particle_list;
    if (!nh1_.getParam(ped_name, particle_list)) {
      ROS_ERROR("Failed to get param '%s'", ped_name.c_str());
      return particles;
    }

    if (particle_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      ROS_ERROR("Particles is not a list");
      return particles;
    }

    for (int32_t i = 0; i < particle_list.size(); ++i) {
      if (!particle_list[i].hasMember("x") || !particle_list[i].hasMember("y") ||
          !particle_list[i].hasMember("z")) {
        ROS_ERROR("Particle %d does not contain x, y or z fields", i);
        continue;
      }

      Eigen::Vector3f voxel;
      voxel[0] = static_cast<double>(particle_list[i]["x"]);
      voxel[1] = static_cast<double>(particle_list[i]["y"]);
      voxel[2] = static_cast<double>(particle_list[i]["z"]);
      particles.push_back(voxel);
    }
    return particles;
  }

  /**
   * @brief update occupancy prediction map
   *
   * @param cloud_in pointer to input cloud
   */
  void mapUpdateCallback(const ros::TimerEvent &event) {
    ROS_INFO("[PedMap] Updating map");
    /* update point cloud */
    pcl::PointCloud<pcl::PointXYZ> cloud;  // point cloud in the world frame
    Eigen::Vector3f                pose          = pose_;
    Eigen::Vector3f                ped1_position = ped1_state_.col(0);
    Eigen::Vector3f                ped2_position = ped2_state_.col(0);
    Eigen::Vector3f                ped3_position = ped3_state_.col(0);

    for (auto &p : ped1_occu_voxels_) {
      Eigen::Vector3f p_world = p + ped1_position;
      if (isInRange(p_world - pose))
        cloud.push_back(pcl::PointXYZ(p_world.x(), p_world.y(), p_world.z()));
    }
    for (auto &p : ped2_occu_voxels_) {
      Eigen::Vector3f p_world = p + ped2_position;
      if (isInRange(p_world - pose))
        cloud.push_back(pcl::PointXYZ(p_world.x(), p_world.y(), p_world.z()));
    }
    for (auto &p : ped3_occu_voxels_) {
      Eigen::Vector3f p_world = p + ped3_position;
      if (isInRange(p_world - pose))
        cloud.push_back(pcl::PointXYZ(p_world.x(), p_world.y(), p_world.z()));
    }
    /* add static obstacles to map */
    for (auto &p : other_occu_voxels_) {
      if (isInRange(p - pose)) cloud.push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
    }

    for (float x = pose.x() - LOCAL_RANGE_X; x < pose.y() + LOCAL_RANGE_X; x += VOXEL_RESOLUTION) {
      for (float y = pose.x() - LOCAL_RANGE_Y; y < pose.y() + LOCAL_RANGE_Y;
           y += VOXEL_RESOLUTION) {
        Eigen::Vector3f p_world = Eigen::Vector3f(x, y, 0);
        if (isInRange(p_world - pose)) {
          cloud.push_back(pcl::PointXYZ(p_world.x(), p_world.y(), p_world.z()));
        }
      }
    }

    cloud.width  = cloud.size();
    cloud.height = 1;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.stamp    = ros::Time::now();
    cloud_msg.header.frame_id = "world";
    cloud_pub_.publish(cloud_msg);

    /* initialize risk map */
    for (int i = 0; i < VOXEL_NUM; ++i) {
      if (indexToWorldZ(i) < 0.0 && indexToWorldZ(i) > -0.15) {
        for (int j = 0; j < PREDICTION_TIMES; ++j) {
          risk_maps_[i][j] = 1;
        }
      } else {
        for (int j = 0; j < PREDICTION_TIMES; ++j) {
          risk_maps_[i][j] = 0;
        }
      }
    }

    /* update risk map */
    std::vector<Eigen::Vector3f> ped1_prediction_map;  // pedestrian prediction in map frame
    ped1_prediction_map.resize(PREDICTION_TIMES + 1);
    ped1_prediction_map[0] = ped1_position - pose;
    for (int i = 1; i < PREDICTION_TIMES + 1; i++) {
      ped1_prediction_map[i] =
          ped1_position - pose + ped1_state_.col(1) * prediction_future_time[i - 1];
    }
    std::vector<Eigen::Vector3f> ped2_prediction_map;  // pedestrian prediction in map frame
    ped2_prediction_map.resize(PREDICTION_TIMES + 1);
    ped2_prediction_map[0] = ped2_position - pose;
    for (int i = 1; i < PREDICTION_TIMES + 1; i++) {
      ped2_prediction_map[i] =
          ped2_position - pose + ped2_state_.col(1) * prediction_future_time[i - 1];
    }
    std::vector<Eigen::Vector3f> ped3_prediction_map;  // pedestrian prediction in map frame
    ped3_prediction_map.resize(PREDICTION_TIMES + 1);
    ped3_prediction_map[0] = ped3_position - pose;
    for (int i = 1; i < PREDICTION_TIMES + 1; i++) {
      ped3_prediction_map[i] =
          ped3_position - pose + ped3_state_.col(1) * prediction_future_time[i - 1];
    }

    for (int i = 0; i < PREDICTION_TIMES; i++) {
      for (auto &pcv : ped1_occu_voxels_) {
        Eigen::Vector3f p_map_i = pcv + ped1_prediction_map[i];
        if (isInRange(p_map_i)) {
          risk_maps_[getIndex(p_map_i)][i] = 1;
        }
      }
    }
    for (int i = 0; i < PREDICTION_TIMES; i++) {
      for (auto &pcv : ped2_occu_voxels_) {
        Eigen::Vector3f p_map_i = pcv + ped2_prediction_map[i];
        if (isInRange(p_map_i)) {
          risk_maps_[getIndex(p_map_i)][i] = 1;
        }
      }
    }

    for (int i = 0; i < PREDICTION_TIMES; i++) {
      for (auto &pcv : ped3_occu_voxels_) {
        Eigen::Vector3f p_map_i = pcv + ped3_prediction_map[i];

        if (isInRange(p_map_i)) {
          risk_maps_[getIndex(p_map_i)][i] = 1;
        }
      }
    }

    for (int i = 0; i < PREDICTION_TIMES; i++) {
      for (auto &pcv : other_occu_voxels_) {
        Eigen::Vector3f p = pcv - pose;
        if (isInRange(p)) {
          risk_maps_[getIndex(p)][i] = 1;
        }
      }
    }
    /* publish map */
    clock_t t1 = clock();

    /** publish future risks */
    std_msgs::Float32MultiArray   future_risk_array_msg;
    std_msgs::MultiArrayDimension future_risk_array_dimension;
    future_risk_array_dimension.size   = VOXEL_NUM;
    future_risk_array_dimension.stride = PREDICTION_TIMES;
    future_risk_array_msg.layout.dim.push_back(future_risk_array_dimension);
    future_risk_array_msg.data.reserve(VOXEL_NUM * PREDICTION_TIMES + 4);
    for (int i = 0; i < VOXEL_NUM; ++i) {
      for (int j = 0; j < PREDICTION_TIMES; ++j) {
        future_risk_array_msg.data.push_back(risk_maps_[i][j]);
      }
    }
    future_risk_array_msg.data.push_back(pose_.x());
    future_risk_array_msg.data.push_back(pose_.y());
    future_risk_array_msg.data.push_back(pose_.z());
    // add current time
    future_risk_array_msg.data.push_back(ros::Time::now().toSec());
    future_risk_pub_.publish(future_risk_array_msg);

    std_msgs::Header time_msg;
    time_msg.stamp = ros::Time::now();
    time_pub_.publish(time_msg);

    clock_t t2 = clock();

    // std::cout << "publish time (ms): " << (t2 - t1) * 1000 / static_cast<double> CLOCKS_PER_SEC
    //           << std::endl;
  }

  /**
   * @brief pose callback
   *
   * @param msg pose message
   */
  void poseCallback(const geometry_msgs::PoseStamped &msg) {
    pose_ << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
    q_.x() = msg.pose.orientation.x;
    q_.y() = msg.pose.orientation.y;
    q_.z() = msg.pose.orientation.z;
    q_.w() = msg.pose.orientation.w;
  }

  /**
   * @brief odometry callback
   *
   * @param msg odometry message
   */
  void odomCallback(const nav_msgs::Odometry &msg) {
    pose_ << msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z;
    q_.x() = msg.pose.pose.orientation.x;
    q_.y() = msg.pose.pose.orientation.y;
    q_.z() = msg.pose.pose.orientation.z;
    q_.w() = msg.pose.pose.orientation.w;
    vel_ << msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z;
  }

  /**
   * @brief pedestrian pose callback
   *
   * @param msg pedestrian pose message, received from optitrack
   */
  void ped1PoseCallback(const geometry_msgs::PoseStamped &msg) {
    Eigen::Vector3f ped_pos, ped_vel, ped_acc;
    ped_pos << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;

    double t = msg.header.stamp.toSec();

    std::pair<Eigen::Vector3f, double> ped_10p_pair = ped1_pos_buffer_.front();
    Eigen::Vector3f                    pos_10p      = ped_10p_pair.first;
    double                             t_10p        = ped_10p_pair.second;
    if (ped1_pos_buffer_.size() == 10) ped1_pos_buffer_.pop_front();
    ped1_pos_buffer_.push_back(std::make_pair(ped1_state_.col(0), t));

    ped_vel     = (ped_pos - pos_10p) / (t - t_10p);
    ped_vel.z() = 0;
    ped_acc     = Eigen::Vector3f::Zero();
    ped1_state_ << ped_pos, ped_vel, ped_acc;
  }

  void ped2PoseCallback(const geometry_msgs::PoseStamped &msg) {
    Eigen::Vector3f ped_pos, ped_vel, ped_acc;
    ped_pos << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;

    double t = msg.header.stamp.toSec();

    std::pair<Eigen::Vector3f, double> ped_10p_pair = ped2_pos_buffer_.front();
    Eigen::Vector3f                    pos_10p      = ped_10p_pair.first;
    double                             t_10p        = ped_10p_pair.second;
    if (ped2_pos_buffer_.size() == 10) ped2_pos_buffer_.pop_front();
    ped2_pos_buffer_.push_back(std::make_pair(ped2_state_.col(0), t));

    ped_vel     = (ped_pos - pos_10p) / (t - t_10p);
    ped_vel.z() = 0;
    ped_acc     = Eigen::Vector3f::Zero();
    ped2_state_ << ped_pos, ped_vel, ped_acc;
  }
  void ped3PoseCallback(const geometry_msgs::PoseStamped &msg) {
    Eigen::Vector3f ped_pos, ped_vel, ped_acc;
    ped_pos << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;

    double t = msg.header.stamp.toSec();

    std::pair<Eigen::Vector3f, double> ped_10p_pair = ped3_pos_buffer_.front();
    Eigen::Vector3f                    pos_10p      = ped_10p_pair.first;
    double                             t_10p        = ped_10p_pair.second;
    if (ped3_pos_buffer_.size() == 10) ped3_pos_buffer_.pop_front();
    ped3_pos_buffer_.push_back(std::make_pair(ped3_state_.col(0), t));

    ped_vel     = (ped_pos - pos_10p) / (t - t_10p);
    ped_vel.z() = 0;
    ped_acc     = Eigen::Vector3f::Zero();
    ped3_state_ << ped_pos, ped_vel, ped_acc;
  }

  void init() {
    predestrian1_sub_ = nh1_.subscribe("/pedestrian1/optitrack/pose", 100,
                                       &PedestriansMapper::ped1PoseCallback, this);
    predestrian2_sub_ = nh1_.subscribe("/pedestrian2/optitrack/pose", 100,
                                       &PedestriansMapper::ped2PoseCallback, this);
    predestrian3_sub_ = nh1_.subscribe("/pedestrian3/optitrack/pose", 100,
                                       &PedestriansMapper::ped3PoseCallback, this);
    // ped1_state_ << -10 * Eigen::Vector3f::Ones(), Eigen::Vector3f::Zero(),
    // Eigen::Vector3f::Zero(); ped2_state_ << -10 * Eigen::Vector3f::Ones(),
    // Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(); ped3_state_ << -10 *
    // Eigen::Vector3f::Ones(), Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero();

    if (!params_.is_pose_sub_) {
      odom_sub_ = nh1_.subscribe("map/odom", 100, &PedestriansMapper::odomCallback,
                                 this); /* use odometry */
    } else {
      pose_sub_ =
          nh1_.subscribe("map/pose", 100, &PedestriansMapper::poseCallback, this); /* use pose */
    }

    future_risk_pub_ = nh2_.advertise<std_msgs::Float32MultiArray>("map/future_risk", 1, true);
    cloud_pub_       = nh2_.advertise<sensor_msgs::PointCloud2>("map/occupancy_inflated", 1, true);
    time_pub_        = nh2_.advertise<std_msgs::Header>("map/time", 1, true);

    ped1_occu_voxels_  = loadOccupancyFromYAML("pedestrian1");
    ped2_occu_voxels_  = loadOccupancyFromYAML("pedestrian2");
    ped3_occu_voxels_  = loadOccupancyFromYAML("pedestrian3");
    other_occu_voxels_ = loadOccupancyFromYAML("static");
    ROS_INFO("[PedMap] Loaded %lu pedestrian voxels", ped1_occu_voxels_.size());
    ROS_INFO("[PedMap] Loaded %lu pedestrian voxels", ped2_occu_voxels_.size());
    ROS_INFO("[PedMap] Loaded %lu pedestrian voxels", ped3_occu_voxels_.size());
    ROS_INFO("[PedMap] Loaded %lu other objects", other_occu_voxels_.size());

    map_update_timer_ =
        nh2_.createTimer(ros::Duration(0.1), &PedestriansMapper::mapUpdateCallback, this);
    ROS_INFO("[PedMap] Init risk voxel map");
  }

  inline bool isInRange(const Eigen::Vector3f &p) {
    return p.x() > -LOCAL_RANGE_X && p.x() < LOCAL_RANGE_X && p.y() > -LOCAL_RANGE_Y &&
           p.y() < LOCAL_RANGE_Y && p.z() > -LOCAL_RANGE_Z && p.z() < LOCAL_RANGE_Z;
  }

  inline int getIndex(const Eigen::Vector3f &pf) {
    int x = (pf.x() + LOCAL_RANGE_X) / VOXEL_RESOLUTION;
    int y = (pf.y() + LOCAL_RANGE_Y) / VOXEL_RESOLUTION;
    int z = (pf.z() + LOCAL_RANGE_Z) / VOXEL_RESOLUTION;
    return x + y * MAP_LENGTH_VOXEL_NUM + z * MAP_LENGTH_VOXEL_NUM * MAP_WIDTH_VOXEL_NUM;
  }

  inline float indexToWorldZ(int index) {
    int z = index / (MAP_LENGTH_VOXEL_NUM * MAP_WIDTH_VOXEL_NUM);
    return z * VOXEL_RESOLUTION - LOCAL_RANGE_Z + pose_.z();
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "ped_voxel_node");
  ros::NodeHandle nh("~");
  ros::NodeHandle nh2("~");
  Params          params(nh);

  ros::CallbackQueue custom_queue1;
  ros::CallbackQueue custom_queue2;

  nh.setCallbackQueue(&custom_queue1);
  nh2.setCallbackQueue(&custom_queue2);

  PedestriansMapper ped_mapper(nh, nh2, params);
  ped_mapper.init();

  ros::AsyncSpinner spinner1(1, &custom_queue1);  // spinner for FSM
  ros::AsyncSpinner spinner2(1, &custom_queue2);  // spinner for planner
  spinner1.start();
  spinner2.start();
  ros::waitForShutdown();
  return 0;
}
