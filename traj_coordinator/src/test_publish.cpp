/**
 * @file test_publisher.hpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2023-07-28
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <geometry_msgs/PolygonStamped.h>
#include <ros/ros.h>
#include <Eigen/Eigen>

void subCallback(const geometry_msgs::PolygonStampedConstPtr& msg) {
  int idx = std::stoi(msg->header.frame_id.substr(3, 1));
  ROS_INFO("[CA] received polygon id %d", idx);
  ROS_INFO("[CA] polygon size: %lu", msg->polygon.points.size());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_publisher");
  ros::NodeHandle nh;
  ros::Publisher  pub = nh.advertise<geometry_msgs::PolygonStamped>("test_polygon", 1);
  ros::Subscriber sub = nh.subscribe("test_polygon", 1, subCallback);

  std::vector<Eigen::Vector3d> points;
  points.clear();

  double WIDTH = 0.2;
  double STEP  = 0.1;
  /* body particles, we use a cube to approximate the body */
  for (double x = -WIDTH; x <= WIDTH; x += STEP) {
    for (double y = -WIDTH; y <= WIDTH; y += STEP) {
      for (double z = -WIDTH; z <= WIDTH; z += STEP) {
        Eigen::Vector3d pt(x, y, z);
        points.push_back(pt);
      }
    }
  }
  ROS_INFO("[CA] ego particles initialized with %lu particles", points.size());
  geometry_msgs::PolygonStamped poly;
  poly.header.frame_id = "uav0/base_link";
  poly.header.stamp    = ros::Time::now();
  poly.polygon.points.reserve(points.size());

  // geometry_msgs::Polygon poly;
  // poly.points.resize(points.size());

  for (auto& pt : points) {
    geometry_msgs::Point32 p;
    p.x = pt.x();
    p.y = pt.y();
    p.z = pt.z();
    poly.polygon.points.push_back(p);
    // poly.points.push_back(p);
  }

  ros::Rate rate(1);
  while (ros::ok()) {
    ROS_INFO("[CA] publishing polygon");
    pub.publish(poly);
    ros::spinOnce();
    rate.sleep();
  }
}
