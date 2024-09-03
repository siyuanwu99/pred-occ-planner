/**
 * @file visualizer.h
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief a better visualization with ground truth data
 * @version 1.0
 * @date 2023-09-22
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef VISUALIZER_H
#define VISUALIZER_H
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <Eigen/Dense>
#include <string>
namespace visualizer {
/**
 * @brief get color from jet colormap
 *
 * @param v current value
 * @param vmin min value
 * @param vmax max value
 * @return std_msgs::ColorRGBA
 */
inline std_msgs::ColorRGBA getColorJet(double v, double vmin, double vmax) {
  std_msgs::ColorRGBA c;
  c.r = 1;
  c.g = 1;
  c.b = 1;
  c.a = 1;
  // white
  double dv;

  if (v < vmin) v = vmin;
  if (v > vmax) v = vmax;
  dv = vmax - vmin;

  if (v < (vmin + 0.25 * dv)) {
    c.r = 0;
    c.g = 4 * (v - vmin) / dv;
  } else if (v < (vmin + 0.5 * dv)) {
    c.r = 0;
    c.b = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
  } else if (v < (vmin + 0.75 * dv)) {
    c.r = 4 * (v - vmin - 0.5 * dv) / dv;
    c.b = 0;
  } else {
    c.g = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
    c.b = 0;
  }

  return (c);
}

inline std_msgs::ColorRGBA getColorG2B(double v, double vmin, double vmax) {
  std_msgs::ColorRGBA c;
  c.r = 0;  // No red component for green-blue transition
  c.a = 1;  // Assuming you always want full opacity

  double dv;

  if (v < vmin) v = vmin;
  if (v > vmax) v = vmax;
  dv = vmax - vmin;

  // If you strictly want to transition from green to blue, the color calculation is simpler
  if (v < (vmin + 0.5 * dv)) {
    c.g = 1;
    c.b = 2 * (v - vmin) / dv;
  } else {
    c.g = 1 - 2 * (v - vmin - 0.5 * dv) / dv;
    c.b = 1;
  }

  return c;
}

}  // namespace visualizer
#endif /* VISUALIZER_H */
