/**
 * @file rmader.hpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 *
 * Robust MADER pipeline:
 * 1. CHECK step
 * 2. broadcast trajectory passed CHECK, execute previous trajectory
 * 3. DELAY CHECK step
 * 4. broadcast trajectory passed delay CHECK
 *
 * @version 1.0
 * @date 2022-12-27
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef RMADER_H
#define RMADER_H

#include <ros/ros.h>
#include <traj_utils/BezierTraj.h>
#include <Eigen/Eigen>
#include <map>
#include <memory>
#include <queue>
#include <traj_utils/bernstein.hpp>
#include <vector>
#include "separator.hpp"
#include "traj_coordinator/mader.hpp"

typedef Bernstein::BernsteinPiece Piece;

// we should give some hyperparameter to control the size of the cube
// Assume our robot is homogenous, i.e. all robots have the same size

class RMADER : public MADER {
 public:
  /* constructor */
  using MADER::MADER;
  // RMADER(ros::NodeHandle &nh) : nh_(nh) {}
  // ~RMADER() {}

  /* functions */
  void init();
  bool collisionCheck(const Bernstein::Bezier &traj);
  bool isSafeAfterOpt(const Bernstein::Bezier &traj);
  bool isSafeAfterDelayCheck(const Bernstein::Bezier &traj);

  typedef std::shared_ptr<RMADER> Ptr;

 protected:
  /* Robust mader */
  double delta_dc_; /* length of delay check in RMADER */
};

#endif  // RMADER_DECONFLICTION_H
