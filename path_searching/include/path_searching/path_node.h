/**
 * This file based on Fast-Planner.
 *
 * Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology,
 * <uav.ust.hk> Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at
 * gmail dot com> for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * Fast-Planner is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Fast-Planner is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef _PATH_NODE_H_
#define _PATH_NODE_H_

#include <path_searching/grid_node.h>
#include <Eigen/Eigen>
#include <iostream>
#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>

class PathNode;
typedef PathNode* PathNodePtr;

class PathNode : public GridNode {
 public:
  /* -------------------- */
  PathNode*                   parent_{nullptr};
  Eigen::Vector3d             input;
  Eigen::Matrix<double, 6, 1> state;
  double                      duration;
  double                      time;  // dyn
  int                         time_idx;

  /* -------------------- */
  PathNode() {
    setParent(NULL);
    setNodeState(NOT_EXPAND);
  }
  ~PathNode() {}
  void             setParent(PathNode* parent) { parent_ = parent; }
  inline PathNode* getParent() const { return parent_; }
};
typedef PathNode* PathNodePtr;

template <typename T>
struct MatrixHash : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < (matrix.size()); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

class NodeHashTable {
 private:
  /* data */
  std::unordered_map<Eigen::Vector3i, PathNodePtr, MatrixHash<Eigen::Vector3i>> data_3d_;
  std::unordered_map<Eigen::Vector4i, PathNodePtr, MatrixHash<Eigen::Vector4i>> data_4d_;

 public:
  NodeHashTable(/* args */) {}
  ~NodeHashTable() {}
  void insert(Eigen::Vector3i idx, PathNodePtr node) { data_3d_.insert(std::make_pair(idx, node)); }
  void insert(Eigen::Vector3i idx, int time_idx, PathNodePtr node) {
    data_4d_.insert(std::make_pair(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx), node));
  }

  PathNodePtr find(Eigen::Vector3i idx) {
    auto iter = data_3d_.find(idx);
    return iter == data_3d_.end() ? NULL : iter->second;
  }
  PathNodePtr find(Eigen::Vector3i idx, int time_idx) {
    auto iter = data_4d_.find(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx));
    return iter == data_4d_.end() ? NULL : iter->second;
  }

  void clear() {
    data_3d_.clear();
    data_4d_.clear();
  }
};

#endif  // _PATH_NODE_H_
