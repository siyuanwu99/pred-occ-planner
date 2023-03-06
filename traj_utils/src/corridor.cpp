/**
 * @file corridor.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2022-09-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <traj_utils/corridor.hpp>

namespace traj_utils {

void cvtPolytopeNormal2H(const std::vector<Eigen::Matrix<double, 6, -1>> &c,
                    std::vector<Eigen::MatrixX4d> &                  h) {
  for (int i = 0; i < static_cast<int>(c.size()); i++) {
    Eigen::MatrixX4d h_i(6, 4);
    for (int j = 0; j < 6; j++) {
      Eigen::Vector3d dir, pos;
      dir = c[i].col(j).head<3>();
      pos = c[i].col(j).tail<3>();
      h_i.row(j) << dir.transpose(), -dir.dot(pos);
    }
    h.push_back(h_i);
  }
}

}