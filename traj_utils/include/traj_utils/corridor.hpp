/**
 * @file corridor.hpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief Definition of convex corridors
 * @version 1.0
 * @date 2022-08-09
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __CORRIDOR_HPP__
#define __CORRIDOR_HPP__

#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <vector>

namespace traj_utils {

/**
 * @brief corridor is represented by a convex polyhedron
 * which is a set of hyperplanes, consist of points and normals
 * The first 3 elements are the normal vector, the last 3 elements are the point
 * on that plane.
 * The normal vectors point outward from the polyhedron.
 */
typedef Eigen::Matrix<double, 6, -1> Polyhedron;

/**
 * @brief corridors is a vector of polyhedrons
 */
typedef std::vector<Polyhedron> Corridors;

/**
 * @brief convert polytope representation
 *
 * normal vector representation:
 * n: normal vector, p: point on the plane
 *     |
 *    p| --> n    pointing outward
 *     |
 * every column is a hyperplane, [n, p]
 *
 * H-representation: x*h0 + y*h1 + z*h2 + h3 <= 0
 *
 * @param c polytope representation with normal vectors pointing outwards
 * @param h H-representation x*h0 + y*h1 + z*h2 + h3 <= 0
 */
void cvtPolytopeNormal2H(const std::vector<Eigen::Matrix<double, 6, -1>> &c,
                         std::vector<Eigen::MatrixX4d>                   &h);
}  // namespace traj_utils

#endif  // __CORRIDOR_HPP__