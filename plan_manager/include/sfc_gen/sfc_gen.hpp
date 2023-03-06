/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef SFC_GEN_HPP
#define SFC_GEN_HPP

#include "sfc_gen/firi.hpp"

#include <Eigen/Eigen>
#include <deque>
#include <memory>

namespace sfc_gen {

inline void genSingleCvxCover(const Eigen::Vector3d &             start_pos,
                              const Eigen::Vector3d &             end_pos,
                              const std::vector<Eigen::Vector3d> &points,
                              const Eigen::Vector3d &             lowCorner,
                              const Eigen::Vector3d &             highCorner,
                              const double &                      progress,
                              const double &                      range,
                              Eigen::MatrixX4d &                  hp,
                              const double                        eps = 1.0e-6) {
  Eigen::Matrix<double, 6, 4> bd = Eigen::Matrix<double, 6, 4>::Zero();

  /* initialize bounding box */
  bd(0, 0) = 1.0;
  bd(1, 0) = -1.0;
  bd(2, 1) = 1.0;
  bd(3, 1) = -1.0;
  bd(4, 2) = 1.0;
  bd(5, 2) = -1.0;

  bd(0, 3) = -std::min(std::max(start_pos(0), end_pos(0)) + range, highCorner(0));
  bd(1, 3) = +std::max(std::min(start_pos(0), end_pos(0)) - range, lowCorner(0));
  bd(2, 3) = -std::min(std::max(start_pos(1), end_pos(1)) + range, highCorner(1));
  bd(3, 3) = +std::max(std::min(start_pos(1), end_pos(1)) - range, lowCorner(1));
  bd(4, 3) = -std::min(std::max(start_pos(2), end_pos(2)) + range, highCorner(2));
  bd(5, 3) = +std::max(std::min(start_pos(2), end_pos(2)) - range, lowCorner(2));

  std::vector<Eigen::Vector3d> valid_pc;
  valid_pc.reserve(points.size());
  valid_pc.clear();
  for (const Eigen::Vector3d &p : points) {  // TODO: can be merged with obstacle points extraction
    if ((bd.leftCols<3>() * p + bd.rightCols<1>()).maxCoeff() < 0.0) {
      valid_pc.emplace_back(p);
    }
  }
  Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pc(valid_pc[0].data(), 3,
                                                                     valid_pc.size());
  // std::cout << "[SFC] bounding box: " << std::endl << bd << std::endl;
  // std::cout << "[SFC] valid_pc size: " << valid_pc.size() << std::endl;
  // std::cout << "[SFC] a: " << a.transpose() << std::endl;
  // std::cout << "[SFC] b: " << b.transpose() << std::endl;
  firi::firi(bd, pc, start_pos, end_pos, hp);

  // std::cout << "[SFC] hp size: " << hp.rows() << std::endl;
  // std::cout << hp << std::endl;
}
/**
 * @brief
 * generate SFC from a given path and point clouds
 * The generated SFCs are stored in a vector, for each SFC, it is a Matrix,
 * each row is a constraint in H-representation.
 *
 * ax + by + cz + d <= 0
 *
 * @param path path represented by a vector of Eigen::Vector3d
 * @param points obstacle point clouds represented by a vector of Eigen::Vector3d
 * @param lowCorner the lower corner of the map
 * @param highCorner the higher corner of the map
 * @param progress
 * @param range
 * @param hpolys return the generated hpolys in H-representation
 * @param eps
 */
inline void convexCover(const std::vector<Eigen::Vector3d> &path,
                        const std::vector<Eigen::Vector3d> &points,
                        const Eigen::Vector3d &             lowCorner,
                        const Eigen::Vector3d &             highCorner,
                        const double &                      progress,
                        const double &                      range,
                        std::vector<Eigen::MatrixX4d> &     hpolys,
                        const double                        eps = 1.0e-6) {
  // std::cout << "lowCorner: " << lowCorner.transpose() << std::endl;
  // std::cout << "highCorner: " << highCorner.transpose() << std::endl;
  hpolys.clear();
  const int                   n  = path.size();
  Eigen::Matrix<double, 6, 4> bd = Eigen::Matrix<double, 6, 4>::Zero();

  /* initialize bounding box */
  bd(0, 0) = 1.0;
  bd(1, 0) = -1.0;
  bd(2, 1) = 1.0;
  bd(3, 1) = -1.0;
  bd(4, 2) = 1.0;
  bd(5, 2) = -1.0;

  /* Debug */
  // for (auto &pt : path) {
  // std::cout << "path: " << pt[1] << std::endl;
  // }

  Eigen::MatrixX4d             hp, gap;
  Eigen::Vector3d              a, b = path[0];
  std::vector<Eigen::Vector3d> valid_pc;
  std::vector<Eigen::Vector3d> bs;
  valid_pc.reserve(points.size());
  for (int i = 1; i < n;) {
    // std::cout << "====== i: " << i << " ======" << std::endl;
    a = b;
    if ((a - path[i]).norm() > progress) {
      b = (path[i] - a).normalized() * progress + a;
    } else {
      b = path[i];
      i++;
    }
    bs.emplace_back(b);

    bd(0, 3) = -std::min(std::max(a(0), b(0)) + range, highCorner(0));
    bd(1, 3) = +std::max(std::min(a(0), b(0)) - range, lowCorner(0));
    bd(2, 3) = -std::min(std::max(a(1), b(1)) + range, highCorner(1));
    bd(3, 3) = +std::max(std::min(a(1), b(1)) - range, lowCorner(1));
    bd(4, 3) = -std::min(std::max(a(2), b(2)) + range, highCorner(2));
    bd(5, 3) = +std::max(std::min(a(2), b(2)) - range, lowCorner(2));

    valid_pc.clear();
    for (const Eigen::Vector3d &p : points) {
      if ((bd.leftCols<3>() * p + bd.rightCols<1>()).maxCoeff() < 0.0) {
        valid_pc.emplace_back(p);
      }
    }
    Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pc(valid_pc[0].data(), 3,
                                                                       valid_pc.size());
    // std::cout << "[SFC] bounding box: " << std::endl << bd << std::endl;
    // std::cout << "[SFC] valid_pc size: " << valid_pc.size() << std::endl;
    // std::cout << "[SFC] a: " << a.transpose() << std::endl;
    // std::cout << "[SFC] b: " << b.transpose() << std::endl;
    firi::firi(bd, pc, a, b, hp);

    // std::cout << "[SFC] hp size: " << hp.rows() << std::endl;
    // std::cout << hp << std::endl;

    if (hpolys.size() != 0) {
      const Eigen::Vector4d ah(a(0), a(1), a(2), 1.0);
      if (3 <= ((hp * ah).array() > -eps).cast<int>().sum() +
                   ((hpolys.back() * ah).array() > -eps).cast<int>().sum()) {
        firi::firi(bd, pc, a, a, gap, 1);
        hpolys.emplace_back(gap);
      }
    }

    hpolys.emplace_back(hp);
  }
}

inline bool overlap(const Eigen::MatrixX4d &hPoly0,
                    const Eigen::MatrixX4d &hPoly1,
                    const double            eps = 1.0e-6)

{
  const int        m = hPoly0.rows();
  const int        n = hPoly1.rows();
  Eigen::MatrixX4d A(m + n, 4);
  Eigen::Vector4d  c, x;
  Eigen::VectorXd  b(m + n);
  A.leftCols<3>().topRows(m)    = hPoly0.leftCols<3>();
  A.leftCols<3>().bottomRows(n) = hPoly1.leftCols<3>();
  A.rightCols<1>().setConstant(1.0);
  b.topRows(m)    = -hPoly0.rightCols<1>();
  b.bottomRows(n) = -hPoly1.rightCols<1>();
  c.setZero();
  c(3) = -1.0;

  const double minmaxsd = sdlp::linprog<4>(c, A, b, x);

  return minmaxsd < -eps && !std::isinf(minmaxsd);
}

inline void shortCut(std::vector<Eigen::MatrixX4d> &hpolys) {
  std::vector<Eigen::MatrixX4d> htemp = hpolys;
  if (htemp.size() == 1) {
    Eigen::MatrixX4d headPoly = htemp.front();
    htemp.insert(htemp.begin(), headPoly);
  }
  hpolys.clear();

  int              M = htemp.size();
  Eigen::MatrixX4d hPoly;
  bool             is_overlap;
  std::deque<int>  idices;
  idices.push_front(M - 1);
  for (int i = M - 1; i >= 0; i--) {
    for (int j = 0; j < i; j++) {
      if (j < i - 1) {
        is_overlap = overlap(htemp[i], htemp[j], 0.01);
      } else {
        is_overlap = true;
      }
      if (is_overlap) {
        idices.push_front(j);
        i = j + 1;
        break;
      }
    }
  }
  for (const auto &ele : idices) {
    hpolys.push_back(htemp[ele]);
  }
}

}  // namespace sfc_gen

#endif
