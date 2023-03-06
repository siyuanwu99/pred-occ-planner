#include <gtest/gtest.h>

#include <Eigen/Eigen>
#include <iostream>

#include "traj_utils/bernstein.hpp"

namespace Bernstein {
class BernsteinTest : public ::testing::Test {
 protected:
  BernsteinTest() { std::cout << "BernsteinTest" << std::endl; }
  ~BernsteinTest() {}
  virtual void SetUp() override {
    std::cout << "enter into SetUp()" << std::endl;
    Eigen::MatrixXd cpts(5, 3);
    cpts << 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4;
    bp = BernsteinPiece(cpts, 2, 4);
  }
  void           TearDown() override { std::cout << "exit from TearDown" << std::endl; }
  BernsteinPiece bp;
};

class BezierTest : public ::testing::Test {
 protected:
  BezierTest() { std::cout << "BezierTest" << std::endl; }
  ~BezierTest() {}
  virtual void SetUp() override {
    std::cout << "Bezier Test: Setup()" << std::endl;
    std::vector<double> ts = {1, 2, 3};
    Eigen::MatrixXd     cpts(15, 3);
    cpts <<
        // clang-format off
    0, 0, 0,
    0.5, 0.5, 0.5,
    1, 1, 1,
    2, 2, 2,
    4, 4, 4,

    4, 4, 4,
    4, 5, 4,
    5, 4, 4,
    5, 6, 4,
    6, 5, 4,

    6, 5, 4,
    7, 5, 4,
    7, 6, 4,
    8, 5, 4,
    8, 6, 4;
    // clang-format on
    bc = Bezier(ts, cpts);
  }
  virtual void TearDown() override {}

  Bezier bc;
};

TEST_F(BernsteinTest, TestCase1) {
  EXPECT_EQ(bp.getDuration(), 2);

  EXPECT_EQ(bp.getPos(2), Eigen::Vector3d::Zero());
  std::cout << bp.getVel(2).transpose() << std::endl;
  EXPECT_EQ(bp.getVel(2), Eigen::Vector3d(2, 2, 2));
  EXPECT_EQ(bp.getAcc(2), Eigen::Vector3d(0, 0, 0));
  EXPECT_EQ(bp.getPos(4), Eigen::Vector3d(4, 4, 4));
}

TEST_F(BernsteinTest, TestControlPoints) {
  Eigen::MatrixXd A;
  A.resize(5, 5);
  A << 1, -4, 6, -4, 1, 0, 4, -12, 12, -4, 0, 0, 6, -12, 6, 0, 0, 0, 4, -4, 0, 0, 0, 0, 1;
  EXPECT_EQ(bp.getCoeffMat(), A);

  Eigen::MatrixXd v_cpts(4, 3);
  v_cpts << 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4;
  std::cout << bp.getVelCtrlPts() << std::endl;
  EXPECT_EQ(bp.getVelCtrlPts(), v_cpts);

  Eigen::MatrixXd a_cpts(3, 3);
  a_cpts << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  EXPECT_EQ(bp.getAccCtrlPts(), a_cpts);
}

TEST_F(BezierTest, TestTraj) {
  EXPECT_DOUBLE_EQ(bc.getDuration(), 6);
  EXPECT_EQ(3, bc.getNumPieces());

  for (double t = 0; t < bc.getDuration(); t += 0.1) {
    Eigen::Vector3d p, v, a;
    p = bc.getPos(t);
    v = bc.getVel(t);
    a = bc.getAcc(t);
    std::cout << "t: " << t << "\tp: " << p.transpose() << "\tv: " << v.transpose()
              << "\ta: " << a.transpose() << std::endl;
  }
}

TEST_F(BezierTest, TestBezierMax) {
  auto v_cpts = bc[1].getVelCtrlPts();
  std::cout << "v_cpts: " << std::endl << v_cpts << std::endl;
  auto a_cpts = bc[1].getAccCtrlPts();
  std::cout << "a_cpts: " << std::endl << a_cpts << std::endl;
  double max_vel = bc.getMaxVelRate();
  std::cout << "max_vel: " << max_vel;
  double max_acc = bc.getMaxAccRate();
  std::cout << ", max_acc: " << max_acc << std::endl;
  EXPECT_TRUE(max_vel > 1.5);
}

}  // namespace Bernstein

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
