#include <gtest/gtest.h>

#include <Eigen/Eigen>
#include <iostream>

#include "bernstein/bezier_optimizer.hpp"
#include "traj_utils/bernstein.hpp"

namespace traj_opt {
class BezierOptTest : public ::testing::Test {
 protected:
  BezierOptTest() {
    _optimizer.reset(new BezierOpt());
    std::cout << "BezierOptTest" << std::endl;
  }
  ~BezierOptTest() {}
  virtual void SetUp() override {
    std::cout << "enter into SetUp()" << std::endl;
    /* set up constraints */
    Eigen::MatrixX4d cube;
    cube.resize(6, 4);
    cube <<
        // clang-format off
    -1, 0, 0, 0,
    1,  0, 0, -3,
    0, -1, 0, 0,
    0,  1, 0, -3,
    0,  0, -1, 0,
    0,  0, 1, -3;
    // clang-format on
    std::vector<Eigen::MatrixX4d> safety_corridors;
    safety_corridors.push_back(cube);
    Eigen::Matrix3d start, end;
    start << 1, 1, 1, 0, 0, 0, 0, 0, 0;
    end << 2, 2, 2, 0, 0, 0, 0, 0, 0;
    std::vector<double> t;
    t.push_back(2);

    _optimizer->setup(start, end, t, safety_corridors);
    _optimizer->calcCtrlPtsCvtMat();
  }

  void           TearDown() override { std::cout << "exit from TearDown" << std::endl; }
  BezierOpt::Ptr _optimizer;
};

class BezierOptTest2 : public ::testing::Test {
 protected:
  BezierOptTest2() {
    _optimizer.reset(new BezierOpt());
    std::cout << "BezierOptTest2" << std::endl;
  }
  ~BezierOptTest2() {}
  virtual void SetUp() override {
    std::cout << "enter into SetUp()" << std::endl;
    /* set up constraints */
    Eigen::MatrixX4d cube1, cube2, cube3;
    cube1.resize(6, 4);
    cube1 <<
        // clang-format off
    -1, 0, 0, 0,
    1,  0, 0, -3,
    0, -1, 0, 0,
    0,  1, 0, -3,
    0,  0, -1, 0,
    0,  0, 1, -3;
    // clang-format on
    cube2.resize(6, 4);
    cube2 <<
        // clang-format off
    -1, 0, 0, -2,
    1,  0, 0, -6,
    0, -1, 0, -2,
    0,  1, 0, -6,
    0,  0, -1, -2,
    0,  0, 1, -6;
    // clang-format on
    cube3.resize(6, 4);
    cube3 <<
        // clang-format off
    -1, 0, 0, -5,
    1,  0, 0, -9,
    0, -1, 0, -5,
    0,  1, 0, -9,
    0,  0, -1, -5,
    0,  0, 1, -9;
    // clang-format on

    std::vector<Eigen::MatrixX4d> safety_corridors;
    safety_corridors.push_back(cube1);
    safety_corridors.push_back(cube2);
    safety_corridors.push_back(cube3);

    start << 1, 0, 1, 0, 0, 0, 0, 0, 0;
    end << 8, 9, 8, 0.5, 1, 0, 0, 0, 0;
    std::vector<double> t;
    t.push_back(2);
    t.push_back(4.0);
    t.push_back(2.0);

    _optimizer->setup(start, end, t, safety_corridors);
    _optimizer->calcCtrlPtsCvtMat();
  }

  void            TearDown() override { std::cout << "exit from TearDown" << std::endl; }
  BezierOpt::Ptr  _optimizer;
  Eigen::Matrix3d start, end;
};

TEST_F(BezierOptTest, TestCvtMat) {
  Eigen::MatrixXd p2v, v2a, a2j;
  p2v = _optimizer->getPos2VelMat();
  v2a = _optimizer->getVel2AccMat();
  a2j = _optimizer->getAcc2JerkMat();
  std::cout << "p2v: " << std::endl << p2v << std::endl;
  std::cout << "v2a: " << std::endl << v2a << std::endl;
  std::cout << "a2j: " << std::endl << a2j << std::endl;
  std::cout << "p2a: " << std::endl << v2a * p2v << std::endl;
  EXPECT_EQ(p2v.rows(), 4 * 3);
  EXPECT_EQ(p2v.cols(), 5 * 3);
  EXPECT_EQ(v2a.rows(), 3 * 3);
  EXPECT_EQ(v2a.cols(), 4 * 3);
  EXPECT_EQ(a2j.rows(), 2 * 3);
  EXPECT_EQ(a2j.cols(), 3 * 3);

  Eigen::MatrixXd p2j = a2j * v2a * p2v;
  std::cout << "p2j: " << std::endl << p2j << std::endl;
}

TEST_F(BezierOptTest, TestMinJerkCost) {
  Eigen::MatrixXd Q = _optimizer->getQ();
  std::cout << "cost: " << std::endl << Q << std::endl;
  EXPECT_EQ(Q.rows(), 5 * 3);
  EXPECT_EQ(Q.cols(), 5 * 3);
}

TEST_F(BezierOptTest2, TestOpt) {
  Eigen::MatrixXd A  = _optimizer->getA();
  Eigen::VectorXd b  = _optimizer->getb();
  Eigen::VectorXd lb = _optimizer->getlb();
  Eigen::MatrixXd Ablb;
  Ablb.resize(A.rows(), A.cols() + 2);
  // Ablb << A, lb, b;
  std::cout << "Ablb: " << std::endl << Ablb << std::endl;
  EXPECT_EQ(b.rows(), A.rows());
  EXPECT_EQ(A.cols(), 5 * 3 * 3);
  bool flag = _optimizer->optimize();
  EXPECT_TRUE(flag);
  std::cout << "x_:" << std::endl << _optimizer->getOptCtrlPts() << std::endl;
  Eigen::MatrixXd X = _optimizer->getOptCtrlPtsMat();
  std::cout << "X: " << std::endl << X << std::endl;
  EXPECT_EQ(X.rows(), 15);
  EXPECT_EQ(X.cols(), 3);
}

TEST_F(BezierOptTest2, TestWaypoints) {
  _optimizer->optimize();
  BezierCurve traj;
  _optimizer->getOptBezier(traj);
  double T = traj.getDuration();
  Eigen::Vector3d p_0, v_0, a_0, p_T, v_T, a_T;
  p_0 = traj.getPos(0);
  v_0 = traj.getVel(0);
  a_0 = traj.getAcc(0);
  p_T = traj.getPos(T);
  v_T = traj.getVel(T);
  a_T = traj.getAcc(T);
  EXPECT_LT(abs(p_0(0) - start(0, 0)), 1e-3);
  EXPECT_LT(abs(p_0(1) - start(0, 1)), 1e-3);
  EXPECT_LT(abs(p_0(2) - start(0, 2)), 1e-3);
  EXPECT_LT(abs(v_0(0) - start(1, 0)), 1e-3);
  EXPECT_LT(abs(v_0(1) - start(1, 1)), 1e-3);
  EXPECT_LT(abs(v_0(2) - start(1, 2)), 1e-3);
  EXPECT_LT(abs(a_0(0) - start(2, 0)), 1e-3);
  EXPECT_LT(abs(a_0(1) - start(2, 1)), 1e-3);
  EXPECT_LT(abs(a_0(2) - start(2, 2)), 1e-3);
  EXPECT_LT(abs(p_T(0) - end(0, 0)), 1e-3);
  EXPECT_LT(abs(p_T(1) - end(0, 1)), 1e-3);
  EXPECT_LT(abs(p_T(2) - end(0, 2)), 1e-3);
  EXPECT_LT(abs(v_T(0) - end(1, 0)), 1e-3);
  EXPECT_LT(abs(v_T(1) - end(1, 1)), 1e-3);
  EXPECT_LT(abs(v_T(2) - end(1, 2)), 1e-3);
  EXPECT_LT(abs(a_T(0) - end(2, 0)), 1e-3);
  EXPECT_LT(abs(a_T(1) - end(2, 1)), 1e-3);
  EXPECT_LT(abs(a_T(2) - end(2, 2)), 1e-3);
  // Eigen::MatrixXd V0 = traj[0].getVelCtrlPts();
  // Eigen::MatrixXd A0 = traj[0].getAccCtrlPts();
  // Eigen::MatrixXd V1 = traj[1].getVelCtrlPts();
  // Eigen::MatrixXd A1 = traj[1].getAccCtrlPts();
  // EXPECT_LT((V0.row(3) - V1.row(0)).norm(), 1e-3);
  // EXPECT_LT((A0.row(2) - A1.row(0)).norm(), 1e-3);
  EXPECT_LT((traj.getVel(1.99) - traj.getVel(2.01)).norm(), 1e-3);
  EXPECT_LT((traj.getVel(1.99) - traj.getVel(2.01)).x(), 1e-3);
  EXPECT_LT((traj.getVel(1.99) - traj.getVel(2.01)).y(), 1e-3);
  EXPECT_LT((traj.getVel(1.99) - traj.getVel(2.01)).z(), 1e-3);
  EXPECT_LT((traj.getAcc(1.99) - traj.getAcc(2.01)).norm(), 1e-3);
  EXPECT_LT((traj.getAcc(1.99) - traj.getAcc(2.01)).x(), 1e-3);
  EXPECT_LT((traj.getAcc(1.99) - traj.getAcc(2.01)).y(), 1e-3);
  EXPECT_LT((traj.getAcc(1.99) - traj.getAcc(2.01)).z(), 1e-3);
}

TEST_F(BezierOptTest2, TestContinuity) {
  _optimizer->optimize();
  BezierCurve traj;
  _optimizer->getOptBezier(traj);
  for (double dt = 0; dt <= traj.getDuration(); dt += 0.1) {
    Eigen::Vector3d pos, vel, acc;
    pos = traj.getPos(dt);
    vel = traj.getVel(dt);
    acc = traj.getAcc(dt);
    std::cout << dt << "\t | " << pos.transpose() << " | " << vel.transpose() << " | "
              << acc.transpose() << std::endl;
  }
  EXPECT_EQ(1, 1);
  EXPECT_LE(abs((traj.getVel(1.0) - traj.getVel(0.9)).norm() / 0.1 -
                (traj.getAcc(1.0) + traj.getAcc(0.9)).norm() / 2),
            0.01);
  EXPECT_LE(abs((traj.getVel(2.0) - traj.getVel(1.9)).norm() / 0.1 -
                (traj.getAcc(2.0) + traj.getAcc(1.9)).norm() / 2),
            0.01);
}

}  // namespace traj_opt

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
