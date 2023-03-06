/**
 * @file basis_converter.hpp
 * @author Jesus Tordesillas
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-09-05
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef BASIS_CONVERTER_H_
#define BASIS_CONVERTER_H_

#include <Eigen/Eigen>
#include <iostream>
#include <vector>

/**
 * @brief
 * mv: MINVO (minimum volume) basis
 * be: Bernstein basis
 * bs: B-spline basis
 *
 * The basis is defined as follows:
 *
 * p(t) = C * A * [t^3, t^2, t, 1]^T
 *
 * C: control points
 * A: transformation matrix
 */
struct basisConverter {
  Eigen::Matrix<double, 4, 4> A_pos_mv_rest;
  Eigen::Matrix<double, 4, 4> A_pos_be_rest;
  Eigen::Matrix<double, 4, 4> A_pos_bs_seg0, A_pos_bs_seg1, A_pos_bs_rest, A_pos_bs_seg_last2,
      A_pos_bs_seg_last;

  Eigen::Matrix<double, 4, 4> M_pos_bs2mv_seg0, M_pos_bs2mv_seg1, M_pos_bs2mv_rest,
      M_pos_bs2mv_seg_last2, M_pos_bs2mv_seg_last;

  Eigen::Matrix<double, 4, 4> M_pos_bs2be_seg0, M_pos_bs2be_seg1, M_pos_bs2be_rest,
      M_pos_bs2be_seg_last2, M_pos_bs2be_seg_last;

  Eigen::Matrix<double, 3, 3> M_vel_bs2mv_seg0, M_vel_bs2mv_rest, M_vel_bs2mv_seg_last;
  Eigen::Matrix<double, 3, 3> M_vel_bs2be_seg0, M_vel_bs2be_rest, M_vel_bs2be_seg_last;

  basisConverter() {
    // See matlab.
    // This is for t \in [0 1];

    // clang-format off

    /*****  MATRICES A FOR MINVO POSITION  *****/  // (there is only one)
    A_pos_mv_rest << 

     -3.4416308968564117698463178385282,  6.9895481477801393310755884158425, -4.4622887507045296828778191411402,                   0.91437149978080234369315348885721,
      6.6792587327074839365081970754545, -11.845989901556746914934592496138,  5.2523596690684613008670567069203,                                                    0,
     -6.6792587327074839365081970754545,  8.1917862965657040064115790301003, -1.5981560640774179482548333908198,                  0.085628500219197656306846511142794,
      3.4416308968564117698463178385282, -3.3353445427890959784633650997421, 0.80808514571348655231020075007109, -0.0000000000000000084567769453869345852581318467855;

        /*****  MATRICES A FOR Bezier POSITION  *****/  // (there is only one)
    A_pos_be_rest << 

        -1.0,  3.0, -3.0, 1.0,
        3.0, -6.0,  3.0,   0,
        -3.0,  3.0,    0,   0,
        1.0,    0,    0,   0;

    /*****  MATRICES A FOR BSPLINE POSITION  *****/
    A_pos_bs_seg0 <<

        -1.0000,    3.0000,   -3.0000,    1.0000,
        1.7500,   -4.5000,    3.0000,         0,
        -0.9167,    1.5000,         0,         0,
        0.1667,         0,         0,         0;

    A_pos_bs_seg1 <<

        -0.2500,    0.7500,   -0.7500,    0.2500,
        0.5833,   -1.2500,    0.2500,    0.5833,
        -0.5000,    0.5000,    0.5000,    0.1667,
        0.1667,         0,         0,         0;

    A_pos_bs_rest << 

        -0.1667,    0.5000,   -0.5000,    0.1667,
        0.5000,   -1.0000,         0,    0.6667,
        -0.5000,    0.5000,    0.5000,    0.1667,
        0.1667,         0,         0,         0;

    A_pos_bs_seg_last2 <<
        -0.1667,    0.5000,   -0.5000,    0.1667,
        0.5000,   -1.0000,    0.0000,    0.6667,
        -0.5833,    0.5000,    0.5000,    0.1667,
        0.2500,         0,         0,         0;

    A_pos_bs_seg_last <<

        -0.1667,    0.5000,   -0.5000,   0.1667,
        0.9167,   -1.2500,   -0.2500,   0.5833,
        -1.7500,    0.7500,    0.7500,   0.2500,
        1.0000,         0,         0,        0;


        /*****  BSPLINE to MINVO POSITION  *****/

    M_pos_bs2mv_seg0 <<

        1.1023313949144333268037598827505,   0.34205724556666972091534262290224, -0.092730934245582874453361910127569, -0.032032766697130621302846975595457,
    -0.049683556253749178166501110354147,   0.65780347324677179710050722860615,   0.53053863760186903419935333658941,   0.21181027098212013015654520131648,
    -0.047309044211162346038612724896666,  0.015594436894155586093013710069499,    0.5051827557159349613158383363043,   0.63650059656260427054519368539331,
    -0.0053387944495217444854096022766043, -0.015455155707597083292181849856206,  0.057009540927778303009976212933907,   0.18372189915240558222286892942066;

    M_pos_bs2mv_seg1 <<

      0.27558284872860833170093997068761,  0.085514311391667430228835655725561, -0.023182733561395718613340477531892, -0.0080081916742826553257117438988644,
        0.6099042761975865811763242163579,   0.63806904207840509091198555324809,   0.29959938009132258684985572472215,    0.12252106674808682651445224109921,
      0.11985166952332682033244282138185,   0.29187180223752445806795208227413,   0.66657381254229419731416328431806,    0.70176522577378930289881964199594,
    -0.0053387944495217444854096022766043, -0.015455155707597083292181849856206,  0.057009540927778303009976212933907,    0.18372189915240558222286892942066;

    M_pos_bs2mv_rest <<

      0.18372189915240555446729331379174,  0.057009540927778309948870116841135, -0.015455155707597117986651369392348, -0.0053387944495218164764338553140988,
      0.70176522577378919187651717948029,   0.66657381254229419731416328431806,   0.29187180223752384744528853843804,    0.11985166952332582113172065874096,
      0.11985166952332682033244282138185,   0.29187180223752445806795208227413,   0.66657381254229419731416328431806,    0.70176522577378930289881964199594,
    -0.0053387944495217444854096022766043, -0.015455155707597083292181849856206,  0.057009540927778303009976212933907,    0.18372189915240558222286892942066;


    M_pos_bs2mv_seg_last2 <<

        0.18372189915240569324517139193631,  0.057009540927778309948870116841135, -0.015455155707597145742226985021261, -0.0053387944495218164764338553140988,
        0.70176522577378952494342456702725,   0.66657381254229453038107067186502,   0.29187180223752412500104469472717,    0.11985166952332593215402312125661,
         0.1225210667480875342816304396365,   0.29959938009132280889446064975346,   0.63806904207840497988968309073243,    0.60990427619758624810941682881094,
     -0.0080081916742826154270717964323012, -0.023182733561395621468825822830695,  0.085514311391667444106623463540018,    0.27558284872860833170093997068761;

    M_pos_bs2mv_seg_last <<

       0.18372189915240555446729331379174, 0.057009540927778309948870116841135, -0.015455155707597117986651369392348, -0.0053387944495218164764338553140988,
       0.63650059656260415952289122287766,   0.5051827557159349613158383363043,  0.015594436894155294659469745965907,  -0.047309044211162887272337229660479,
       0.21181027098212068526805751389475,  0.53053863760186914522165579910506,   0.65780347324677146403359984105919,  -0.049683556253749622255710960416764,
     -0.032032766697130461708287185729205, -0.09273093424558248587530329132278,   0.34205724556666977642649385416007,     1.1023313949144333268037598827505;


  /*****  BSPLINE to BEZIER POSITION  *****/ //

    M_pos_bs2be_seg0 <<

        1.0000,    0.0000,   -0.0000,         0,
              0,    1.0000,    0.5000,    0.2500,
              0,   -0.0000,    0.5000,    0.5833,
              0,         0,         0,    0.1667;

    M_pos_bs2be_seg1 <<

        0.2500,    0.0000,   -0.0000,         0,
        0.5833,    0.6667,    0.3333,    0.1667,
        0.1667,    0.3333,    0.6667,    0.6667,
              0,         0,         0,    0.1667;

    M_pos_bs2be_rest <<

        0.1667,    0.0000,         0,         0,
        0.6667,    0.6667,    0.3333,    0.1667,
        0.1667,    0.3333,    0.6667,    0.6667,
              0,         0,         0,    0.1667;

    M_pos_bs2be_seg_last2 <<

        0.1667,         0,   -0.0000,         0,
        0.6667,    0.6667,    0.3333,    0.1667,
        0.1667,    0.3333,    0.6667,    0.5833,
              0,         0,         0,    0.2500;

    M_pos_bs2be_seg_last <<

        0.1667,    0.0000,         0,         0,
        0.5833,    0.5000,         0,         0,
        0.2500,    0.5000,    1.0000,         0,
              0,         0,         0,    1.0000;

    /*****  BSPLINE to MINVO VELOCITY  *****/
    M_vel_bs2mv_seg0 <<

    1.077349059083916,  0.1666702138890985, -0.07735049175615138,
 -0.03867488648729411,  0.7499977187062712,   0.5386802643920123,
 -0.03867417280506149, 0.08333206631563977,    0.538670227146185;

    M_vel_bs2mv_rest <<

    0.538674529541958, 0.08333510694454926, -0.03867524587807569,
   0.4999996430546639,  0.8333328256508203,   0.5000050185139366,
 -0.03867417280506149, 0.08333206631563977,    0.538670227146185;

    M_vel_bs2mv_seg_last <<

    0.538674529541958, 0.08333510694454926, -0.03867524587807569,
   0.5386738158597254,  0.7500007593351806, -0.03866520863224832,
 -0.07734834561012298,  0.1666641326312795,     1.07734045429237;

  /*****  BSPLINE to BEZIER VELOCITY  *****/
    M_vel_bs2be_seg0 <<

        1.0000,         0,         0,
              0,    1.0000,    0.5000,
              0,         0,    0.5000;

    M_vel_bs2be_rest <<

        0.5000,         0,         0,
        0.5000,    1.0000,    0.5000,
              0,         0,    0.5000;

    M_vel_bs2be_seg_last <<

        0.5000,         0,         0,
        0.5000,    1.0000,         0,
              0,         0,    1.0000;

    // clang-format on
  }

  /*****  MATRIX A FOR MINVO POSITION  *****/  //
  Eigen::Matrix<double, 4, 4> getArestMinvo() { return A_pos_mv_rest; }
  /*****  MATRIX A FOR Bezier POSITION  *****/  //
  Eigen::Matrix<double, 4, 4> getArestBezier() { return A_pos_be_rest; }

  /*****  MATRIX A FOR BSPLINE POSITION  *****/  //
  Eigen::Matrix<double, 4, 4> getArestBSpline() { return A_pos_bs_rest; }
  /*****  MATRICES A FOR MINVO POSITION  *****/  //
  std::vector<Eigen::Matrix<double, 4, 4>> getAMinvo(int num_pol) {
    std::vector<Eigen::Matrix<double, 4, 4>> A_pos_mv;  // will have as many elements as num_pol
    for (int i = 0; i < num_pol; i++) {
      A_pos_mv.push_back(A_pos_mv_rest);
    }
    return A_pos_mv;
  }

  /*****  MATRICES A FOR Bezier POSITION  *****/  //
  std::vector<Eigen::Matrix<double, 4, 4>> getABezier(int num_pol) {
    std::vector<Eigen::Matrix<double, 4, 4>> A_pos_be;  // will have as many elements as num_pol
    for (int i = 0; i < num_pol; i++) {
      A_pos_be.push_back(A_pos_be_rest);
    }
    return A_pos_be;
  }

  /*****  MATRICES A FOR BSPLINE POSITION  *****/  //
  std::vector<Eigen::Matrix<double, 4, 4>> getABSpline(int num_pol) {
    std::vector<Eigen::Matrix<double, 4, 4>> A_pos_bs;  // will have as many elements as num_pol
    A_pos_bs.push_back(A_pos_bs_seg0);
    A_pos_bs.push_back(A_pos_bs_seg1);
    for (int i = 0; i < (num_pol - 4); i++) {
      A_pos_bs.push_back(A_pos_bs_rest);
    }
    A_pos_bs.push_back(A_pos_bs_seg_last2);
    A_pos_bs.push_back(A_pos_bs_seg_last);
    return A_pos_bs;
  }

  /*****  BSPLINE to MINVO POSITION  *****/  //
  std::vector<Eigen::Matrix<double, 4, 4>> getMinvoPosConverters(int num_pol) {
    std::vector<Eigen::Matrix<double, 4, 4>> M_pos_bs2mv;  // will have as many elements as num_pol
    M_pos_bs2mv.push_back(M_pos_bs2mv_seg0);
    M_pos_bs2mv.push_back(M_pos_bs2mv_seg1);
    for (int i = 0; i < (num_pol - 4); i++) {
      M_pos_bs2mv.push_back(M_pos_bs2mv_rest);
    }
    M_pos_bs2mv.push_back(M_pos_bs2mv_seg_last2);
    M_pos_bs2mv.push_back(M_pos_bs2mv_seg_last);
    return M_pos_bs2mv;
  }

  /*****  BSPLINE to BEZIER POSITION  *****/  //
  std::vector<Eigen::Matrix<double, 4, 4>> getBezierPosConverters(int num_pol) {
    std::vector<Eigen::Matrix<double, 4, 4>> M_pos_bs2be;  // will have as many elements as num_pol
    M_pos_bs2be.push_back(M_pos_bs2be_seg0);
    M_pos_bs2be.push_back(M_pos_bs2be_seg1);
    for (int i = 0; i < (num_pol - 4); i++) {
      M_pos_bs2be.push_back(M_pos_bs2be_rest);
    }
    M_pos_bs2be.push_back(M_pos_bs2be_seg_last2);
    M_pos_bs2be.push_back(M_pos_bs2be_seg_last);
    return M_pos_bs2be;
  }

  /*****  BSPLINE to BSPLINE POSITION  *****/  //
  std::vector<Eigen::Matrix<double, 4, 4>> getBSplinePosConverters(int num_pol) {
    std::vector<Eigen::Matrix<double, 4, 4>> M_pos_bs2bs;  // will have as many elements as num_pol
    for (int i = 0; i < num_pol; i++) {
      M_pos_bs2bs.push_back(Eigen::Matrix<double, 4, 4>::Identity());
    }
    return M_pos_bs2bs;
  }

  /*****  BSPLINE to MINVO Velocity  *****/  //
  std::vector<Eigen::Matrix<double, 3, 3>> getMinvoVelConverters(int num_pol) {
    std::vector<Eigen::Matrix<double, 3, 3>> M_vel_bs2mv;  // will have as many elements as num_pol
    M_vel_bs2mv.push_back(M_vel_bs2mv_seg0);
    for (int i = 0; i < (num_pol - 2 - 1); i++) {
      M_vel_bs2mv.push_back(M_vel_bs2mv_rest);
    }
    M_vel_bs2mv.push_back(M_vel_bs2mv_seg_last);
    return M_vel_bs2mv;
  }

  /*****  BSPLINE to BEZIER Velocity  *****/  //
  std::vector<Eigen::Matrix<double, 3, 3>> getBezierVelConverters(int num_pol) {
    std::vector<Eigen::Matrix<double, 3, 3>> M_vel_bs2be;  // will have as many elements as segments
    M_vel_bs2be.push_back(M_vel_bs2be_seg0);
    for (int i = 0; i < (num_pol - 2 - 1); i++) {
      M_vel_bs2be.push_back(M_vel_bs2be_rest);
    }
    M_vel_bs2be.push_back(M_vel_bs2be_seg_last);
    return M_vel_bs2be;
  }

  /*****  BSPLINE to BSPLINE Velocity  *****/  //
  std::vector<Eigen::Matrix<double, 3, 3>> getBSplineVelConverters(int num_pol) {
    std::vector<Eigen::Matrix<double, 3, 3>> M_vel_bs2bs;  // will have as many elements as num_pol
    for (int i = 0; i < num_pol; i++) {
      M_vel_bs2bs.push_back(Eigen::Matrix<double, 3, 3>::Identity());
    }
    return M_vel_bs2bs;
  }
};

#endif /* BASIS_CONVERTER_H_ */