/**
 * @file visualizer.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-08-08
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <traj_utils/geo_utils.hpp>
#include <traj_utils/quickhull.hpp>
#include <traj_utils/visualizer.hpp>

namespace visualizer {

/**
 * @brief publish trajectory and color the speed
 *
 * @param start_pos when planning starts
 * @param traj       trajectory to be visualized
 * @param max_vel    maximum velocity to visualize as red line
 */
void Visualizer::visualizePolyTraj(const Eigen::Vector3d&        start_pos,
                                   const polynomial::Trajectory& traj,
                                   double                        max_vel) {
  visualization_msgs::Marker traj_marker;
  traj_marker.header.frame_id    = _frame_id;
  traj_marker.header.stamp       = ros::Time::now();
  traj_marker.type               = visualization_msgs::Marker::LINE_LIST;
  traj_marker.pose.orientation.w = 1.00;
  traj_marker.action             = visualization_msgs::Marker::ADD;
  traj_marker.id                 = 0;
  traj_marker.ns                 = "trajectory";
  traj_marker.color.r            = 0.00;
  traj_marker.color.g            = 0.50;
  traj_marker.color.b            = 1.00;
  traj_marker.color.a            = 1.00;
  traj_marker.scale.x            = 0.10;

  double          T     = 0.05;
  Eigen::Vector3d lastX = traj.getPos(0.0) + start_pos;
  for (double t = T; t < traj.getDuration(); t += T) {
    std_msgs::ColorRGBA c;
    Eigen::Vector3d     jets = jetColorMap(traj.getVel(t).norm() / max_vel);
    c.r                      = jets[0];
    c.g                      = jets[1];
    c.b                      = jets[2];

    geometry_msgs::Point point;
    Eigen::Vector3d      X = traj.getPos(t) + start_pos;
    point.x                = lastX(0);
    point.y                = lastX(1);
    point.z                = lastX(2);
    traj_marker.points.push_back(point);
    traj_marker.colors.push_back(c);
    point.x = X(0);
    point.y = X(1);
    point.z = X(2);
    traj_marker.points.push_back(point);
    traj_marker.colors.push_back(c);
    lastX = X;
  }
  _colorful_traj_pub.publish(traj_marker);
}

/**
 * @brief visualize corridors
 * Corridor is given by normal vector and offset
 * @param corridors
 * @param pose
 * @param rviz_map_center_locked true if map center is locked
 * @param clear_corridors
 */
void Visualizer::visualizeCorridors(const traj_utils::Corridors& corridors,
                                    const Eigen::Vector3d&       map_pose) {
  displayCorridors(corridors, map_pose, _corridor_pub, _frame_id);
}

void Visualizer::visualizePolytope(const std::vector<Eigen::MatrixX4d>& hPolys) {
  // Due to the fact that H-representation cannot be directly visualized
  // We first conduct vertex enumeration of them, then apply quickhull
  // to obtain triangle meshs of polyhedra
  Eigen::Matrix3Xd mesh(3, 0), curTris(3, 0), oldTris(3, 0);
  for (size_t id = 0; id < hPolys.size(); id++) {
    oldTris = mesh;
    Eigen::Matrix<double, 3, -1, Eigen::ColMajor> vPoly;
    geo_utils::enumerateVs(hPolys[id], vPoly);

    quickhull::QuickHull<double> tinyQH;
    const auto  polyHull  = tinyQH.getConvexHull(vPoly.data(), vPoly.cols(), false, true);
    const auto& idxBuffer = polyHull.getIndexBuffer();
    int         hNum      = idxBuffer.size() / 3;

    curTris.resize(3, hNum * 3);
    for (int i = 0; i < hNum * 3; i++) {
      curTris.col(i) = vPoly.col(idxBuffer[i]);
    }
    mesh.resize(3, oldTris.cols() + curTris.cols());
    mesh.leftCols(oldTris.cols())  = oldTris;
    mesh.rightCols(curTris.cols()) = curTris;
  }

  // RVIZ support tris for visualization
  visualization_msgs::Marker meshMarker, edgeMarker;

  meshMarker.id                 = 0;
  meshMarker.header.stamp       = ros::Time::now();
  meshMarker.header.frame_id    = "world";
  meshMarker.pose.orientation.w = 1.00;
  meshMarker.action             = visualization_msgs::Marker::ADD;
  meshMarker.type               = visualization_msgs::Marker::TRIANGLE_LIST;
  meshMarker.ns                 = "mesh";
  meshMarker.color.r            = 0.00;
  meshMarker.color.g            = 0.00;
  meshMarker.color.b            = 1.00;
  meshMarker.color.a            = 0.10;
  meshMarker.scale.x            = 1.0;
  meshMarker.scale.y            = 1.0;
  meshMarker.scale.z            = 1.0;

  edgeMarker         = meshMarker;
  edgeMarker.type    = visualization_msgs::Marker::LINE_LIST;
  edgeMarker.ns      = "edge";
  edgeMarker.color.r = 0.00;
  edgeMarker.color.g = 1.00;
  edgeMarker.color.b = 1.00;
  edgeMarker.color.a = 1.00;
  edgeMarker.scale.x = 0.02;

  geometry_msgs::Point point;

  int ptnum = mesh.cols();

  for (int i = 0; i < ptnum; i++) {
    point.x = mesh(0, i);
    point.y = mesh(1, i);
    point.z = mesh(2, i);
    meshMarker.points.push_back(point);
  }

  for (int i = 0; i < ptnum / 3; i++) {
    for (int j = 0; j < 3; j++) {
      point.x = mesh(0, 3 * i + j);
      point.y = mesh(1, 3 * i + j);
      point.z = mesh(2, 3 * i + j);
      edgeMarker.points.push_back(point);
      point.x = mesh(0, 3 * i + (j + 1) % 3);
      point.y = mesh(1, 3 * i + (j + 1) % 3);
      point.z = mesh(2, 3 * i + (j + 1) % 3);
      edgeMarker.points.push_back(point);
    }
  }

  _mesh_pub.publish(meshMarker);
  _edge_pub.publish(edgeMarker);

  return;
}

/** NOTE: There are still some bugs in this function
 * @brief if corridor is given in H-representation
 * @param corridors H-representation of corridors
 * @param map_pose
 * @param crd_pub
 * @param frame_id
 */
void displayCorridors(const std::vector<Eigen::MatrixX4d>& corridors,
                      const Eigen::Vector3d&               map_pose,
                      const ros::Publisher&                crd_pub,
                      const std::string                    frame_id = "world") {
  vec_E<Polyhedron3D> polyhedra;
  polyhedra.reserve(corridors.size());
  for (const auto& crd : corridors) {
    Polyhedron3D poly;
    for (int i = 0; i < crd.rows(); i++) {
      Eigen::Vector3d pos;

      double a  = crd(i, 0);
      double b  = crd(i, 1);
      double c  = crd(i, 2);
      double d  = crd(i, 3);
      double z0 = (c == 0) ? 0.0 : -(a + b + c) / c;
      double x0 = (c == 0) ? -(b + d) / a : 1.0;
      pos << x0, 1.0, z0;
      poly.add(Hyperplane3D(pos, crd.row(i).head<3>()));
      // std::cout << "normal: " << gcrd.row(i).head<3>() << " pos: " << pos.transpose() <<
      // std::endl;
    }
    polyhedra.push_back(poly);
  }
  decomp_ros_msgs::PolyhedronArray msg = DecompROS::polyhedron_array_to_ros(polyhedra);
  msg.header.frame_id                  = frame_id;
  msg.header.stamp                     = ros::Time::now();
  crd_pub.publish(msg);
}

/**
 * @brief if corridor is given in H-representation
 * c[0]x + c[1]y + c[2]z + c[3] <= 0
 * @param corridors
 * @param map_pose
 */
void Visualizer::visualizeCorridors(const std::vector<Eigen::MatrixX4d>& corridors,
                                    const Eigen::Vector3d&               map_pose) {
  displayCorridors(corridors, map_pose, _corridor_pub, _frame_id);
}

void Visualizer::visualizePath(const std::vector<Eigen::Vector3d>& route) {
  visualization_msgs::Marker routeMarker;
  routeMarker.id                 = 0;
  routeMarker.type               = visualization_msgs::Marker::LINE_LIST;
  routeMarker.header.stamp       = ros::Time::now();
  routeMarker.header.frame_id    = _frame_id;
  routeMarker.pose.orientation.w = 1.00;
  routeMarker.action             = visualization_msgs::Marker::ADD;
  routeMarker.ns                 = "path";
  routeMarker.color.r            = 1.00;
  routeMarker.color.g            = 0.00;
  routeMarker.color.b            = 0.00;
  routeMarker.color.a            = 1.00;
  routeMarker.scale.x            = 0.05;
  if (route.size() > 0) {
    bool            first = true;
    Eigen::Vector3d last;
    for (auto it : route) {
      if (first) {
        first = false;
        last  = it;
        continue;
      }
      geometry_msgs::Point point;

      point.x = last(0);
      point.y = last(1);
      point.z = last(2);
      routeMarker.points.push_back(point);
      point.x = it(0);
      point.y = it(1);
      point.z = it(2);
      routeMarker.points.push_back(point);
      last = it;
    }
    _astar_path_pub.publish(routeMarker);
  }
}

void Visualizer::visualizeAstarPathXYT(const std::vector<Eigen::Vector3d>& points, double dt) {
  int                          idx = 0;
  std::vector<Eigen::Vector3d> points_xyt;
  points_xyt.reserve(points.size());
  for (auto& p : points) {
    double t = idx * dt;
    points_xyt.push_back(Eigen::Vector3d(p(0), p(1), t));
    idx++;
  }

  visualization_msgs::Marker astar_mkr;
  astar_mkr.header.frame_id = _frame_id;
  astar_mkr.header.stamp    = ros::Time::now();

  astar_mkr.type    = visualization_msgs::Marker::LINE_LIST;
  astar_mkr.action  = visualization_msgs::Marker::ADD;
  astar_mkr.ns      = "astar_path";
  astar_mkr.id      = 1;
  astar_mkr.scale.x = 0.1;
  astar_mkr.scale.y = 0.1;
  astar_mkr.scale.z = 0.1;
  astar_mkr.color.a = 0.8;
  astar_mkr.color.r = 0.942;
  astar_mkr.color.g = 0.215;
  astar_mkr.color.b = 0.322;

  astar_mkr.pose.orientation.w = 1.0;

  Eigen::Vector3d last;
  bool            first = true;
  for (const auto& point : points_xyt) {
    if (first) {
      first = false;
      last  = point;
      continue;
    }
    geometry_msgs::Point p;
    p.x = last.x();
    p.y = last.y();
    p.z = last.z();
    astar_mkr.points.push_back(p);
    p.x = point.x();
    p.y = point.y();
    p.z = point.z();
    astar_mkr.points.push_back(p);
    last = point;
  }
  _astar_t_path_pub.publish(astar_mkr);
}

void Visualizer::visualizeAstarPath(const std::vector<Eigen::Vector3d>& points) {
  visualization_msgs::Marker pt_marker;
  pt_marker.header.frame_id = _frame_id;
  pt_marker.header.stamp    = ros::Time::now();

  pt_marker.type    = visualization_msgs::Marker::POINTS;
  pt_marker.ns      = "astar_path";
  pt_marker.id      = 0;
  pt_marker.action  = visualization_msgs::Marker::ADD;
  pt_marker.scale.x = 0.1;
  pt_marker.scale.y = 0.1;
  pt_marker.scale.z = 0.1;
  pt_marker.color.a = 1.0;
  pt_marker.color.r = 0.8;
  pt_marker.color.g = 0.3;
  pt_marker.color.b = 0.4;

  visualization_msgs::Marker astar_mkr;
  astar_mkr.header.frame_id = _frame_id;
  astar_mkr.header.stamp    = ros::Time::now();

  astar_mkr.type    = visualization_msgs::Marker::LINE_LIST;
  astar_mkr.action  = visualization_msgs::Marker::ADD;
  astar_mkr.ns      = "astar_path";
  astar_mkr.id      = 1;
  astar_mkr.scale.x = 0.1;
  astar_mkr.scale.y = 0.1;
  astar_mkr.scale.z = 0.1;
  astar_mkr.color.a = 0.7;
  astar_mkr.color.r = 0.942;
  astar_mkr.color.g = 0.215;
  astar_mkr.color.b = 0.322;

  astar_mkr.pose.orientation.w = 1.0;

  Eigen::Vector3d last;
  bool            first = true;
  for (const auto& point : points) {
    if (first) {
      first = false;
      last  = point;
      continue;
    }
    geometry_msgs::Point p;
    p.x = last.x();
    p.y = last.y();
    p.z = last.z();
    astar_mkr.points.push_back(p);
    p.x = point.x();
    p.y = point.y();
    p.z = point.z();
    astar_mkr.points.push_back(p);
    pt_marker.points.push_back(p);
    last = point;
  }

  _astar_path_pub.publish(astar_mkr);
  // _astar_path_pub.publish(pt_marker);
}

/**
 * @brief visualize start and goal points
 *
 * @param center
 * @param radius
 * @param sg
 */
void Visualizer::visualizeStartGoal(const Eigen::Vector3d& center, int sg) {
  visualization_msgs::Marker sphereMarkers, sphereDeleter;
  float                      radius = 0.1;

  sphereMarkers.id                 = sg;
  sphereMarkers.type               = visualization_msgs::Marker::SPHERE;
  sphereMarkers.header.stamp       = ros::Time::now();
  sphereMarkers.header.frame_id    = _frame_id;
  sphereMarkers.pose.orientation.w = 1.00;
  sphereMarkers.action             = visualization_msgs::Marker::ADD;
  sphereMarkers.ns                 = "StartGoal";
  sphereMarkers.color.r            = 1.00;
  sphereMarkers.color.g            = 0.00;
  sphereMarkers.color.b            = 0.00;
  sphereMarkers.color.a            = 1.00;
  sphereMarkers.scale.x            = radius * 2.0;
  sphereMarkers.scale.y            = radius * 2.0;
  sphereMarkers.scale.z            = radius * 2.0;

  sphereMarkers.pose.position.x = center(0);
  sphereMarkers.pose.position.y = center(1);
  sphereMarkers.pose.position.z = center(2);

  // geometry_msgs::Point point;
  // point.x = center(0);
  // point.y = center(1);
  // point.z = center(2);
  // sphereMarkers.points.push_back(point);

  // if (sg == 0) {
  //   _start_goal_pub.publish(sphereDeleter);
  //   ros::Duration(1.0e-9).sleep();
  //   sphereMarkers.header.stamp = ros::Time::now();
  // }
  _start_goal_pub.publish(sphereMarkers);
}

void Visualizer::visualizeBezierCurve(const Eigen::Vector3d&   start_pos,
                                      const Bernstein::Bezier& traj,
                                      double                   max_vel) {
  visualization_msgs::Marker traj_marker;
  traj_marker.header.frame_id    = _frame_id;
  traj_marker.header.stamp       = ros::Time::now();
  traj_marker.type               = visualization_msgs::Marker::LINE_LIST;
  traj_marker.pose.orientation.w = 1.00;
  traj_marker.action             = visualization_msgs::Marker::ADD;
  traj_marker.id                 = 0;
  traj_marker.ns                 = "trajectory";
  traj_marker.color.r            = 0.00;
  traj_marker.color.g            = 0.50;
  traj_marker.color.b            = 1.00;
  traj_marker.color.a            = 1.00;
  traj_marker.scale.x            = 0.05;

  double          T     = 0.05;
  Eigen::Vector3d lastX = traj.getPos(0.0) + start_pos;
  for (double t = T; t < traj.getDuration(); t += T) {
    std_msgs::ColorRGBA c;
    Eigen::Vector3d     jets = jetColorMap(traj.getVel(t).norm() / max_vel);
    c.r                      = jets[0];
    c.g                      = jets[1];
    c.b                      = jets[2];

    geometry_msgs::Point point;
    Eigen::Vector3d      X = traj.getPos(t) + start_pos;
    point.x                = lastX(0);
    point.y                = lastX(1);
    point.z                = lastX(2);
    traj_marker.points.push_back(point);
    traj_marker.colors.push_back(c);
    point.x = X(0);
    point.y = X(1);
    point.z = X(2);
    traj_marker.points.push_back(point);
    traj_marker.colors.push_back(c);
    lastX = X;
  }
  _colorful_traj_pub.publish(traj_marker);
}

/* visualize control pointss */
void Visualizer::visualizeControlPoints(const Eigen::MatrixX3d& cpts) {
  visualization_msgs::Marker cpts_marker;
  cpts_marker.header.frame_id    = _frame_id;
  cpts_marker.header.stamp       = ros::Time::now();
  cpts_marker.type               = visualization_msgs::Marker::SPHERE_LIST;
  cpts_marker.pose.orientation.w = 1.00;
  cpts_marker.action             = visualization_msgs::Marker::ADD;
  cpts_marker.id                 = 1;
  cpts_marker.ns                 = "control_points";
  cpts_marker.color.r            = 0.00;
  cpts_marker.color.g            = 1.00;
  cpts_marker.color.b            = 0.00;
  cpts_marker.color.a            = 1.00;
  cpts_marker.scale.x            = 0.10;
  cpts_marker.scale.y            = 0.10;
  cpts_marker.scale.z            = 0.10;

  for (int i = 0; i < cpts.rows(); i++) {
    geometry_msgs::Point point;
    point.x = cpts(i, 0);
    point.y = cpts(i, 1);
    point.z = cpts(i, 2);
    cpts_marker.points.push_back(point);
  }
  _ctrl_pts_pub.publish(cpts_marker);
}

void Visualizer::displayOptimizationInfo(const double& comp_time,
                                         const double& max_velocity,
                                         const double& max_acceleration,
                                         const double& duration) {
  visualization_msgs::Marker textMarker;
  textMarker.header.frame_id = _frame_id;
  textMarker.header.stamp    = ros::Time::now();
  textMarker.ns              = "text";
  textMarker.id              = 1;
  textMarker.type            = visualization_msgs::Marker::TEXT_VIEW_FACING;
  textMarker.action          = visualization_msgs::Marker::ADD;

  textMarker.pose.position.x    = -9;
  textMarker.pose.position.y    = 0.0;
  textMarker.pose.position.z    = 6.0;
  textMarker.pose.orientation.x = 0.0;
  textMarker.pose.orientation.y = 0.0;
  textMarker.pose.orientation.z = 0.0;
  textMarker.pose.orientation.w = 1.0;
  textMarker.scale.x            = 1.0;
  textMarker.scale.y            = 1.0;
  textMarker.scale.z            = 1.0;
  textMarker.color.r            = 1.0;
  textMarker.color.g            = 0.0;
  textMarker.color.b            = 0.0;
  textMarker.color.a            = 1.0;
  textMarker.text               = "Comp: ";
  textMarker.text += std::to_string(static_cast<int>(comp_time));
  textMarker.text += ".";
  textMarker.text += std::to_string(static_cast<int>(comp_time * 10) % 10);
  textMarker.text += "ms\n";
  textMarker.text += "Max speed: ";
  textMarker.text += std::to_string(static_cast<int>(max_velocity));
  textMarker.text += ".";
  textMarker.text += std::to_string(static_cast<int>(max_velocity * 100) % 100);
  textMarker.text += "m/s\n";
  textMarker.text += "Max acceleration: ";
  textMarker.text += std::to_string(static_cast<int>(max_acceleration));
  textMarker.text += ".";
  textMarker.text += std::to_string(static_cast<int>(max_acceleration * 100) % 100);
  textMarker.text += "m/s\n";
  textMarker.text += "Total time: ";
  textMarker.text += std::to_string(static_cast<int>(duration));
  textMarker.text += ".";
  textMarker.text += std::to_string(static_cast<int>(duration * 100) % 100);
  textMarker.text += "s\n";
  _text_pub.publish(textMarker);
}

Eigen::Vector3d rotateVectorByQuaternion(const Eigen::Vector3d& v, const Eigen::Quaterniond& q) {
  Eigen::Quaterniond qvec, rst;
  qvec.w()   = 0;
  qvec.vec() = v;
  rst        = q * qvec * q.inverse();
  return rst.vec();
}

void Visualizer::visualizeFOV(const Eigen::Vector3d&    pose,
                              const Eigen::Quaterniond& attitude,
                              double                    angle_h_deg,
                              double                    angle_v_deg,
                              double                    length = 5) {
  double angle_h = angle_h_deg / 180 * M_PI;
  double angle_v = angle_v_deg / 180 * M_PI;

  geometry_msgs::Point p_cam;
  p_cam.x = pose.x();
  p_cam.y = pose.y();
  p_cam.z = pose.z();

  geometry_msgs::Point p1, p2, p3, p4;

  Eigen::Vector3d p1v(length, length * tan(angle_h / 2), length * tan(angle_v / 2));
  p1v  = rotateVectorByQuaternion(p1v, attitude);
  p1.x = p_cam.x + p1v.x();
  p1.y = p_cam.y + p1v.y();
  p1.z = p_cam.z + p1v.z();

  Eigen::Vector3d p2v(length, -length * tan(angle_h / 2), length * tan(angle_v / 2));
  p2v  = rotateVectorByQuaternion(p2v, attitude);
  p2.x = p_cam.x + p2v.x();
  p2.y = p_cam.y + p2v.y();
  p2.z = p_cam.z + p2v.z();

  Eigen::Vector3d p3v(length, length * tan(angle_h / 2), -length * tan(angle_v / 2));
  p3v  = rotateVectorByQuaternion(p3v, attitude);
  p3.x = p_cam.x + p3v.x();
  p3.y = p_cam.y + p3v.y();
  p3.z = p_cam.z + p3v.z();

  Eigen::Vector3d p4v(length, -length * tan(angle_h / 2), -length * tan(angle_v / 2));
  p4v  = rotateVectorByQuaternion(p4v, attitude);
  p4.x = p_cam.x + p4v.x();
  p4.y = p_cam.y + p4v.y();
  p4.z = p_cam.z + p4v.z();

  visualization_msgs::Marker fov;
  fov.header.frame_id = "world";
  fov.header.stamp    = ros::Time::now();
  fov.action          = visualization_msgs::Marker::ADD;
  fov.ns              = "lines_and_points";
  fov.id              = 999;
  fov.type            = 4;

  fov.scale.x  = 0.1;
  fov.scale.y  = 0.1;
  fov.scale.z  = 0.1;
  fov.color.r  = 0.8;
  fov.color.g  = 0.5;
  fov.color.b  = 0.5;
  fov.color.a  = 0.8;
  fov.lifetime = ros::Duration(0);

  fov.points.push_back(p1);
  fov.points.push_back(p2);
  fov.points.push_back(p_cam);
  fov.points.push_back(p4);
  fov.points.push_back(p3);
  fov.points.push_back(p_cam);
  fov.points.push_back(p1);
  fov.points.push_back(p3);
  fov.points.push_back(p4);
  fov.points.push_back(p2);
  _fov_pub.publish(fov);
}

}  // namespace visualizer
