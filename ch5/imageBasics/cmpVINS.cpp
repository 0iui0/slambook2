
#include <csignal>
#include <cstdio>
#include <execinfo.h>
#include <iostream>
#include <queue>

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

distortion(const Eigen::Vector2d &p_u, Eigen::Vector2d &d_u) const {
  double k1 = 0;
  double k2 = 0;
  double p1 = 0;
  double p2 = 0;

  double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

  mx2_u = p_u(0) * p_u(0);
  my2_u = p_u(1) * p_u(1);
  mxy_u = p_u(0) * p_u(1);
  rho2_u = mx2_u + my2_u;
  rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;
  d_u << p_u(0) * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u),
      p_u(1) * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);
}

liftProjective(const Eigen::Vector2d &p, Eigen::Vector3d &P, double m_inv_K11,
               m_inv_K13, m_inv_K22, m_inv_K23) const {
  double mx_d, my_d, mx2_d, mxy_d, my2_d, mx_u, my_u;
  double rho2_d, rho4_d, radDist_d, Dx_d, Dy_d, inv_denom_d;
  // double lambda;
  // Lift points to normalised plane
  mx_d = m_inv_K11 * p(0) + m_inv_K13;
  my_d = m_inv_K22 * p(1) + m_inv_K23;

  // Recursive distortion model
  int n = 8;
  Eigen::Vector2d d_u;
  distortion(Eigen::Vector2d(mx_d, my_d), d_u);
  // Approximate value
  mx_u = mx_d - d_u(0);
  my_u = my_d - d_u(1);

  for (int i = 1; i < n; ++i) {
    distortion(Eigen::Vector2d(mx_u, my_u), d_u);
    mx_u = mx_d - d_u(0);
    my_u = my_d - d_u(1);
  }
}

void undistortedPoints(vector<cv::Point2f> cur_pts,
                       vector<cv::Point2f> cur_un_pts,
                       map<int, cv::Point2f> cur_un_pts_map) {
  cur_un_pts.clear();
  cur_un_pts_map.clear();
  double m_inv_K11, m_inv_K13, m_inv_K22, m_inv_K23;
  //   cv::undistortPoints(cur_pts, cur_un_pts, K, cv::Mat());
  for (unsigned int i = 0; i < cur_pts.size(); i++) {
    Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
    Eigen::Vector3d b;
    liftProjective(a, b, m_inv_K11, m_inv_K13, m_inv_K22, m_inv_K23);
    cur_un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
    cur_un_pts_map.insert(
        make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
    printf("cur pts id %d %f %f", ids[i], cur_un_pts[i].x, cur_un_pts[i].y);
  }
}
