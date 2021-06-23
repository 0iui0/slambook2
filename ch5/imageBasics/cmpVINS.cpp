
#include <csignal>
#include <cstdio>
#include <execinfo.h>
#include <iostream>
#include <queue>

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace Eigen;
using namespace cv;

//#camera calibration
// model_type: PINHOLE
//    camera_name: camera
//    image_width: 752
// image_height: 480
// distortion_parameters:
// k1: -2.917e-01
// k2: 8.228e-02
// p1: 5.333e-05
// p2: -1.578e-04
// projection_parameters:
// fx: 4.616e+02
// fy: 4.603e+02
// cx: 3.630e+02
// cy: 2.481e+02
//

// 畸变参数
double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359, p2 = 1.76187114e-05;
// 内参
double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;
const Mat K = (Mat_<double>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
const Mat D = (Mat_<double>(4, 1) << k1, k2, p1, p2);
// 分辨率
int rows = 752, cols = 480;

void distortion(const Eigen::Vector2d &p_u, Eigen::Vector2d &d_u) {
  double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

  mx2_u = p_u(0) * p_u(0);
  my2_u = p_u(1) * p_u(1);
  mxy_u = p_u(0) * p_u(1);
  rho2_u = mx2_u + my2_u;
  rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;
  d_u << p_u(0) * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u),
      p_u(1) * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);
}

void liftProjective(const Eigen::Vector2d &p, Eigen::Vector3d &P,
                    double m_inv_K11, double m_inv_K13, double m_inv_K22,
                    double m_inv_K23) {
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
                       map<int, cv::Point2f> cur_un_pts_map, vector<int> ids) {
  cur_un_pts.clear();
  cur_un_pts_map.clear();
  double m_inv_K11, m_inv_K13, m_inv_K22, m_inv_K23;
  m_inv_K11 = 1.0 / fx;
  m_inv_K13 = -cx / fx;
  m_inv_K22 = 1.0 / fy;
  m_inv_K23 = -cy / fy;
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
int main(int argc, char **argv) {
  cv::Point2f point1(0.0, 0.0);
  cv::Point2f point2(0.5, 0.6);
  vector<cv::Point2f> cur_pts{point1, point2};
  vector<cv::Point2f> cur_un_pts{point1, point2};
  map<int, cv::Point2f> cur_un_pts_map;
  vector<int> ids{1, 2, 3};
  undistortedPoints(cur_pts, cur_un_pts, cur_un_pts_map, ids);
  return 0;
}