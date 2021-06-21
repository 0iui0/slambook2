//
// Created on 2021/6/21.
//

#include <string>
// Eigen 核心部分
#include "Eigen/Core"
// 稠密矩阵的代数运算（逆，特征值等）
#include "Eigen/Dense"
//包含头文件（顺序不能错！！！先包含eigen相关库，再包含opencv库！）
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

using namespace std;
using namespace Eigen;
using namespace cv;
string image_file = "./distorted.png";

int main(int argc, char **argv) {

  // 本程序实现调用OpenCV的去畸变
  // 畸变参数
  double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359,
         p2 = 1.76187114e-05;
  // 内参
  double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;
  const Mat K =
      (Mat_<double>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
  const Mat D = (Mat_<double>(4, 1) << k1, k2, p1, p2);

  Mat image = imread(image_file, 0); // 图像是灰度图，CV_8UC1
  int rows = image.rows, cols = image.cols;
  Mat image_undistort = Mat(rows, cols, CV_8UC1); // 去畸变以后的图

  // 计算去畸变后图像的内容
  clock_t time_stt = clock(); // 计时
  undistort(image, image_undistort, K, D, K);
  cout << "time of undistort is "
       << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << endl;
  int row = image.rows;
  int col = image.cols;
  MatrixXd imageM(row, col);
  MatrixXd imageMundistor(row, col);
  cv2eigen(image,imageM);
  cv2eigen(image_undistort,imageMundistor);
  cout << "err is " << (imageM-imageMundistor).cwiseAbs().sum() << endl;
  // 画图去畸变后图像
  imshow("distorted", image);
  imshow("undistorted", image_undistort);
  waitKey();
  return 0;
}
