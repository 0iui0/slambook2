//
// Created on 2021/6/22.
//
//
// Created on 2021/6/21.
//

#include <string>
// Eigen 核心部分
#include "Eigen/Core"
// 稠密矩阵的代数运算（逆，特征值等）
#include "Eigen/Dense"
//包含头文件（顺序不能错！！！先包含eigen相关库，再包含opencv库！）
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace Eigen;
using namespace cv;
//#Camera specific definitions.
// rate_hz: 20
// resolution: [752, 480]
// camera_model: pinhole
// intrinsics: [458.654, 457.296, 367.215, 248.375] #fu, fv, cu, cv
// distortion_model: radial-tangential
// distortion_coefficients: [-0.28340811,0.07395907,0.00019359, 1.76187114e-05]
/// 使用了slam 14讲的代码
int main(int argc, char **argv) {
  // 畸变参数
  double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359,
         p2 = 1.76187114e-05;
  // 内参
  double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;
  const Mat K = (Mat_<double>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
  const Mat D = (Mat_<double>(4, 1) << k1, k2, p1, p2);
  // 分辨率
  int rows = 480, cols = 480;

  int blockSize = 60;
  Mat chessBoard(rows, cols, CV_8UC1, Scalar::all(0));
  unsigned char color = 0;

  for (int i = 0; i < rows; i = i + blockSize) {
    color = ~color;
    for (int j = 0; j < cols; j = j + blockSize) {
      Mat ROI = chessBoard(Rect(i, j, blockSize, blockSize));
      ROI.setTo(Scalar::all(color));
      color = ~color;
    }
  }

  Mat imageGT = chessBoard;             // 图像是灰度图，CV_8UC1
  Mat image = Mat(rows, cols, CV_8UC1); // 畸变以后的图
  Mat image_undistort = Mat(rows, cols, CV_8UC1); // 去畸变以后的图

  // 畸变
  for (int v = 0; v < rows; v++) {
    for (int u = 0; u < cols; u++) {
      // 按照公式，计算点(u,v)对应到畸变图像中的坐标(u_distorted, v_distorted)
      double x = (u - cx) / fx, y = (v - cy) / fy;
      double r = sqrt(x * x + y * y);
      double x_distorted = x * (1 + k1 * r * r + k2 * r * r * r * r) +
                           2 * p1 * x * y + p2 * (r * r + 2 * x * x);
      double y_distorted = y * (1 + k1 * r * r + k2 * r * r * r * r) +
                           p1 * (r * r + 2 * y * y) + 2 * p2 * x * y;
      double u_distorted = fx * x_distorted + cx;
      double v_distorted = fy * y_distorted + cy;

      // 赋值 (最近邻插值)
      if (u_distorted >= 0 && v_distorted >= 0 && u_distorted < cols &&
          v_distorted < rows) {
        image.at<uchar>(v_distorted, u_distorted) =
            imageGT.at<uchar>((int)v, (int)u);
      } else {
        image.at<uchar>(v, u) = 0;
      }
    }
  }

  //  计算去畸变后图像的内容
  clock_t time_stt = clock(); // 计时
  undistort(image, image_undistort, K, D, K);
  cout << "time of cv undistort is: "
       << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << endl;
  // cv2eigen
  MatrixXd imageM(rows, cols);
  MatrixXd imageMundistor(rows, cols);
  cv2eigen(image, imageM);
  cv2eigen(image_undistort, imageMundistor);
  cout << "err of cv undistort  is: " << (imageM - imageMundistor).cwiseAbs().sum() << endl;

  imshow("Chess board", imageGT);
  // 画图去畸变后图像
  imshow("distorted", image);
  imshow("undistorted", image_undistort);
  waitKey();
  return 0;
}
