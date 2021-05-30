#include <iostream>

using namespace std;

#include "Eigen/Core"
// Eigen/Geometry 模块提供了各种旋转和平移的表示
#include "Eigen/Geometry"

using namespace Eigen;

// 本程序演示了 Eigen 几何模块的使用方法

int main(int argc, char **argv) {
    // 3D 旋转矩阵直接使用 Matrix3d 或 Matrix3f; 用矩阵表示
    Matrix3d rotation_matrix = Matrix3d::Identity();
    // 旋转向量使用 AngleAxisd AngleAxis<double>, 它底层不直接是Matrix，但运算可以当作矩阵（因为重载了运算符）；旋转向量表示 //沿 Z 轴旋转 45 度
    AngleAxisd rotation_vector(M_PI / 4, Vector3d(0, 0, 1));
    cout.precision(3);
    // 旋转向量转换成矩阵 用matrix()
    cout << "rotation matrix =\n" << rotation_vector.matrix() << endl;   //用matrix()转换成矩阵
    // 也可以直接赋值
    rotation_matrix = rotation_vector.toRotationMatrix();
    // 用 AngleAxis 可以进行坐标变换
    Vector3d v(1, 0, 0);
    cout << "(1,0,0) after rotation (by angle axis) = " << (rotation_vector * v).transpose() << endl;
    // 或者用旋转矩阵
    cout << "(1,0,0) after rotation (by matrix) = " << (rotation_matrix * v).transpose() << endl;

    // 欧拉角: 可以将旋转矩阵直接转换成欧拉角; ZYX顺序：yaw-pitch-roll
    // 旋转矩阵转换成欧拉角；ZYX顺序，即yaw-pitch-roll顺序 {0,1,2} 是什么意思？
    Vector3d euler_angles = rotation_matrix.eulerAngles(2, 0, 0);
    cout << "yaw pitch roll = " << euler_angles.transpose() << endl;

    // 欧氏变换矩阵使用 Eigen::Isometry；欧式变换SE(3)
    Isometry3d se3 = Isometry3d::Identity();// 虽然称为3d，实质上是4＊4的矩阵
    se3.rotate(rotation_vector);// 按照rotation_vector进行旋转
    se3.pretranslate(Vector3d(1, 3, 4));// 把平移向量设成(1,3,4)
    cout << "Transform matrix = \n" << se3.matrix() << endl;

    // 用变换矩阵进行坐标变换
    // 相当于R*v+t
    cout << "v tranformed = " << (se3 * v).transpose() << endl;

    // 对于仿射和射影变换，使用 Eigen::Affine3d 和 Eigen::Projective3d 即可，略

    // 四元数
    // 可以直接把AngleAxis赋值给四元数，反之亦然
    // coeffs 顺序是x y z ，w (实部)
    cout << "quaternion from rotation vector = " << Quaterniond(rotation_vector).coeffs().transpose()
         << endl;   // 请注意coeffs的顺序是(x,y,z,w),w为实部，前三者为虚部
    // 也可以把旋转矩阵赋给它
    cout << "quaternion from rotation matrix = " << Quaterniond(rotation_matrix).coeffs().transpose() << endl;
    // 使用四元数旋转一个向量，使用重载的乘法即可
    // v_rotated = q * v; // 注意数学上是qvq^{-1}
    cout << "(1,0,0) after rotation = " << (Quaterniond(rotation_vector) * v).transpose() << endl;
    // 用常规向量乘法表示，则应该如下计算
    cout << "should be equal to "
         << (Quaterniond(rotation_vector) * Quaterniond(0, 1, 0, 0) * Quaterniond(rotation_vector).inverse()).coeffs().transpose()
         << endl;

    return 0;
}
