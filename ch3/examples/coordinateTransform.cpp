#include <iostream>
#include <vector>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv) {
    //小萝卜头1号，2号，的旋转用四元数表示
    Quaterniond q1(0.35, 0.2, 0.3, 0.1), q2(-0.5, 0.4, -0.1, 0.2);
    q1.normalize();
    q2.normalize();
    //小萝卜头1号，2号，的平移用向量表示
    Vector3d t1(0.3, 0.1, 0.1), t2(-0.1, 0.5, 0.3);
    //1号看到某一个点在自身坐标系下的坐标
    Vector3d p1(0.5, 0, 0.2);

    Isometry3d T1w(q1), T2w(q2);
    T1w.pretranslate(t1);
    T2w.pretranslate(t2);

    //T1w.inverse()*p1先转换到世界坐标，然后左成2号的变换矩阵，就是在2号坐标系下的坐标。
    Vector3d p2 = T2w * T1w.inverse() * p1;
    cout << endl << p2.transpose() << endl;
    return 0;
}