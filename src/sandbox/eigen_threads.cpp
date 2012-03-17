#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
using namespace std;
int main() {
    // cout << Eigen::internal::nbThreads() << endl;
    // Matrix4f m;
    // Affine3f a;
    // a = Translation3f(Vector3f(1,0,0)) * AngleAxisf(3.141592/2, Vector3f::UnitZ());
    // m = a;
    // cout << m << endl;
    
    
    ArrayXXf q = MatrixXf(2,3).array();
    q << 1,2,3,4,5,6;
    Array<bool,Dynamic,Dynamic> c = (q.col(0) <= 1) * (q.col(1) <= 1);
    cout << c << endl;
    cout << c.rows() << " " << c.cols() << endl;
}