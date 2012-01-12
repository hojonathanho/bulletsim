#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <btBulletDynamicsCommon.h>
#include "bullet_io.h"
#include <iostream>
using namespace Eigen;
using namespace std;
int main() {
  // vector< Vector3f > m(2);
  // m[0] = Vector3f(0,0,1);
  // m[1] = Vector3f(0,1,1);

  // vector<btVector3> n(2);
  // vector<Vector3f>::iterator begin = m.begin();
  // vector<Vector3f>::iterator end = m.end();

  // vector<btVector3>::iterator begin1 = *reinterpret_cast<vector<btVector3>::iterator*>(&m.begin());
  // vector<btVector3>::iterator end1 = *reinterpret_cast<vector<btVector3>::iterator*>(&m.end());

  // ///vector<btVector3>::iterator end1 = m.end();
  // n.assign(begin1,end1);
  // cout << n[0] << endl;
  //   cout << n[1] << endl;
  //   cout << n[2] << endl;
  //   cout << n[3] << endl;
  //   cout << n[4] << endl;

  MatrixXf x(10,3);
  x.setZero();
  //  Affine3f t = Affine3f::Identity;
  Affine3f aux(Affine3f::Identity());

}
