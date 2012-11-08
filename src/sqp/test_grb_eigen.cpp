#include <gurobi_c++.h>
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include "utils/utils_vector.h"
#include <btBulletDynamicsCommon.h>
using namespace std;
using namespace Eigen;

#define PI SIMD_PI

double toNegPiPi1(double x) {
  while (true) {
    if (x > SIMD_PI)
      x -= 2 * SIMD_PI;
    else if (x < -SIMD_PI)
      x += 2 * SIMD_PI;
    else
      break;
  }
  return x;
}

MatrixXd toNegPiPi(const MatrixXd& in) {
  MatrixXd out(in.rows(), in.cols());
  for (int i = 0; i < in.rows(); ++i) {
    for (int j = 0; j < in.cols(); ++j) {
      out(i,j) = toNegPiPi1(in(i,j));
    }
  }
  return out;
}

int main() {
  MatrixXd x(3, 2);
  x << -5, -2, 1, 4, 7, 20;
  cout << toNegPiPi(x) << endl;
  ;

}
