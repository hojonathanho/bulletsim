#include "correction.h"
#include <iostream>
#include <pcl/point_types.h>
#include "my_assert.h"
using namespace pcl;
using namespace Eigen;
using namespace std;

MatrixXf quadFeats(const MatrixXf& X) {
  int nColsX = X.cols();
  int nRowsX = X.rows();
  ENSURE(nColsX == 3);
  int nColsY = 10;
  MatrixXf Y(X.rows(), nColsY);
  Y.col(0).setOnes();
  Y.block(0,1,nRowsX, 3) = X;
  Y.block(0,4, nRowsX, 3) = X.array().square();
  Y.col(7) = X.col(0).cwiseProduct(X.col(1));
  Y.col(8) = X.col(0).cwiseProduct(X.col(2));
  Y.col(9) = X.col(1).cwiseProduct(X.col(2));
  return Y;
}


ColorCloudPtr correctCloudXYZRGB(ColorCloudPtr in, const Eigen::MatrixXf& coefs) {
  MatrixXf X = in->getMatrixXfMap(3,8,0);
  Eigen::MatrixXf Y = correctPoints(X, coefs);
  ColorCloudPtr out(new ColorCloud());
  out->resize(in->width*in->height);
  out->height=in->height;
  out->width=in->width;
  out->getMatrixXfMap(3,8,0) = Y;
  out->getMatrixXfMap(1,8,4) = in->getMatrixXfMap(1,8,4);
  out->is_dense = in->is_dense;
  return out;
}


Eigen::MatrixXf correctPoints(const Eigen::MatrixXf& X, const Eigen::MatrixXf& coefs) {
  MatrixXf f = quadFeats(X);
  cout << f.rows() << " " << f.cols() << " " << coefs.rows() << " " << coefs.cols() << endl;
  MatrixXf Y = f*coefs;
  cout << coefs << endl;
  return Y;
}
