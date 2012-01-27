#include "dist_math.h"
#include "utils_perception.h"
using namespace Eigen;

MatrixXf pairwiseSquareDist(const Eigen::MatrixX3f& x_m3, const Eigen::MatrixX3f& y_n3) {
  // vectors are rows of x and y
  MatrixXf dots_mn = x_m3 * y_n3.transpose();
  VectorXf xnorm_m = x_m3.rowwise().squaredNorm();
  VectorXf ynorm_n = y_n3.rowwise().squaredNorm(); 
  MatrixX3f sumpart_mn = xnorm_m.replicate(1,y_n3.rows()) + ynorm_n.transpose().replicate(x_m3.rows(),1);
  MatrixX3f sqdists_mn = sumpart_mn - 2*dots_mn;
  return sqdists_mn;
}

vector<int> argminAlongRows(const MatrixXf& d_mn) {
  int nRows = d_mn.rows();
  vector<int> out(nRows);
  for (int i=0; i<nRows; i++){
    int j;
    d_mn.row(i).minCoeff(&j);
    out[i]=j;
  }
  return out;
}

class btVector3;
vector<int> getNNInds(vector<btVector3> from, vector<btVector3> to) {
  MatrixXf eigenFrom = toEigenMatrix(from);
  MatrixXf eigenTo = toEigenMatrix(to);
  return argminAlongRows(pairwiseSquareDist(eigenFrom, eigenTo));
}
