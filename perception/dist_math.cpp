#include "dist_math.h"
using namespace Eigen;


MatrixXf pairwiseSquareDist(const MatrixXf& x_m3, const MatrixXf& y_n3) {
  // vectors are rows of x and y
  MatrixXf dots_mn = x_m3 * y_n3.transpose();
  VectorXf xnorm_m = x_m3.rowwise().squaredNorm();
  VectorXf ynorm_n = y_n3.rowwise().squaredNorm();
  MatrixXf sqdists_mn = ((-2*dots_mn).colwise() + xnorm_m).transpose().colwise()+ynorm_n;
  return sqdists_mn.transpose();
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
/*
int main() {
  MatrixXf X(4,3);
  MatrixXf Y(2,3);
  X << 1,0,0,
    0,1,0,
    0,0,1,
    1,1,1;
  Y << 1,0,0,
    0,1,0;
  MatrixXf psd = pairwiseSquareDist(X,Y);
  cout << psd << endl;
  VectorXi amar = argminAlongRows(psd);
  cout << amar << endl;
}
*/
