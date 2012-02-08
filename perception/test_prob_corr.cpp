#include <Eigen/Dense>
#include <vector>
#include <iostream>
using namespace std;
using namespace Eigen;


MatrixXf pairwiseSquareDist(const MatrixXf& x_m3, const MatrixXf& y_n3) {
  // vectors are rows of x and y
  MatrixXf dots_mn = x_m3 * y_n3.transpose();
  VectorXf xnorm_m = x_m3.rowwise().squaredNorm();
  VectorXf ynorm_n = y_n3.rowwise().squaredNorm();
  MatrixXf sqdists_mn = ((-2*dots_mn).colwise() + xnorm_m).transpose().colwise()+ynorm_n;
  return sqdists_mn.transpose();
}

// todo: normalization factor in likelihood
MatrixXf calcCorrProb(const MatrixXf& estPts, const MatrixXf& obsPts, const VectorXf& pVis, float stdev, float pBandOutlier) {
  MatrixXf sqdists = pairwiseSquareDist(estPts, obsPts);
  MatrixXf pBgivenZ_unnormed = (-sqdists/(2*stdev)).array().exp();
  MatrixXf pBandZ_unnormed = pVis.asDiagonal()*pBgivenZ_unnormed;
  VectorXf pB_unnormed = pBandZ_unnormed.colwise().sum();
  VectorXf pBorOutlier_unnormed = (pB_unnormed.array() + pBandOutlier).inverse();
  MatrixXf pZgivenB = pBandZ_unnormed * pBorOutlier_unnormed.asDiagonal();
  return pZgivenB;
}

int main() {

  MatrixXf A(2,3);
  MatrixXf B(4,3);

  A << 1,0,0,
    0,1,0;
  B << 1,0,0,
    0,1,0,
    0,1,0,
    1,1,1;
  VectorXf pVis(2);
  pVis << 1,0;

  MatrixXf corr;

  corr = calcCorrProb(A, B, pVis,1, 1);
  cout << corr << endl;

  corr = calcCorrProb(A, B, pVis,1, 0);
  cout << corr << endl;


}
