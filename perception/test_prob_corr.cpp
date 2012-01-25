#include "optimization_forces.h"
#include <Eigen/Dense>
#include <vector>
using namespace std;

int main() {

  MatrixXf A(2,3);
  MatrixXf B(4,3);

  A << 1,0,0,
    0,1,0;
  B << 1,0,0
    0,1,0,
    0,1,0
    1,1,1;
  VectorXf pVis;
  pVis << 1,0;

  MatrixXf corr = calcCorrProb(A, B, pVis,1, 1);
  cout << corr;

}
