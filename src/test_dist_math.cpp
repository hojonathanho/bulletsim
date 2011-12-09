#include "dist_math.h"
#include <iostream>
using namespace std;
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

