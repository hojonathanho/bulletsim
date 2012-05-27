#include "sparse_utils.h"
#include <iostream>
using namespace std;
using namespace Eigen;
int main(int argc, char* argv[]) {
  SparseMatrixf s(3,3);
  for (int i=0; i < 3; i++) {
    s.startVec(i);
    for (int j=0; j < 3; j++) {
      s.insertBack(i,j) = i+j;
    }
  }



  cout << s << endl;
  cout << s.sum() << endl;
  
  VectorXf vec(3);
  vec << -1,0,1;
  
  rowTimesMatrix(vec,s);
  cout << s << endl;
  
  colTimesMatrix(vec, s);
  cout << s << endl;
      
}