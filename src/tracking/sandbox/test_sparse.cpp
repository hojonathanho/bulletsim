#include "tracking/sparse_utils.h"
#include <iostream>
using namespace std;
using namespace Eigen;
int main(int argc, char* argv[]) {
	int rows = 3;
	int cols = 4;
  SparseMatrixf s(rows,cols);
  for (int i=0; i < rows; i++) {
    s.startVec(i);
    for (int j=0; j < cols; j++) {
      s.insertBack(i,j) = i;
    }
  }



  cout << s << endl;
  cout << s.sum() << endl;
  
  VectorXf row(4);
  SparseMatrixf s1(s);
  row << -1,0,1,2;
  rowTimesMatrix(row,s1);
  cout << s1 << endl;
  
  SparseMatrixf s2(s);
  VectorXf col(3);
  col << -1,0,1;
  colTimesMatrix(col, s2);
  cout << s2 << endl;

  MatrixXf q(2,3);
  q << 1.1,0,1.3,
	   0,1.2,0;
  cout << q << endl;
  cout << toSparseMatrix(q,.1) << endl;


      
}
