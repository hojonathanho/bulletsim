#include <LinearMath/btVector3.h>
#include <Eigen/Dense>
#include <iostream>
#include <vector>
using namespace std;

#define EIGEN_DEFAULT_TO_ROW_MAJOR

int main() {
  btVector3 v0(1,2,3);
  btVector3 v1(4,5,6);
  vector<btVector3> q;
  q.push_back(v0);
  q.push_back(v1);
  

  float* dataptr = reinterpret_cast<float*>(q.data());
  for (int i=0; i < 8; i++)
    cout << (dataptr[i]) << endl;  
}