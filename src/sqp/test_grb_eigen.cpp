#include <gurobi_c++.h>
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include "utils/utils_vector.h"
using namespace std;
using namespace Eigen;



int main() {
  typedef Matrix<GRBVar, Dynamic, Dynamic> VarMatrix;
  VarMatrix vm(3,3);
  VarMatrix vm1(3,3);
  VarMatrix vm2 = vm + vm1;

}
