#include <gurobi_c++.h>
#include <iostream>
#include <Eigen/Dense>
#include <vector>
using namespace std;
using namespace Eigen;
int main() {
  typedef Matrix<GRBVar, Dynamic, Dynamic> VarMatrix;
  GRBEnv env;
  GRBModel model(env);
  GRBVar v1,v2;
  GRBVar v3 = model.addVar(-1,1,0,GRB_CONTINUOUS);
  cout << "null?" << v1.get(GRB_StringAttr_VarName) << endl;
#if 0
  VarMatrix vm(5,5);
  GRBEnv env;
  GRBModel model(env);
  GRBLinExpr b;
  b += 3;
  cout << b << endl;
  vector<int> a(3,1);
  cout << a.data() << endl;
  try {
    for (int i=0; i < vm.rows(); i++)
      for (int j=0; j < vm.cols(); j++)      
        vm(i,j) = model.addVar(-1,1,1,GRB_CONTINUOUS);
    model.update();
    cout << vm(0,0)-1-vm(0,0)  << endl;
  }
  catch (GRBException e) {
    cout << e.getMessage() << endl;
  }
#endif
}
