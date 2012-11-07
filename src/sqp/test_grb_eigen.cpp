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

  GRBEnv env = GRBEnv();
  GRBModel model = GRBModel(env);
  // Create variables
  GRBVar x = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS, "x");
  GRBVar y = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS, "y");
  GRBVar z = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS, "z");
  // Integrate new variables
  model.update();
  // Set objective
  GRBQuadExpr obj = x*x + x*y + y*y + y*z + z*z;
  model.setObjective(obj);
  // Add constraint: x + 2 y + 3 z >= 4
  model.addConstr(x + 2 * y + 3 * z >= 4, "c0");
  // Add constraint: x + y >= 1
  model.addConstr(x + y >= 1, "c1");
  // Optimize model
  GRBLinExpr xy = x+y;

  model.optimize();
  try {
    cout << "objective: " << model.getObjective() << endl;
    cout << "x value: " << x.get(GRB_DoubleAttr_X) << endl;
    cout << "x+y value: " << xy.getValue();
    cout << "objective: " << model.getObjective() << endl;
    cout << "x value: " << x.get(GRB_DoubleAttr_X) << endl;
    cout << "x+y value: " << xy.getValue();
  }
  catch (GRBException e) {
    cout << e.getMessage() << endl;
  }



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
