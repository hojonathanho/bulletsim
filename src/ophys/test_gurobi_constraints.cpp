#include <gurobi_c++.h>
#include <iostream>

using namespace std;

int main(int argc, char *argv[]) {
  GRBEnv env;
  GRBModel model(env);

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

  GRBConstr constr = model.addConstr(x + y >= 1, "c1");

  // Optimize model

  model.optimize();

  cout << x.get(GRB_StringAttr_VarName) << " "
       << x.get(GRB_DoubleAttr_X) << endl;
  cout << y.get(GRB_StringAttr_VarName) << " "
       << y.get(GRB_DoubleAttr_X) << endl;
  cout << z.get(GRB_StringAttr_VarName) << " "
       << z.get(GRB_DoubleAttr_X) << endl;

  cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;

  // Change variable types to integer

  x.set(GRB_CharAttr_VType, GRB_INTEGER);
  y.set(GRB_CharAttr_VType, GRB_INTEGER);
  z.set(GRB_CharAttr_VType, GRB_INTEGER);

  // Optimize model

  model.optimize();

  cout << x.get(GRB_StringAttr_VarName) << " "
       << x.get(GRB_DoubleAttr_X) << endl;
  cout << y.get(GRB_StringAttr_VarName) << " "
       << y.get(GRB_DoubleAttr_X) << endl;
  cout << z.get(GRB_StringAttr_VarName) << " "
       << z.get(GRB_DoubleAttr_X) << endl;

  cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;


  // remove constraint

  model.remove(constr);
  model.optimize();

  cout << x.get(GRB_StringAttr_VarName) << " "
       << x.get(GRB_DoubleAttr_X) << endl;
  cout << y.get(GRB_StringAttr_VarName) << " "
       << y.get(GRB_DoubleAttr_X) << endl;
  cout << z.get(GRB_StringAttr_VarName) << " "
       << z.get(GRB_DoubleAttr_X) << endl;

  cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;



  return 0;

}
