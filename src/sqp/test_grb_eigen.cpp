#include <gurobi_c++.h>
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include "utils/utils_vector.h"
using namespace std;
using namespace Eigen;

void merged(const vector<double>& v0, const vector<double>& v1, vector<double>& out, vector<int>& inds0, vector<int>& inds1) {
  int nNew = v0.size() + v1.size();
  out.resize(nNew);
  inds0.resize(v0.size());
  inds1.resize(v1.size());

  int i0(0), i1(0);
  bool done0(false), done1(false);
  for (int i=0; i < nNew; ++i) {
    if (i0 == v0.size()) done0=true;
    if (i1 == v1.size()) done1=true;
    assert(!done0 || !done1);
    if (!done0 && (done1 || v0[i0] < v1[i1])) {
      out[i] = v0[i0];
      inds0[i0] = i;
      ++i0;
    }
    else {
      out[i] = v1[i1];
      inds1[i1] = i;
      ++i1;
    }
  }
}


int main() {
#if 0
  vector<double> V0,V1,Vout;
  vector<int> I0, I1;
  V0.push_back(0.05);
  V0.push_back(1);
  V0.push_back(2);
  V0.push_back(3);
  V1.push_back(.5);
  V1.push_back(2.5);
  V1.push_back(3.5);
  merged(V0, V1, Vout, I0, I1);
  cout << Vout << endl;
  cout << I0 << endl;
  cout << I1 << endl;
  V0.clear();
  merged(V0, V1, Vout, I0, I1);
  cout << Vout << endl;
  V1.clear();
  merged(V0, V1, Vout, I0, I1);
  cout << Vout << endl;
  VectorXd x = VectorXd::LinSpaced(10,0,9);
  cout << x << endl;
  typedef Matrix<GRBVar, Dynamic, Dynamic> VarMatrix;

#endif

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
    model.s
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
