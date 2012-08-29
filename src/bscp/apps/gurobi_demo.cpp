/* Copyright 2012, Gurobi Optimization, Inc. */

/* This example formulates and solves the following simple QP model:

     minimize    x + y + x^2 + x*y + y^2 + y*z + z^2
     subject to  x + 2 y + 3 z >= 4
                 x +   y       >= 1

   The example illustrates the use of dense matrices to store A and Q
   (and dense vectors for the other relevant data).  We don't recommend
   that you use dense matrices, but this example may be helpful if you
   already have your data in this format.
*/

#include "gurobi_c++.h"
#include <Eigen/Sparse>

using namespace std;
using namespace Eigen;

static bool
dense_optimize(GRBEnv* env,
               int     rows,
               int     cols,
               SparseVector<double>& c,     /* linear portion of objective function */
               SparseMatrix<double>& Q,     /* quadratic portion of objective function */
               SparseMatrix<double>& A,     /* constraint matrix */
               char*   sense, /* constraint senses */
               VectorXd & rhs,   /* RHS vector */
               double* lb,    /* variable lower bounds */
               double* ub,    /* variable upper bounds */
               char*   vtype, /* variable types (continuous, binary, etc.) */
               double* solution,
               double* objvalP)
{
  GRBModel model = GRBModel(*env);
  int i, j;
  bool success = false;

  /* Add variables to the model */

  GRBVar* vars = model.addVars(lb, ub, NULL, vtype, NULL, cols);
  model.update();

  GRBLinExpr lhs[cols]; 
  for (int i = 0; i < cols; i++) lhs[i] = 0;

  cout << "A" << endl; 
  for (int k = 0; k < A.outerSize(); ++k) {
    for (SparseMatrix<double>::InnerIterator it(A,k); it; ++it) {
      double value = A.coeff(it.row(), it.col()); 
      cout << "(" << it.row() << ", " << it.col() << ", " <<  value << ")" << endl; 
      lhs[it.row()] += value * vars[it.col()]; 
    }
  }

  for (int i = 0; i < cols; i++) 
    model.addConstr(lhs[i], sense[i], rhs(i));

  GRBQuadExpr obj = 0; 
  cout << "Q" << endl; 
  for (int k = 0; k < Q.outerSize(); ++k) {
    for (SparseMatrix<double>::InnerIterator it(Q,k); it; ++it) {
      double value = Q.coeff(it.row(), it.col()); 
      cout << "(" << it.row() << ", " << it.col() << ", " <<  value  << ")" << endl;
      obj += value * vars[it.row()]*vars[it.col()];
    }
  }
  cout << "c" << endl; 
  for (SparseVector<double>::InnerIterator it(c); it; ++it) {
    double value = c.coeff(it.index()); 
    cout << "(" << it.index() << ", " <<  value << ")" << endl;
    obj += value * vars[it.row()]; 
  }
  
  /* Populate A matrix */

  /*
  for (i = 0; i < rows; i++) {
    GRBLinExpr lhs = 0;
    for (j = 0; j < cols; j++)
      if (A[i*cols+j] != 0)
        lhs += A[i*cols+j]*vars[j];
    model.addConstr(lhs, sense[i], rhs[i]);
  }

  GRBQuadExpr obj = 0;

  for (j = 0; j < cols; j++)
    obj += c[j]*vars[j];
  for (i = 0; i < cols; i++)
    for (j = 0; j < cols; j++)
      if (Q[i*cols+j] != 0)
        obj += Q[i*cols+j]*vars[i]*vars[j];
  */ 

  model.setObjective(obj);

  model.update();
  //model.write("dense2.lp");

  model.optimize();

  if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
    *objvalP = model.get(GRB_DoubleAttr_ObjVal);
    for (i = 0; i < cols; i++)
      solution[i] = vars[i].get(GRB_DoubleAttr_X);
    success = true;
  }

  delete[] vars;
  return success;
}

int
main(int   argc,
     char *argv[])
{
  GRBEnv* env = 0;
  try {
    env = new GRBEnv();
    double c[] = {1, 1, 0};
    double  Q[3][3] = {{1, 1, 0}, {0, 1, 1}, {0, 0, 1}};
    double  A[2][3] = {{1, 2, 3}, {1, 1, 0}};
    char    sense[] = {'>', '>'};
    double  rhs[]   = {4, 1};
    double  lb[]    = {0, 0, 0};
    bool    success;
    double  objval, sol[3];

    SparseVector<double> spC(3,1);
    spC.coeffRef(0) = 1.0;
    spC.coeffRef(1) = 1.0;
    //spC.coeffRef(2) = 0.0;

    SparseMatrix<double> spQ(3,3); 
    spQ.coeffRef(0,0) = 1.0;
    spQ.coeffRef(0,1) = 1.0;
    //spQ.coeffRef(0,2) = 0.0;
    //spQ.coeffRef(1,0) = 0.0; 
    spQ.coeffRef(1,1) = 1.0;
    spQ.coeffRef(1,2) = 1.0;
    //spQ.coeffRef(2,0) = 0.0;
    //spQ.coeffRef(2,1) = 0.0;
    spQ.coeffRef(2,2) = 1.0; 

    SparseMatrix<double> spA(2,3);
    spA.coeffRef(0,0) = 1.0;
    spA.coeffRef(0,1) = 2.0;
    spA.coeffRef(0,2) = 3.0;
    spA.coeffRef(1,0) = 1.0;
    spA.coeffRef(1,1) = 1.0;
    //spA.coeffRef(1,2) = 0.0;
    
    VectorXd spRHS(2); 
    spRHS.coeffRef(0) = 4.0;
    spRHS.coeffRef(1) = 1.0;

    // previous x = 0.57, y = 0.42, z = 0.85
    success = dense_optimize(env, 2, 3, spC, spQ, spA, sense, spRHS,
                             NULL, NULL, NULL, sol, &objval);

    cout << "x: " << sol[0] << " y: " << sol[1] << " z: " << sol[2] << endl;

  } catch(GRBException e) {
    cout << "Error code = " << e.getErrorCode() << endl;
    cout << e.getMessage() << endl;
  } catch(...) {
    cout << "Exception during optimization" << endl;
  }

  delete env;
  return 0;
}
