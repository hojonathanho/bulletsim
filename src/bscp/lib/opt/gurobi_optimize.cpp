#include "gurobi_optimize.h"

bool
dense_optimize(GRBEnv* env, 
               const int   NUM_VARIABLES,
               const SparseVector<double>& c,     /* linear portion of objective function */
               const SparseMatrix<double>& Q,     /* quadratic portion of objective function */
               const SparseMatrix<double,RowMajor>& A,     /* constraint matrix */
               char*   sense, /* constraint senses */
               const VectorXd & rhs,   /* RHS vector */
               double* lb,    /* variable lower bounds */
               double* ub,    /* variable upper bounds */
               char*   vtype, /* variable types (continuous, binary, etc.) */
               VectorXd& solution,
               double* objvalP,
               bool print_objective)
{
  GRBModel model = GRBModel(*env);
  int i, j;
  bool success = false;
  Timer timer = Timer();

  assert(A.rows() == rhs.rows()); 
  assert(Q.rows() == Q.cols());
  assert(Q.rows() == NUM_VARIABLES);
  assert(c.rows() == NUM_VARIABLES); 
  int NUM_CONSTRAINTS = rhs.rows();
  //model.getEnv().set(GRB_IntParam_OutputFlag, 0);
  //model.getEnv().set(GRB_IntParam_CrossoverBasis,1);
  //model.getEnv().set(GRB_DoubleParam_MarkowitzTol, 0.75);
  //model.getEnv().set(GRB_IntParam_Method, 1);
  //model.getEnv().set(GRB_IntParam_Presolve, 0);

  /* Add variables to the model */

  GRBVar* vars = model.addVars(lb, ub, NULL, vtype, NULL,NUM_VARIABLES);
  model.update();

  GRBLinExpr lhs[NUM_CONSTRAINTS]; 
  for (int i = 0; i < NUM_CONSTRAINTS; i++) 
    lhs[i] = 0;

  for (int k = 0; k < A.outerSize(); ++k) {
    for (SparseMatrix<double,RowMajor>::InnerIterator it(A,k); it; ++it) {
      double value = A.coeff(it.row(), it.col()); 
      //cout << "(" << it.row() << ", " << it.col() << ", " <<  value << ")" << endl; 
      lhs[it.row()] += value * vars[it.col()]; 
    }
  }
  
  for (int i = 0; i < NUM_CONSTRAINTS; i++) 
    model.addConstr(lhs[i], sense[i], rhs(i));

  GRBQuadExpr obj = 0; 
  //cout << "Q" << endl; 
  for (int k = 0; k < Q.outerSize(); ++k) {
    for (SparseMatrix<double>::InnerIterator it(Q,k); it; ++it) {
      double value = Q.coeff(it.row(), it.col()); 
      //cout << "(" << it.row() << ", " << it.col() << ", " <<  value  << ")" << endl;
      obj += value * vars[it.row()]*vars[it.col()];
    }
  }
  //cout << "c" << endl; 
  for (SparseVector<double>::InnerIterator it(c); it; ++it) {
    double value = c.coeff(it.index()); 
    //cout << "(" << it.index() << ", " <<  value << ")" << endl;
    obj += value * vars[it.row()]; 
  }
  
  model.setObjective(obj);

  model.update();
  //model.write("dense2.lp");
  
  timer.restart();
  //cout << "optimize start" << endl; 
  model.optimize();
  //cout << "optimize time: " << timer.elapsed() << endl; 

  if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL || true) {
    solution = VectorXd(NUM_VARIABLES); 
    *objvalP = model.get(GRB_DoubleAttr_ObjVal);
    if (print_objective) 
      cout << "Objective value: " << *objvalP << endl;
    for (i = 0; i < NUM_VARIABLES; i++)
      solution(i) = vars[i].get(GRB_DoubleAttr_X);
    success = true;
  }

  delete[] vars;
  //delete env; 
  return success;
}


bool
solve_matrix_linear_system(GRBEnv* env,
                          const SparseMatrix<double>& A,
                          const SparseMatrix<double>& B,
                          MatrixXd& solution) {

  /*
  int m = A.rows(); // also B.rows()
  int n = A.cols();
  int p = B.cols();

  VectorXd b_vec(m*p);
  for (int i = 0; i < p; i++) {
    for (int j = 0; j < m; j++) {
      b_vec(i*m + j) = B.coeff(j,i);GRB_EQUAL,
  SparseMatrix<double> A_blkdiag;
  for (int i = 0; i < p; i++) {
    //A_blkdiag = blkdiag(A_blkdiag, A);
  }

  VectorXd sol_vec; 
  bool success = solve_linear_system(env, A_blkdiag, b_vec, sol_vec);

  solution = MatrixXd(n, p);
  for (int i = 0; i < p; i++) {
    solution.col(i) = sol_vec.segment(i*n, n); 
  }
  */
  GRBModel model = GRBModel(*env);
  int i, j;
  bool success = false;
  Timer timer = Timer();
  //model.getEnv().set(GRB_IntParam_OutputFlag, 0);
  
  assert(A.rows() == B.rows());
  int m = A.rows();
  int n = A.cols();
  int p = B.cols(); 
  int NUM_VARIABLES = n*p; 
  int NUM_CONSTRAINTS = m*p;

  //cout << "m = " << m << endl;
  //cout << "n = " << n << endl;
  //cout << "p = " << p << endl;

  /* Add variables to the model */

  double lb[NUM_VARIABLES], ub[NUM_VARIABLES];
  // default bounds 
  for (int i = 0; i < NUM_VARIABLES; i++) { 
    lb[i] = -1e30; // treated as - infinity
    ub[i] =  1e30; // treated as + infinity 
  }

  GRBVar* vars = model.addVars(lb, ub, NULL, NULL, NULL, NUM_VARIABLES);
  model.update();

  GRBLinExpr lhs[NUM_CONSTRAINTS]; 
  for (int i = 0; i < NUM_CONSTRAINTS; i++) 
    lhs[i] = 0;

  for (int k = 0; k < A.outerSize(); ++k) {
    for (SparseMatrix<double>::InnerIterator it(A,k); it; ++it) {
      double value = A.coeff(it.row(), it.col());
      int row = it.row();
      int col = it.col();
      for (int repeat = 0; repeat < p; repeat++) { 
      //cout << "(" << it.row() << ", " << it.col() << ", " <<  value << ")" << endl;

        lhs[repeat*m + row] += value * vars[repeat*n + col];

      }
    }
  }
  
  for (int i = 0; i < NUM_CONSTRAINTS; i++)  
    model.addConstr(lhs[i], GRB_EQUAL, B.coeff(i%m,i/m));

  GRBLinExpr obj = 0; 
  
  model.setObjective(obj);

  model.update();
  //model.write("dense3.lp");
  
  cout << "starting optimize" << endl; 
  model.optimize();
  cout << "optimize time: " << timer.elapsed() << endl; 

  VectorXd sol(NUM_VARIABLES); 
  if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
    for (i = 0; i < NUM_VARIABLES; i++)
      sol(i) = vars[i].get(GRB_DoubleAttr_X);
    success = true;
  } 

  solution = MatrixXd(n,p);
  for (int i = 0; i < p; i++) { 
    solution.col(i) = sol.segment(i*n, n); 
  }

  delete[] vars;
  //delete env; 
  return success;

} 

bool
solve_linear_system(GRBEnv* env,
                    const SparseMatrix<double>& A,
                    const VectorXd& b,
                    VectorXd& solution
    )
{
	SparseVector<double> sparse_b = toSparseVector(b);
	MatrixXd mat_sol;
	solve_matrix_linear_system(env, A, sparse_b, mat_sol);
	solution = mat_sol.col(0);

//  int NUM_VARIABLES = A.cols();
//  int NUM_CONSTRAINTS = A.rows();
//  SparseVector<double> c = toSparseVector(VectorXd::Zero(NUM_VARIABLES));
//  SparseMatrix<double> Q = spzeros(NUM_VARIABLES, NUM_VARIABLES);
//  char sense[NUM_CONSTRAINTS];
//  for (int i = 0; i < NUM_CONSTRAINTS; i++) {
//    sense[i] = GRB_EQUAL;
//  }
//  double lb[NUM_VARIABLES], ub[NUM_VARIABLES];
//  // default bounds
//  for (int i = 0; i < NUM_VARIABLES; i++) {
//    lb[i] = -1e30; // treated as - infinity
//    ub[i] =  1e30; // treated as + infinity
//  }
//
//  double objvalP;
//
//  return dense_optimize(env,NUM_VARIABLES,c,Q,A,sense,b,lb,ub,NULL,solution,&objvalP, false);

}
