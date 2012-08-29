#ifndef _gurobi_optimize_h
#define _gurobi_optimize_h

#include "gurobi_c++.h"
#include <Eigen/Sparse>
#include "eigen_sparse_util.h"
#include "timer.h"

using namespace Eigen;
using namespace std; 

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
               bool print_objective = true);
bool
solve_linear_system(GRBEnv* env,
                    const SparseMatrix<double>& A,
                    const VectorXd& b,
                    VectorXd& solution); 
bool
solve_matrix_linear_system(GRBEnv* env,
                          const SparseMatrix<double>& A,
                          const SparseMatrix<double>& B,
                          MatrixXd& solution);
#endif
