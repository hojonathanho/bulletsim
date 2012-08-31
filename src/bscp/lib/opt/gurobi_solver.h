#ifndef _gurobi_solver_h
#define _gurobi_solver_h

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "gurobi_optimize.h"
#include "eigen_sparse_util.h"
#include "timer.h"
#include "trajectory_util.h"

using namespace std;

void A_prod(const vector<MatrixXd> &As, int tau, int t, MatrixXd& result);

void build_G(const vector<MatrixXd> &As, SparseMatrix<double> &G);

void build_G_T1(const vector<MatrixXd> &As, SparseMatrix<double> &G_T1);

void build_H(const vector<MatrixXd> &As, const vector<MatrixXd> &Bs,
		SparseMatrix<double> &H);

void build_H_T1(const vector<MatrixXd> &As, const vector<MatrixXd> &Bs,
		SparseMatrix<double> &H_T1);

void build_x0(const vector<MatrixXd> &As, const vector<VectorXd> &Cs,
		const VectorXd &x_start, const SparseMatrix<double> &G, VectorXd &x0);

void build_kx(int NU, VectorXd& x0, SparseMatrix<double> &result);

void build_Qx(VectorXd x0, int NX, int NU, int T, SparseMatrix<double> &result);

void build_xbar(const vector<VectorXd>& X_bar, VectorXd& x_bar);

void build_ubar(const vector<VectorXd>& U_bar, VectorXd& u_bar);

void build_sample_matrix(const vector<MatrixXd> &W_bar, MatrixXd & W_s);

void convex_gurobi_solver(const TrajectoryInfo &nominal,
		const vector<TrajectoryInfo>& samples, const VectorXd& x_goal,
		vector<VectorXd>& opt_X, vector<VectorXd>& opt_U, MatrixXd &opt_K,
		VectorXd & opt_u0, vector<vector<VectorXd> >& opt_sample_X,
		vector<vector<VectorXd> >& opt_sample_U, const bool compute_policy);

//void convex_gurobi_solver(const vector<VectorXd>& X_bar, const vector<VectorXd>& U_bar, const vector<MatrixXd>& W_bar,
//		const vector<MatrixXd>& As, const vector<MatrixXd>& Bs, const vector<VectorXd>& Cs,
//		const vector<MatrixXd>& Ps, const vector<VectorXd>& p_offsets,
//		const vector<VectorXd>& X_upper_limit, const vector<VectorXd>& X_lower_limit,
//		const vector<VectorXd>& U_upper_limit, const vector<VectorXd>& U_lower_limit,
//		const VectorXd &x_goal,
//		vector<VectorXd>& opt_X, vector<VectorXd>& opt_U, MatrixXd &opt_K, VectorXd & opt_u0,
//		const bool compute_policy);

#endif
