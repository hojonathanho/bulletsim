#ifndef _scp_solver_h
#define _scp_solver_h

#include "robots.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>

/*
 * scp_solver interface is as follows: 
 * Robot r: used for abstracting the dynamics and observation models. %TODO: pass in a simulation environment instead. 
 * X_bar: a nominal trajectory of states with length T+1
 * U_bar: a nominal trajectory of controls with length T
 * W_bar: a nominal trajectory of noise samples, each matrix is NX*NS; vector of length T
 * rho_x: trust region constraint for noiseless state trajectory
 * rho_u: trust region constraint for noiseless control trajectory
 * x_goal: desired state
 * N_iter: number of iterations to run SCP
 * opt_X: return value containing states for noiseless trajectory
 * opt_U: return value containing controls for noiseless trajectory
 * K: return value containing control policy (Linear feedback part)
 * u0: return value containing control policy (affine offset part)
 */

void scp_solver(Robot &r, const vector<VectorXd>& X_bar,
		const vector<VectorXd>& U_bar, const vector<MatrixXd>& W_bar,
		const double rho_x, const double rho_u, const VectorXd& x_goal,
		const int N_iter, vector<VectorXd>& opt_X, vector<VectorXd>& opt_U,
		MatrixXd& K, VectorXd& u0);

#endif
