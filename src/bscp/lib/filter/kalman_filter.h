#ifndef _kalman_filter_h
#define _kalman_filter_h 

#include <Eigen/Dense>
#include "robots.h"

using namespace Eigen; 
using namespace std; 

/* 
 * Square root Kalman filter
 * Robot r for dynamics and jacobians
 * x_t = estimate at time t
 * u_t = control input at time t
 * rt_Sigma_t = sqrt(Sigma_t), 
 * z_t1 = observation at time t, See RECV_OBSERVATION
 * M_t = sqrt(Q_t) where Q_t is process noise
 * N_t = sqrt(R_t) where R_t is measurement noise
 * x_t1 is the new mean
 * rt_Sigma_t1 is the update
 * RECV_OBSERVATION is used to denote whether z_t1 is a valid observation
 */
inline void ekf_update(Robot& r, const VectorXd& x_t, const VectorXd& u_t, const MatrixXd& rt_Sigma_t, const VectorXd& z_t1, const MatrixXd& M_t, const MatrixXd& N_t, VectorXd& x_t1, MatrixXd& rt_Sigma_t1, const bool RECV_OBSERVATION) {
  // Following notation from Jur Van Den Berg's journal paper on POMDPs 

  assert(r._NX == x_t.rows());
  assert(x_t.rows() == rt_Sigma_t.rows());
  assert(rt_Sigma_t.rows() == rt_Sigma_t.cols());
  assert(!RECV_OBSERVATION || z_t1.rows() == r._NZ);

  MatrixXd A;
  MatrixXd H;

  r.dfdx(x_t,u_t,A);
  r.dgdx(x_t,H);

  MatrixXd ARtSigma = A*rt_Sigma_t; 
  MatrixXd Gamma = ARtSigma * ARtSigma.transpose() + M_t * M_t.transpose();
  
  MatrixXd HGamma = H*Gamma; 

  MatrixXd A_K = (HGamma*H.transpose() + N_t*N_t.transpose());
  PartialPivLU<MatrixXd> solver(A_K); 
  MatrixXd K_transpose = solver.solve(HGamma);
  MatrixXd K = K_transpose.transpose();

  r.dynamics(x_t, u_t, x_t1);
  if (RECV_OBSERVATION) {
    VectorXd exp_obs;
    r.observe(x_t1, exp_obs); 
    x_t1 += K*(z_t1 - exp_obs);  
  }

  MatrixXd Sigma_t1 = Gamma - K*H*Gamma;
  LLT<MatrixXd> lltSigma_t1(Sigma_t1);
  rt_Sigma_t1 = lltSigma_t1.matrixL();
  //cout << "Sigma_t1" << endl; 
  //cout << Sigma_t1 << endl;

}

//wrapper for above only considering covariance
inline void ekf_cov_update(Robot &r, const VectorXd& x_t, const VectorXd& u_t, const MatrixXd& rt_Sigma_t, const MatrixXd& M_t, const MatrixXd& N_t, MatrixXd& rt_Sigma_t1) {

  VectorXd x_t1;
  ekf_update(r,x_t,u_t,rt_Sigma_t,VectorXd::Zero(0),M_t,N_t, x_t1, rt_Sigma_t1, false); 

}






#endif
