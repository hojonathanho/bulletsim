#include "scp_solver.h"
#include "gurobi_solver.h"
#include "timer.h"
#include <algorithm>

void scp_solver(Robot &r, const vector<VectorXd>& X_bar, const vector<VectorXd>& U_bar, const vector<MatrixXd>& W_bar, const double rho_x, const double rho_u, const VectorXd& x_goal, const int N_iter, vector<VectorXd>& opt_X, vector<VectorXd>& opt_U, MatrixXd& opt_K, VectorXd& opt_u0, const bool BELIEF_DYNAMICS) { 

  int NX = X_bar[0].rows();
  int NU = U_bar[0].rows();
  int T = U_bar.size();
  assert(T+1 == X_bar.size());

  vector<VectorXd> X_scp(T+1), U_scp(T); 
  for (int i = 0; i < T+1; i++) 
    X_scp[i] = X_bar[i];
  for (int i = 0; i < T; i++)
    U_scp[i] = U_bar[i]; 


  Timer timer = Timer();
  for (int iter = 0; iter < N_iter; iter++) 
  {

    // Compute Jacobians
    vector<MatrixXd> As(T), Bs(T), Ps(T+1);
    vector<VectorXd> Cs(T), p_offsets(T+1);
    
    if (BELIEF_DYNAMICS) {
      r.db_trajectory(X_scp, U_scp, As, Bs, Cs);
      r.dbp_trajectory(X_scp, Ps, p_offsets);
    }
    else {
      r.df_trajectory(X_scp, U_scp, As, Bs, Cs); 
      r.dp_trajectory(X_scp, Ps, p_offsets);
    }
    


    // construct variable bounds
    vector<VectorXd> X_upper_limit(T+1), X_lower_limit(T+1), U_upper_limit(T), U_lower_limit(T);

    for (int i = 0; i < T+1; i++) {
    	VectorXd upper_X_t(NX);
    	VectorXd lower_X_t(NX);
    	for (int j = 0; j < NX; j++) {
    		lower_X_t(j) = max(-rho_x, r.x_lower_limit(j) - X_scp[i][j]);
    		upper_X_t(j) = min(rho_x, r.x_upper_limit(j)  - X_scp[i][j]);
    	}
    	X_lower_limit[i] = lower_X_t;
    	X_upper_limit[i] = upper_X_t;
     }
    for (int i = 0; i < T; i++) {
    	VectorXd upper_U_t(NU);
     	VectorXd lower_U_t(NU);
     	for (int j = 0; j < NU; j++) {
     		lower_U_t(j) = max(U_scp[i][j] - rho_u, r.u_lower_limit(j));
     		upper_U_t(j) = min(U_scp[i][j] + rho_u, r.u_upper_limit(j));
     	}
     	U_lower_limit[i] = lower_U_t;
     	U_upper_limit[i] = upper_U_t;
    }




    // Setup variables for sending to convex solver
    vector<VectorXd> opt_X, opt_U;
    //SparseMatrix<double> opt_Q;
    //VectorXd opt_r;

    //Send to convex solver
    //convex_gurobi_solver(X_scp, U_scp, W_bar, As, Bs, Cs, X_upper_limit, X_lower_limit, U_upper_limit, U_lower_limit, x_goal, opt_X, opt_U, opt_K, opt_u0, iter == (N_iter - 1));
    convex_gurobi_solver(X_scp, U_scp, W_bar, As, Bs, Cs, Ps, p_offsets,
    		X_upper_limit, X_lower_limit, U_upper_limit, U_lower_limit, x_goal, opt_X, opt_U, opt_K, opt_u0, iter == (N_iter - 1));

    //Shoot
    
    for (int t = 0; t < T; t++) {
      U_scp[t] = opt_U[t];
      if (BELIEF_DYNAMICS) 
        r.belief_dynamics(X_scp[t], U_scp[t], X_scp[t+1]);
      else 
        r.dynamics(X_scp[t], U_scp[t], X_scp[t+1]);
    }

    // Collocation
    //X_scp = opt_X;
    //U_scp = opt_U;

    //if (iter < N_iter - 1) // hack for now  
    //  X_scp[T] = x_goal;
    cout << "Iter " << iter << " time: " << timer.elapsed() << endl; 
  }

  opt_X = X_scp;
  opt_U = U_scp; 


}

