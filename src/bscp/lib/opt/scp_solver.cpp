#include "scp_solver.h"
#include "gurobi_solver.h"
#include "timer.h"
#include "trajectory_util.h"
#include <algorithm>
#include "eigen_io_util.h"


void scp_solver(Robot &r, const vector<VectorXd>& X_bar, const vector<VectorXd>& U_bar, const vector<MatrixXd>& W_bar, const double rho_x, const double rho_u, const VectorXd& x_goal, const int N_iter, vector<VectorXd>& opt_X, vector<VectorXd>& opt_U, MatrixXd& opt_K, VectorXd& opt_u0) {

  int NX = X_bar[0].rows();
  int NU = U_bar[0].rows();
  int T = U_bar.size();
  int NS = W_bar[0].cols();
  assert(T+1 == X_bar.size());


  vector<VectorXd> X_scp(T+1), U_scp(T); 
  for (int i = 0; i < T+1; i++) 
    X_scp[i] = X_bar[i];
  for (int i = 0; i < T; i++)
    U_scp[i] = U_bar[i]; 



  TrajectoryInfo nominal = TrajectoryInfo(X_scp, U_scp, rho_x, rho_u);

  vector<vector<VectorXd> > W_s_bar;
  index_by_sample(W_bar, W_s_bar);
  vector<TrajectoryInfo> samples;
  for (int i = 0; i < NS; i++) {
	  samples.push_back(TrajectoryInfo(X_scp, U_scp, W_s_bar[i], rho_x, rho_u));
	  samples[i].integrate(r);
  }


  Timer timer = Timer();
  for (int iter = 0; iter < N_iter; iter++) 
  {

	nominal.update(r); // updates jacobians and bounds
	for (int s = 0; s < NS; s++) {
		samples[s].update(r);
	}

    // Setup variables for sending to convex solver
    vector<VectorXd> opt_X, opt_U;
    vector<vector<VectorXd> > opt_sample_X, opt_sample_U;
    //Send to convex solver
    convex_gurobi_solver(nominal, samples, x_goal, opt_X, opt_U, opt_K, opt_u0,
    		opt_sample_X, opt_sample_U, false);//iter == (N_iter - 1));

    nominal.set(opt_X, opt_U);
    nominal.integrate(r); //shooting

    for (int s = 0; s < NS; s++) {
    	samples[s].set(opt_X, opt_U); //hack for now
    	samples[s].integrate(r);
    }

    cout << "Iter " << iter << " time: " << timer.elapsed() << endl; 
  }


  opt_X = nominal._X;
  opt_U = nominal._U;


}

