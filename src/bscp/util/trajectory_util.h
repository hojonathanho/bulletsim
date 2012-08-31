#ifndef _trajectory_util_h
#define _trajectory_util_h

#include <Eigen/Dense>
#include <vector>
#include "robots.h"
#include "eigen_io_util.h"
#include "eigen_sparse_util.h"
#include <algorithm>
using namespace Eigen;
using namespace std;



struct TrajectoryInfo {
	//states, controls, and noise
	vector<VectorXd> _X;
	vector<VectorXd> _U;
	vector<VectorXd> _W;

	// dynamics jacobians
	vector<MatrixXd> _A;
	vector<MatrixXd> _B;
	vector<VectorXd> _C;

	// collision jacobians
	vector<MatrixXd> _P;
	vector<VectorXd> _p_offset;

	// state and control limits around current trajectory
    vector<VectorXd> _X_upper_limit, _X_lower_limit, _U_upper_limit, _U_lower_limit;

	// trust region bounds
	double _rho_x;
	double _rho_u;

	//these are intended to be private

	TrajectoryInfo(const vector<VectorXd>& X, const vector<VectorXd>& U, const vector<VectorXd>& W, const double rho_x=1e20, const double rho_u=1e20) {
		init(X, U, W, rho_x, rho_u);
	}
	TrajectoryInfo(const vector<VectorXd>& X, const vector<VectorXd>& U, const double rho_x=1e20, const double rho_u=1e20) {
		int T = U.size();
		int NX = X[0].rows();
		vector<VectorXd> W(T);
		for (int t = 0; t < T; t++ )
			W[t] = VectorXd::Zero(NX);
		init(X, U, W, rho_x, rho_u);
	}

	TrajectoryInfo(const VectorXd x0) {
		vector<VectorXd> x(0);
		x.push_back(x0);
		vector<VectorXd> u(0);
		vector<VectorXd> w(0);
		init(x,u,w,1e20,1e20);
	}

	void init(const vector<VectorXd>& X, const vector<VectorXd>& U, const vector<VectorXd>& W, const double rho_x, const double rho_u) {
		_X = X;
		_U = U;
		_W = W;
		_rho_x = rho_x;
		_rho_u = rho_u;

//		cout << "T = " << _T << endl;
//		cout << "NX = " << _NX << endl;
//		cout << "NU = " << _NU << endl;
	}

	VectorXd X_vec() const {
		VectorXd x_vec;
		toFullVector(_X, x_vec);
		return x_vec;
	}

	VectorXd U_vec() const {
		VectorXd u_vec;
		toFullVector(_U, u_vec);
		return u_vec;
	}

	VectorXd W_vec() const {
		VectorXd w_vec;
		toFullVector(_W, w_vec);
		return w_vec;
	}

	SparseMatrix<double> build_G() {
	  int T = _U.size();
	  int NX = _X[0].rows();

	  SparseMatrix<double> G(NX,0);
	  SparseMatrix<double> block_A;
	  for (int i = T; i > 0; i--) {
	    if (i == T) block_A = speye(NX);
	    else block_A = block_A * toSparse(_A[i]);
	    G = hcat(block_A, G);
	  }

	  return G;
	}

	MatrixXd A_prod(int tau, int t) {
	  int NX = _X[0].rows();
	  MatrixXd result = MatrixXd::Identity(NX,NX);
	  for (int i = tau; i < t; i++) {
	    result = _A[i] * result;
	  }
	  return result;
	}

	void add_and_integrate(const VectorXd& u, const VectorXd& w, Robot &r) {
		_U.push_back(VectorXd(u));
		_W.push_back(VectorXd(w));
		integrate(r);
	}

	VectorXd Q_feedback(Robot &r)  {
		int _T, _NU, _NX;

		_T = _U.size();
		if (_T == 0) {
			return _X[0];
		}
		else {
			_NX = _X[0].rows();
			_NU = _U[0].rows();
			update(r);
			SparseMatrix<double> G = build_G();
			VectorXd c_vec; toFullVector(_C, c_vec);
			VectorXd w_vec = W_vec();
			VectorXd feedback = A_prod(0,_T) * _X[0] + G*(c_vec+w_vec);
			return feedback;
		}

	}


	void set(const vector<VectorXd>& X, const vector<VectorXd>& U) {
		_X = X;
		_U = U;
	}

	void integrate(Robot &r) {
		VectorXd x0 = _X[0];
		r.forward_integrate(x0, _U, _W, _X);
	}

	void update(Robot &r) {
		int _T, _NU, _NX;

		_T = _U.size();
		_NX = _X[0].rows();
		_NU = _U[0].rows();


	    if (r._NB == _NX) {
	      r.db_trajectory(_X, _U, _A, _B, _C);
	      r.dbp_trajectory(_X, _P, _p_offset);
	    }
	    else {
	      r.df_trajectory(_X, _U, _A, _B, _C);
	      r.dp_trajectory(_X, _P, _p_offset);
	    }


	    _X_upper_limit.resize(_T+1);
	    _X_lower_limit.resize(_T+1);

	    for (int i = 0; i < _T+1; i++) {
	    	VectorXd upper_X_t(_NX);
	    	VectorXd lower_X_t(_NX);
	    	for (int j = 0; j < _NX; j++) {
	    		lower_X_t(j) = max(-_rho_x, r.x_lower_limit(j) - _X[i][j]);
	    		upper_X_t(j) = min(_rho_x, r.x_upper_limit(j)  - _X[i][j]);
	    	}
	    	_X_lower_limit[i] = lower_X_t;
	    	_X_upper_limit[i] = upper_X_t;
	     }


	    _U_upper_limit.resize(_T);
	    _U_lower_limit.resize(_T);
	    for (int i = 0; i < _T; i++) {
	    	VectorXd upper_U_t(_NU);
	     	VectorXd lower_U_t(_NU);
	     	for (int j = 0; j < _NU; j++) {
	     		lower_U_t(j) = max(_U[i][j] - _rho_u, r.u_lower_limit(j));
	     		upper_U_t(j) = min(_U[i][j] + _rho_u, r.u_upper_limit(j));
	     	}
	     	_U_lower_limit[i] = lower_U_t;
	     	_U_upper_limit[i] = upper_U_t;
	    }


	}


};


#endif
