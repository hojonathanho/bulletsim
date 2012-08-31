#ifndef eigen_io_util_h
#define eigen_io_util_h

#include<Eigen/Dense>
#include<iostream>
#include<fstream>
#include<string>
#include<math.h>

using namespace Eigen;
using namespace std; 

inline void readMatrix(MatrixXd& m, istream &is){
  for (int y = 0; y < m.rows(); y++)  {
    for (int x = 0; x < m.cols(); x++) {
      is >> m(y,x); 
    }
  }

}

inline void readMatrixFromFile(MatrixXd &m, const char *filename) {
  ifstream ifs(filename);
  Vector2d line_1;
  ifs >> line_1(0);
  ifs >> line_1(1); 
  m.resize(line_1(0), line_1(1));
  readMatrix(m, ifs);
}

//rcov is sqrt(cov) = L
//vec is a vector representation
inline void rcov2vec(const MatrixXd& rcov, VectorXd& vec) {
  int c = 0; 
  int n = rcov.rows();
  vec = VectorXd(n*(n+1)/2);
  for (int i = 0; i < n; i++) { 
    for (int j = 0; j <= i; j++) {
      vec(c) = rcov(i,j);
      c += 1; 
    }
  }
}

inline void rvec2cov(const VectorXd& vec, MatrixXd& rcov) {
  int c = 0;
  int k = vec.rows(); 
  int n = (sqrt(8*k+1) - 1) / 2;

  rcov = MatrixXd::Zero(n,n);
  for (int i = 0; i < n; i++) { 
    for (int j = 0; j <= i; j++) {
      rcov(i,j) = vec(c);
      c += 1;
    }
  } 
}

inline void build_belief_state(const VectorXd& x, const MatrixXd& rt_Sigma, VectorXd& b) {
  int n = x.rows();
  b = VectorXd(n*(n+3) / 2); 
  b.segment(0,n) = x;

  VectorXd rt_Sigma_vec;
  rcov2vec(rt_Sigma, rt_Sigma_vec);
  b.segment(n, b.rows() - n ) = rt_Sigma_vec; 
}

inline void parse_belief_state(const VectorXd& b, VectorXd& x, MatrixXd& rt_Sigma) {
  int k = b.rows(); 
  int n = (sqrt(9+8*k)-3)/2;

  x = b.segment(0,n);
  VectorXd rt_Sigma_vec = b.segment(n,b.rows()-n);
  rvec2cov(rt_Sigma_vec, rt_Sigma); 
}

inline void index_by_sample(const vector<MatrixXd>& W_s, vector<vector<VectorXd> >& W_s_t) {
	int T = W_s.size();
	int NS = W_s[0].cols();

	W_s_t.resize(NS);
	for (int s = 0; s < NS; s++) {
		vector<VectorXd> sample_noise_traj(T);
		for (int t = 0; t < T; t++) {
			sample_noise_traj[t] = W_s[t].col(s);
		}
		W_s_t[s] = sample_noise_traj;
	}
}

inline void toFullVector(const vector<VectorXd>& v, VectorXd& vec) {
  int T = v.size();
  int NV = v[0].rows();
  vec = VectorXd(T*NV);
  for (int t = 0; t < T; t++) {
    vec.segment(t*NV, NV) = v[t];
  }
}






#endif
