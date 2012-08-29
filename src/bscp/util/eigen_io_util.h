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







#endif
