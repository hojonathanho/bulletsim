#ifndef _eigen_sparse_util_h
#define _eigen_sparse_util_h

#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <Eigen/Core> 

using namespace Eigen;
using namespace std; 

inline SparseMatrix<double> toSparse(const MatrixXd &input) { 
  SparseMatrix<double> output(input.rows(), input.cols());
  for (int i = 0; i < input.rows(); i++) {
    for (int j = 0; j < input.cols(); j++) {
      if (abs(input(i,j)) > 1e-30) {
        output.coeffRef(i,j) = input(i,j); 
      }
    }
  }
  return output; 
}

inline SparseVector<double> toSparseVector(const VectorXd &input) {
  SparseVector<double> output(input.rows());
  for (int i = 0; i < input.rows(); i++) { 
    if (abs(input(i)) > 1e-30) {
      output.coeffRef(i) = input(i); 
    }
  }
  return output; 
}


inline SparseMatrix<double> speye(int n) { 
  SparseMatrix<double> output(n,n); 
  for (int i = 0; i < n; i++) {
    output.coeffRef(i,i) = 1.0; 
  }
  return output;
}

inline SparseMatrix<double> spzeros(int m, int n) { 
  return SparseMatrix<double>(m,n); 
}

inline SparseMatrix<double> spdiag(const VectorXd& diag) { 
  SparseMatrix<double> output(diag.rows(), diag.rows());
  for (int i = 0; i < diag.rows(); i++) {
    if (abs(diag(i)) >= 1e-30) {
      output.coeffRef(i,i) = diag(i);
    }
  } 
  return output; 
}

inline MatrixXd toDense(const SparseMatrix<double> &input) { 
  MatrixXd output(input.rows(), input.cols());
  for (int k = 0; k < input.outerSize(); ++k) {
    for (SparseMatrix<double>::InnerIterator it(input,k); it; ++it) {
      double value = input.coeff(it.row(), it.col());
      output(it.row(), it.col()) = value; 
    }
  }
  return output; 

}

inline void vcat(const SparseMatrix<double>& A, const SparseMatrix<double>& B, SparseMatrix<double,RowMajor>& result) {
  assert(A.cols() == B.cols());
  result = SparseMatrix<double,RowMajor>(A.rows() + B.rows(), A.cols());
  result.middleRows(0,A.rows()) = A;
  result.middleRows(A.rows(), B.rows()) = B; 
}

inline void hcat(const SparseMatrix<double>& A, const SparseMatrix<double>& B, SparseMatrix<double,ColMajor>& result) {
  assert(A.rows() == B.rows());
  result = SparseMatrix<double,ColMajor>(A.rows(), A.cols() + B.cols());
  result.middleCols(0,A.cols()) = A;
  result.middleCols(A.cols(), B.cols()) = B; 
}

// this method is "functional", it can work as vcat(A, B, A) -> A = [A; B] 
// but you need to copy the result, so might not be ideal 
inline SparseMatrix<double,RowMajor> vcat(const SparseMatrix<double>& A, const SparseMatrix<double>& B) {
//  assert(A.cols() == B.cols());
//  SparseMatrix<double,RowMajor> r(A.rows() + B.rows(), A.cols());
//  r.middleRows(0,A.rows()) = A;
//  r.middleRows(A.rows(), B.rows()) = B;
  SparseMatrix<double,RowMajor> r; 
  vcat(A, B, r); 
  return r; 
}
// this method is "functional"; it can work as hcat(A, B, A) -> A = [A, B] 
// but you need to copy the result, so might not be ideal 
inline SparseMatrix<double,ColMajor> hcat(const SparseMatrix<double>& A, const SparseMatrix<double>& B) {
//  assert(A.rows() == B.rows());
//  SparseMatrix<double,ColMajor>r (A.rows(), A.cols() + B.cols());
//  r.middleCols(0,A.cols()) = A;
//  r.middleCols(A.cols(), B.cols()) = B;
  SparseMatrix<double,ColMajor> r;
  hcat(A, B, r); 
  return r; 
}
inline void blkdiag(const SparseMatrix<double>& A, const SparseMatrix<double>& B, SparseMatrix<double,ColMajor>& result) {
  // concatenate A vertically with a block of zeros of size (B.rows, A.cols)
  SparseMatrix<double>block12(B.rows(), A.cols());
  SparseMatrix<double,RowMajor>block1;
  vcat(A, block12, block1);
  // concatenate B vertically with a block of zeros of size (A.rows, B.cols)
  SparseMatrix<double>block21(A.rows(), B.cols());
  SparseMatrix<double,RowMajor>block2;
  vcat(block21, B, block2);
  // horizontally concatenate the two blocks
  hcat(block1, block2, result); 
}

inline void blkdiag(const SparseMatrix<double>& A, const SparseMatrix<double>& B, SparseMatrix<double,RowMajor>& result) {
  // concatenate A horizontally with a block of zeros of size (A.rows, B.cols)
  SparseMatrix<double>block21(A.rows(), B.cols());
  SparseMatrix<double>block1;
  hcat(A, block21, block1);
  // concatenate B horizontally with a block of zeros of size (B.rows, A.cols)
  SparseMatrix<double>block12(B.rows(), A.cols());
  SparseMatrix<double>block2;
  hcat(block12, B, block2);
  // vertically concatenate the two blocks
  vcat(block1, block2, result); 
}

inline SparseMatrix<double> blkdiag(const SparseMatrix<double>& A, const SparseMatrix<double>& B) {
  SparseMatrix<double> r;
  blkdiag(A, B, r);
  return r; 
}


#endif


