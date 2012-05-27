#pragma once
#include <Eigen/Sparse>


typedef Eigen::SparseMatrix<float, Eigen::RowMajor> SparseMatrixf;
typedef Eigen::SparseVector<float, Eigen::RowMajor> SparseVectorf;

void rowTimesMatrix(const Eigen::VectorXf& row, SparseMatrixf&);
void colTimesMatrix(const Eigen::VectorXf& col, SparseMatrixf&);
Eigen::VectorXf rowSums(const SparseMatrixf&);
Eigen::VectorXf colSums(const SparseMatrixf&);

SparseMatrixf toSparseMatrix(const Eigen::MatrixXf& in, float thresh);
