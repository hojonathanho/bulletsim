#include "sparse_utils.h"
using namespace Eigen;

void rowTimesMatrix(const Eigen::VectorXf& vec, SparseMatrixf& mat) { 
  for (int iRow=0; iRow<mat.rows(); ++iRow)
    for (SparseMatrixf::InnerIterator it(mat,iRow); it; ++it)
    {
      it.valueRef() *= vec(it.col());
    }   
}

void colTimesMatrix(const Eigen::VectorXf& vec, SparseMatrixf& mat) {
  for (int iRow=0; iRow<mat.rows(); ++iRow)
    mat.row(iRow) *= vec(iRow);
}

VectorXf rowSums(const SparseMatrixf& mat) {
  VectorXf out(mat.rows());
  for (int iRow=0; iRow<mat.rows(); ++iRow)
    out(iRow) = mat.row(iRow).sum();
  return out;
}

VectorXf colSums(const SparseMatrixf& mat) {
  VectorXf out = VectorXf::Zero(mat.cols());
  for (int iRow=0; iRow<mat.rows(); ++iRow)
    for (SparseMatrixf::InnerIterator it(mat,iRow); it; ++it)
    {
      out(it.col()) += it.value();
    } 
    return out;
}

SparseMatrixf toSparseMatrix(const Eigen::MatrixXf& in, float thresh) {
  Eigen::MatrixXf absIn = in.array().abs();
  SparseMatrixf out(in.rows(), in.cols());
  out.reserve(in.size() / 10);
  for (int iRow = 0; iRow < in.rows(); iRow++) {
    for (int iCol = 0;iCol < in.cols(); iCol++) {
      if (absIn(iRow, iCol) > thresh) out.insertBack(iRow, iCol) = in(iRow, iCol);
    }
  }
  return out;
}
