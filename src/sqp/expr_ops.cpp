#include "expr_ops.h"

Matrix3d leftCrossProdMat(const Vector3d& x) {
  Matrix3d out;
  out << 0,   -x[2],   x[1],
      x[2],   0,     -x[0],
      -x[1],  x[0],    0;
  return out;
}
Matrix3d rightCrossProdMat(const Vector3d& x) {
  return leftCrossProdMat(x).transpose();
}

ExprVector exprMatMult(const MatrixXd& A, const VarVector& x) {
  ExprVector y(A.rows());
  for (int i=0; i < A.rows(); ++i) {
    y[i] = varDot(A.row(i), x);
  }
  return y;
}
ExprVector exprMatMult(const MatrixXd& A, const ExprVector& x) {
  ExprVector y(A.rows());
  for (int i=0; i < A.rows() ; ++i) {
    y[i] = exprDot(A.row(i), x);
  }
  return y;
}

ExprVector exprCross(const VectorXd& x, const VarVector& y) {
  return exprMatMult(leftCrossProdMat(x), y);
}
ExprVector exprCross(const VarVector& x, const VectorXd& y) {
  return exprMatMult(rightCrossProdMat(y), x);
}
ExprVector exprCross(const VectorXd& x, const ExprVector& y) {
  return exprMatMult(leftCrossProdMat(x), y);
}
ExprVector exprCross(const ExprVector& x, const VectorXd& y) {
  return exprMatMult(rightCrossProdMat(y), x);
}


GRBLinExpr varDot(const VectorXd& x, const VarVector& v) {
  assert(x.size() == v.size());
  GRBLinExpr out;
  out.addTerms(x.data(), v.data(), x.size());
  return out;
}
GRBLinExpr exprDot(const VectorXd& x, const ExprVector& v) {
  assert(x.size() == v.size());
  GRBLinExpr out;
  for (int i=0; i < x.size(); ++i) out += x[i] * v[i];
  return out;
}

GRBQuadExpr varNorm2(const VarVector& v) {
  GRBQuadExpr out(0);
  for (int i = 0; i < v.size(); ++i)
    out += v[i] * v[i];
  return out;
}

GRBQuadExpr exprNorm2(const ExprVector& v) {
  GRBQuadExpr out(0);
  for (int i = 0; i < v.size(); ++i)
    out += v[i] * v[i];
  return out;
}

#include "expr_ops_autogen.cpp"
