#include "expr_ops.h"

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
