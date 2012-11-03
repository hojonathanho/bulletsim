#include "expr_ops.h"

GRBQuadExpr exprNorm2(const ExprVector& v) {
  GRBQuadExpr out(0);
  for (int i = 0; i < v.size(); ++i)
    out += v[i] * v[i];
  return out;
}

#include "expr_ops_autogen.cpp"
