#pragma once
#include "utils_sqp.h"

GRBLinExpr varDot(const VectorXd& x, const VarVector& v);
GRBLinExpr exprDot(const VectorXd& x, const ExprVector& v);
GRBQuadExpr varNorm2(const VarVector& v);
GRBQuadExpr exprNorm2(const ExprVector& v);

#include "expr_ops_autogen.h"
