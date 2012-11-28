#pragma once
#include "utils_sqp.h"
using Eigen::Matrix3d;

Matrix3d leftCrossProdMat(const Vector3d& x);
Matrix3d rightCrossProdMat(const Vector3d& x);

ExprVector exprMatMult(const MatrixXd& A, const VarVector& x);
ExprVector exprMatMult(const MatrixXd& A, const ExprVector& x);

ExprVector exprCross(const VectorXd& x, const VarVector& y);
ExprVector exprCross(const VarVector& x, const VectorXd& y);
ExprVector exprCross(const VectorXd& x, const ExprVector& y);
ExprVector exprCross(const ExprVector& x, const VectorXd& y);

GRBLinExpr varDot(const VectorXd& x, const VarVector& v);
GRBLinExpr exprDot(const VectorXd& x, const ExprVector& v);

GRBQuadExpr varNorm2(const VarVector& v);
GRBQuadExpr exprNorm2(const ExprVector& v);

#include "expr_ops_autogen.h"
