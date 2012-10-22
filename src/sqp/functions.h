#pragma once

#include <Eigen/Dense>
#include <boost/function.hpp>
#include "utils_sqp.h"

struct fScalarOfMatrix {
	virtual double operator()(const Eigen::MatrixXd&) const=0;
};
struct fMatrixOfMatrix {
	virtual Eigen::MatrixXd operator()(const Eigen::MatrixXd&) const=0;
};

Eigen::MatrixXd matGrad(const fScalarOfMatrix& f, Eigen::MatrixXd& x0, double epsilon);
Eigen::MatrixXd getLinCoeffs(const GRBLinExpr&, const VarArray&);

class QuadFuncFromGRB {
  GRBQuadExpr m_expr;
  QuadFuncFromGRB(const GRBQuadExpr& expr) : m_expr(expr) {
  }
  double operator()(const Eigen::MatrixXd&, const VarArray&);
  double operator()(const Eigen::VectorXd&, const VarVector&);
};

void testMatGrad(const fScalarOfMatrix& f, const fMatrixOfMatrix& gradf, const Eigen::MatrixXd& x0, double epsilon);
double testMatGradBox(const fScalarOfMatrix& f, const Eigen::MatrixXd& x0, const Eigen::MatrixXd& grad, double scale, int nTest, bool allSame);

GRBLinExpr linearizationFromCostAndGrad(VarArray& xvars, const Eigen::MatrixXd& x0, double cost, const Eigen::MatrixXd& grad);
