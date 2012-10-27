#pragma once

#include <Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::VectorXd;
//#include <boost/function.hpp>
#include "utils_sqp.h"

struct fScalarOfScalar {
  virtual double operator()(const double&) const=0;
};
struct fScalarOfVector {
  virtual double operator()(const Eigen::VectorXd&) const=0;
};
struct fScalarOfMatrix {
  virtual double operator()(const Eigen::MatrixXd&) const=0;
};

struct fVectorOfScalar {
  virtual VectorXd operator()(const double&) const=0;
};
struct fVectorOfVector {
  virtual VectorXd operator()(const Eigen::VectorXd&) const=0;
};
struct fVectorOfMatrix {
  virtual VectorXd operator()(const Eigen::MatrixXd&) const=0;
};

struct fMatrixOfScalar {
  virtual MatrixXd operator()(const double&) const=0;
};
struct fMatrixOfVector {
  virtual MatrixXd operator()(const Eigen::VectorXd&) const=0;
};
struct fMatrixOfMatrix {
  virtual MatrixXd operator()(const Eigen::MatrixXd&) const=0;
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
