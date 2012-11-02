#pragma once

#include <Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::VectorXd;
//#include <boost/function.hpp>
#include "utils_sqp.h"

struct fScalarOfScalar {
  virtual double operator()(const double&) const=0;
  virtual ~fScalarOfScalar(){}
};
struct fScalarOfVector {
  virtual double operator()(const Eigen::VectorXd&) const=0;
  virtual ~fScalarOfVector(){}
};
struct fScalarOfMatrix {
  virtual double operator()(const Eigen::MatrixXd&) const=0;
  virtual ~fScalarOfMatrix(){}
};

struct fVectorOfScalar {
  virtual VectorXd operator()(const double&) const=0;
  virtual ~fVectorOfScalar(){}
};
struct fVectorOfVector {
  virtual VectorXd operator()(const Eigen::VectorXd&) const=0;
  virtual ~fVectorOfVector(){}
};
struct fVectorOfMatrix {
  virtual VectorXd operator()(const Eigen::MatrixXd&) const=0;
  virtual ~fVectorOfMatrix(){}
};

struct fMatrixOfScalar {
  virtual MatrixXd operator()(const double&) const=0;
  virtual ~fMatrixOfScalar(){}
};
struct fMatrixOfVector {
  virtual MatrixXd operator()(const Eigen::VectorXd&) const=0;
  virtual ~fMatrixOfVector(){}
};
struct fMatrixOfMatrix {
  virtual MatrixXd operator()(const Eigen::MatrixXd&) const=0;
  virtual ~fMatrixOfMatrix(){}
};



Eigen::MatrixXd calcJacobian(const fVectorOfVector& f, const Eigen::VectorXd& x0, double epsilon=1e-5);
Eigen::MatrixXd matGrad(const fScalarOfMatrix& f, const Eigen::MatrixXd& x0, double epsilon=1e-5);

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
