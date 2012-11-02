#include "functions.h"
#include <iostream>
using namespace std;
using namespace Eigen;


MatrixXd calcJacobian(const fVectorOfVector& f, const Eigen::VectorXd& x0, double epsilon) {
  int nIn = x0.size();
  VectorXd y0 = f(x0);
  int nOut = y0.size();
  MatrixXd out(nOut, nIn);
  VectorXd x = x0;
  for (int i = 0; i < nIn; ++i) {
    x(i) = x0(i) + epsilon;
    out.col(i) = (f(x) - y0) / epsilon;
    x(i) = x0(i);
  }
  return out;
}

Eigen::MatrixXd matGrad(const fScalarOfMatrix& f, const Eigen::MatrixXd& x0, double epsilon) {
	double y0 = f(x0);
	MatrixXd grad(x0.rows(), x0.cols());
	MatrixXd x = x0;
	for (int i=0; i < x0.rows(); ++i) {
		for (int j=0; j < x0.cols(); ++j) {
			x(i,j) = x0(i,j) + epsilon;
			double y = f(x);
			grad(i,j) = (y-y0)/epsilon;
			x(i,j) = x0(i,j);
		}
	}
	return grad;
}

void testMatGrad(const fScalarOfMatrix& f, const fMatrixOfMatrix& gradf, const Eigen::MatrixXd& x0, double epsilon) {
	Eigen::MatrixXd gradNum = matGrad(f,x0,epsilon);
	Eigen::MatrixXd gradCalc = gradf(x0);
	cout << "GRADIENT TEST" << endl;
	cout << "row, col, numerical gradient, calculated gradient:" << endl;
	for (int i=0; i < x0.rows(); ++i) {
		for (int j=0; j < x0.cols(); ++j) {
			printf("%i %i %.4e %.4e\n", i, i, gradNum(i,j), gradCalc(i,j));
		}
	}
}

Eigen::VectorXd evalAtRandomPointsInBox(fScalarOfMatrix& f, const Eigen::MatrixXd& x0, double scale, int n) {
	Eigen::VectorXd out(n);
	for (int i=0; i < n; ++i) {
		out(i) = f(x0 + MatrixXd::Random(x0.rows(), x0.cols()) * scale);
	}
	return out;
}

double sign(double x) {
  return x < 0 ? -1 : +1;
}
MatrixXd matsign(const MatrixXd& x) {
  MatrixXd y(x.rows(), x.cols());
  for (int i=0; i < x.rows(); ++i)
    for (int j=0; j < x.cols(); ++j)
      y(i,j) = sign(x(i,j));
  return y;
}

double testMatGradBox(const fScalarOfMatrix& f, const Eigen::MatrixXd& x0, const Eigen::MatrixXd& grad, double scale, int nTest, bool allSame) {
	
	// Eigen::MatrixXd gradNum = matGrad(f,x0,epsilon);
	double y0 = f(x0);

	VectorXd dytrue(nTest);
	VectorXd dypred(nTest);

	for (int i=0; i < nTest; ++i) {
	  MatrixXd dx=MatrixXd::Random(x0.rows(), x0.cols()) * scale;
	  if (allSame) {
	    dx = matsign(dx)*scale;
	  }
		dytrue(i) = f(x0+dx) - y0;
		dypred(i) = grad.cwiseProduct(dx).sum();
	}

	dytrue.array() -= dytrue.mean();
	dypred.array() -= dypred.mean();
	double cov = dytrue.dot(dypred) / (dytrue.norm() * dypred.norm());
	return cov;
}

GRBLinExpr linearizationFromCostAndGrad(VarArray& xvars, const MatrixXd& x0, double cost, const MatrixXd& grad) {
  GRBLinExpr out(0);
#if 0
  for (int i=0; i < grad.rows(); ++i)
    for (int j=0; j<grad.cols(); ++j)
      out += xvars(i,j) * grad(i,j);
#else
  out.addTerms(grad.data(), xvars.data(), grad.rows()*grad.cols());
#endif
  out += cost - grad.cwiseProduct(x0).sum();
  return out;
}
