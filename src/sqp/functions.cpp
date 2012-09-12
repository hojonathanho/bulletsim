#include "functions.h"

Eigen::VectorXd ScalarValuedFunction::calcNumDeriv(const Eigen::VectorXd& x, float spacing) {
	float y = calcVal(x);
	Eigen::VectorXd grad(x.size());
	Eigen::VectorXd xPert = x;
	for (int i=0; i < x.size(); ++i) {
		xPert(i) = x(i) + spacing/2;
		float yPlus = calcVal(xPert);
		xPert(i) = x(i) - spacing/2;
		float yMinus = calcVal(xPert);
		xPert(i) = x(i);
		grad(i) = (yPlus - yMinus) / (spacing);
	}
}
