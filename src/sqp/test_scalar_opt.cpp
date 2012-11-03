#include "sqp.h"
#include <cmath>
#include <limits>
#include "utils/logging.h"
#include "config_sqp.h"
#include <iostream>
#include <boost/format.hpp>
using namespace std;

class ScalarBox : public TrustRegion {
public:
	GRBVar& m_xvar;
	double& m_x;
	double m_radius;
	ScalarBox(GRBVar& xvar, double& x, double radius) : m_xvar(xvar), m_x(x), m_radius(radius) {}
	void adjustTrustRegion(double ratio) {
		m_radius *= ratio;
		m_shrinkage *= ratio;
		LOG_INFO_FMT("new radius: %.3f", m_radius);
	}
	ConvexConstraintPtr convexify() {		
		m_xvar.set(GRB_DoubleAttr_LB, m_x - m_radius);
		m_xvar.set(GRB_DoubleAttr_UB, m_x + m_radius);
		return ConvexConstraintPtr(new ConvexConstraint());
	}
};

class Polynomial : public Cost {
public:
	
	vector<double> m_coeffs;
	GRBVar& m_xvar;
	double& m_x;
	
	Polynomial(const vector<double>& coeffs, GRBVar& xvar, double& x) : m_coeffs(coeffs), m_xvar(xvar), m_x(x) {}
	
	double evaluate(double x) {
		double out = 0;
		for (int i=0; i < m_coeffs.size(); ++i) {
			out += pow(x,i) * m_coeffs[i];
		}
		return out;
	}
	double evaluate() {
		return evaluate(m_x);
	}
	
	ConvexObjectivePtr convexify() {
		double eps = 1e-4;
		double y = evaluate(m_x);
		double yminus = evaluate(m_x-eps/2);
		double yplus = evaluate(m_x+eps/2);
		double yprime = (yplus - yminus)/eps;
		double yprimeprime = (yplus + yminus - 2*y)/(eps*eps/4);
		ConvexObjectivePtr out(new ConvexObjective());
		out->m_objective = .5 * yprimeprime * (m_xvar - m_x)*(m_xvar - m_x) + yprime * (m_xvar - m_x) + y;
		return out;			
	}

	string getName() {
	  return "Polynomial";
	}

};


class ScalarOptimization : public OptimizationProblem {
public:
	GRBVar m_var;
	double m_val;
	double m_val_backup;
	ScalarOptimization() :
		OptimizationProblem(),
		m_val(std::numeric_limits<double>::quiet_NaN()) {}
	void updateValues() {
		cout << boost::format("x: %.2f -> %.2f")%m_val%(m_var.get(GRB_DoubleAttr_X)) << endl;
		m_val = m_var.get(GRB_DoubleAttr_X);
	}
	void storeValues() {
		m_val_backup = m_val;
	}
	void rollbackValues() {
		m_val = m_val_backup;
	}
	void initialize(double x) {
		m_val = x;
		m_var = m_model->addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "x");
		m_model->update();
	}
	void preOptimize() {
		printf("x before: %.2f\n", m_val);
	}
	void postOptimize() {
		printf("x after: %.2f\n", m_val);
	}
};
#if 0
class VectorOptimization : public OptimizationProblem {
public:
	vector<GRBVar> vecs;
	vector<double> vals;
	void updateVariableValues() {
		for (int i=0; i < vecs.size(); ++i) {
			vals[i] = vecs[i].get(GRB_DoubleAttr_X);
		}
	}
};
#endif

int main(int argc, char* argv[]) {
	Parser parser;
	parser.addGroup(GeneralConfig());
	parser.addGroup(SQPConfig());
	parser.read(argc, argv);
	initializeGRB();
	const int deg=4;
	double coeffs_a[deg+1] = {1,2,3,4,5};
	vector<double> coeffs(coeffs_a, coeffs_a+deg+1);
	ScalarOptimization opt;
	opt.initialize(3);
	opt.setTrustRegion(TrustRegionPtr(new ScalarBox(opt.m_var, opt.m_val, 5)));
	CostPtr p( new Polynomial(coeffs, opt.m_var, opt.m_val));
	opt.addCost(p);
	opt.optimize();
}
