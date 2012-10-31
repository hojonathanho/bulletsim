#include "sqp.h"
#include <cmath>
#include <limits>
#include "utils/logging.h"
#include "config_sqp.h"
#include <iostream>
#include <boost/format.hpp>
#include "functions.h"
#include "utils_sqp.h"
using namespace std;

class VectorBox : public TrustRegion {
public:
  VarVector& m_xvar;
  VectorXd& m_x;
  VectorXd m_radii;
  VectorBox(VarVector& vars, VectorXd& vals, const VectorXd& radii) : m_xvar(vars), m_x(vals), m_radii(radii) {}
  void adjustTrustRegion(double ratio) {
    m_radii *= ratio;
    m_shrinkage *= ratio;
    LOG_INFO("new radii: " << m_radii);
  }
  ConvexConstraintPtr convexify() {
    for (int i=0; i < m_xvar.size(); ++i) {
      m_xvar[i].set(GRB_DoubleAttr_LB, m_x[i] - m_radii[i]);
      m_xvar[i].set(GRB_DoubleAttr_UB, m_x[i] + m_radii[i]);
    }
    return ConvexConstraintPtr(new ConvexConstraint());
  }
};

class NonlinearCostFunc : public Cost {
public:

  fScalarOfVector& m_f;
  vector<double> m_coeffs;
  VarVector& m_xvar;
  VectorXd& m_x;

  NonlinearCostFunc(fScalarOfVector& f, VarVector& xvar, VectorXd& x) : m_f(f), m_xvar(xvar), m_x(x) {}

  double evaluate(const VectorXd& x) {
    return m_f(x);
  }
  double evaluate() {
    return evaluate(m_x);
  }

  ConvexObjectivePtr convexify() {
    double eps = 1e-4;
    double y = evaluate(m_x);

    ConvexObjectivePtr out(new ConvexObjective());
    out->m_objective = y;
    VectorXd x = m_x;
    for (int i=0; i < m_x.size(); ++i) {
      x(i) = m_x(i) - eps/2;
      double yminus = evaluate(x);
      x(i) = m_x(i) + eps/2;
      double yplus = evaluate(x);
      double yprime = (yplus - yminus)/eps;
      double yprimeprime = (yplus + yminus - 2*y)/(eps*eps/4);
      out->m_objective += .5 * yprimeprime * (m_xvar[i] - m_x[i])*(m_xvar[i] - m_x[i])
          + yprime * (m_xvar[i] - m_x[i]);
      x(i) = m_x(i);
    }
    return out;
  }

};


class VectorOptimizer : public Optimizer {
public:
  vector<GRBVar> m_xvar;
  VectorXd m_x;
  VectorXd m_x_backup;
  VectorOptimizer() :
    Optimizer() {}
  void updateValues() {
    for (int i=0; i < m_x.size(); ++i) {
      m_x[i] = m_xvar[i].get(GRB_DoubleAttr_X);
    }
  }
  void storeValues() {
    m_x_backup = m_x;
  }
  void rollbackValues() {
    m_x = m_x_backup;
  }
  void initialize(const VectorXd& x) {
    m_x = x;
    m_xvar.resize(x.size());
    for (int i=0; i < x.size(); ++i) {
      char namebuf[5];
      sprintf(namebuf, "x_%i", i);
      m_xvar[i] = m_model->addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, namebuf);
    }
    m_model->update();
  }
  void preOptimize() {
    LOG_INFO("x before: " << m_x.transpose());
  }
  void postOptimize() {
    LOG_INFO("x after: " << m_x.transpose());
  }
};
#if 0
class VectorOptimization : public Optimizer {
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

struct QuadBasinTest : public fScalarOfVector {
  double operator()(const VectorXd& x) const {
    return pow(x(0)-1,2) + pow(x(1)-1,2);
  }
};

int main(int argc, char* argv[]) {
  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.addGroup(SQPConfig());
  parser.read(argc, argv);
  initializeGRB();
  VectorOptimization opt;
  opt.initialize(Eigen::Vector2d(0,0));
  opt.setTrustRegion(TrustRegionPtr(new VectorBox(opt.m_xvar, opt.m_x, Eigen::Vector2d::Constant(2,.1))));
  QuadBasinTest f;
  CostPtr cost( new NonlinearCostFunc(f, opt.m_xvar, opt.m_x));
  opt.addCost(cost);
  opt.optimize();
}
