#include "utils_sqp.h"
#include "utils/logging.h"
#include <boost/format.hpp>
using namespace std;

VectorXd arange(int n) {
  VectorXd out(n);
  for (int i=0; i < n; ++i) out(i) = i;
  return out;
}

MatrixXd linearInterp(const Eigen::VectorXd& startJoints, const Eigen::VectorXd& endJoints, int nSteps) {
  MatrixXd out(nSteps, startJoints.size());
  for (int i=0; i < nSteps; ++i) {
    out.row(i) = ((double)i/(nSteps-1))*endJoints + ((double)(nSteps-1-i)/(nSteps-1))*startJoints;
  }
  return out;
}

VectorXd concatenate(VectorXd x0, VectorXd x1) {
  VectorXd out(x0.size()+x1.size());
  out.middleRows(0,x0.size()) = x0;
  out.middleRows(x0.size(),x1.size()) = x1;
  return out;
}

std::vector<double> toDoubleVec(const Eigen::VectorXd& in) {
  vector<double> out;
  out.assign(in.data(), in.data() + in.size());
  return out;
}

void printAllConstraints(const GRBModel& model) {
  GRBConstr* cnts = model.getConstrs();
  std::ostringstream active, inactive;
  for (int i = 0; i < model.get(GRB_IntAttr_NumConstrs); ++i) {
      double lambda = cnts[i].get(GRB_DoubleAttr_Pi);
//      double p=0;
      if (lambda > 1e-5) active << boost::format("%s (%.3f), ") % cnts[i].get(GRB_StringAttr_ConstrName) % lambda;
      else inactive << cnts[i].get(GRB_StringAttr_ConstrName) << ", ";
  }
  delete[] cnts;

  GRBVar* vars = model.getVars();

  for (int i=0; i < model.get(GRB_IntAttr_NumVars); ++i) {
    double lambda = vars[i].get(GRB_DoubleAttr_RC);
//      double p=0;
    if (lambda > 1e-4) active << boost::format("%s (%.3e), ") % vars[i].get(GRB_StringAttr_VarName) % lambda;
    else inactive << vars[i].get(GRB_StringAttr_VarName) << ", ";
  }
  delete[] vars;

  LOG_DEBUG("active: " << active.str());
  LOG_DEBUG("inactive: " << inactive.str());
}

void setValsToVars(VarVector& vars, Vector3d& vals) {
  assert(vars.size()==vals.size());
  for (int i=0; i < vars.size(); ++i) {
    vals(i) = vars[i].get(GRB_DoubleAttr_X);
  }
}
void setValsToVars(VarVector& vars, VectorXd& vals) {
  assert(vars.size()==vals.size());
  for (int i=0; i < vars.size(); ++i) {
    vals(i) = vars[i].get(GRB_DoubleAttr_X);
  }
}
void setValsToVars(VarArray& vars, MatrixXd& vals) {
  assert(vars.rows()==vals.rows());
  for (int i=0; i < vars.rows(); ++i) {
    for (int j=0; j < vars.cols(); ++j) {
      vals(i,j) = vars(i,j).get(GRB_DoubleAttr_X);
    }
  }
}


GRBLinExpr makeDerivExpr(const VectorXd& grad, const VarVector& vars, const VectorXd& curvals) {
  assert (grad.size() == vars.size());
  GRBLinExpr out;
  out.addTerms(grad.data(), vars.data(), curvals.size());
  out -= grad.dot(curvals);
  return out;
}
