#include "utils_sqp.h"
#include "utils/logging.h"
#include <boost/format.hpp>
using namespace std;
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
