#include "utils_sqp.h"
#include "utils/logging.h"
using namespace std;
std::vector<double> toDoubleVec(const Eigen::VectorXd& in) {
  vector<double> out;
  out.assign(in.data(), in.data()+in.size());
  return out;
}

void printAllConstraints(const GRBModel& model) {
  GRBConstr* cnts = model.getConstrs();
  std::ostringstream ss;
  ss << "constraints: ";
  for (int i=0; i < model.get(GRB_IntAttr_NumConstrs); ++i)
    ss << cnts[i].get(GRB_StringAttr_ConstrName) << " ";
  LOG_DEBUG(ss);
  delete[] cnts;
  cout << endl;

}
