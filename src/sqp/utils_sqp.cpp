#include "utils_sqp.h"
using namespace std;
std::vector<double> toDoubleVec(const Eigen::VectorXd& in) {
  vector<double> out;
  out.assign(in.data(), in.data()+in.size());
  return out;
}

