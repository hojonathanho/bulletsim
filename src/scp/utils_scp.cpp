#include "utils_scp.h"
using namespace std;
std::vector<double> toDoubleVec(const Eigen::VectorXf& in) {
	vector<double> out(in.size());
	for (int i = 0; i < in.size(); ++i)
		out[i] = in[i];
	return out;
}

