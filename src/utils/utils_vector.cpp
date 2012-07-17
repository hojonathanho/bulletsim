#include "utils_vector.h"
#include <algorithm>

bool cwiseOr(const std::vector<bool>& v) {
	bool ret = false;
	for (int i=0; i<v.size(); i++)
		ret = ret || v[i];
	return ret;
}

bool cwiseAnd(const std::vector<bool>& v) {
	bool ret = true;
	for (int i=0; i<v.size(); i++)
		ret = ret && v[i];
	return ret;
}
