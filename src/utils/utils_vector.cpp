#include "utils_vector.h"
#include <algorithm>
#include <sstream>

using namespace std;

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

// Converts the integer n to a string. If padding>0, then the result is padded
// with enough zeros such that the result has a length of padding (or greater
// if n has more than padding digits).
string itoa(int n, int padding) {
	stringstream ss;
	if (padding > 0) {
		for (int i=itoa(n, 0).size(); i<padding; i++)
			ss << 0;
	}
	ss << n;
	return ss.str();
}
