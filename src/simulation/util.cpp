#include "util.h"
#include <boost/foreach.hpp>
#include "conversions.h"

using namespace Eigen;

void toggle(bool* b){
	*b = !(*b);
}

void add(int* n, int increment) {
	*n += increment;
}

namespace util {

}
