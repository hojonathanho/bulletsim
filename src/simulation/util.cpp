#include "util.h"
#include "thread_socket_interface.h"
#include <boost/foreach.hpp>
#include "utils/conversions.h"

using namespace Eigen;

void toggle(bool* b){
	*b = !(*b);
}

void add(int* n, int increment) {
	*n += increment;
}

namespace util {

}
