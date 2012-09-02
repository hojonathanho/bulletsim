#include "registration.h"

#include <boost/python.hpp>
using namespace boost::python;

static void _tps_rpm_pywrapper(
  py::object &x_nd,
  py::object &y_md,
  int n_iter=5,
  double reg_init=.1,
  double reg_final=.001,
  double rad_init=.2,
  double rad_final=.001,
  bool plotting=false,
  bool verbose=true
) {





}


namespace lfd { namespace registration {

void tps_rpm() {
  object fun = import("lfd.registration").attr("tps_rpm");
}

} }
