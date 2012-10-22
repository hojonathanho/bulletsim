#ifndef _LFD_REGISTRATION_H_
#define _LFD_REGISTRATION_H_

namespace lfd { namespace registration {

void tps_rpm(
  py::object &x_nd,
  py::object &y_md,
  int n_iter=5,
  double reg_init=.1,
  double reg_final=.001,
  double rad_init=.2,
  double rad_final=.001,
  bool plotting=false,
  bool verbose=true
);

} }

#endif // _LFD_REGISTRATION_H_
