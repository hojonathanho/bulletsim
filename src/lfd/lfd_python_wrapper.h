#ifndef _LFD_PYTHON_WRAPPER_H_
#define _LFD_PYTHON_WRAPPER_H_

#include "utils_python.h"

namespace lfd {

class ExecutionModule : public PyModule {
public:
  ExecutionModule();

  void testing();

  py::object init(const string &task);
  py::object selectTrajectory(py::object points, py::object currRobotJointVals);
//  py::object selectTraj();
};

} // namespace lfd

#endif // _LFD_PYTHON_WRAPPER_H_
