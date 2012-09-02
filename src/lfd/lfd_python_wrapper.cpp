#include "lfd_python_wrapper.h"

namespace lfd {

ExecutionModule::ExecutionModule() : PyModule("/home/jonathan/python/lfd/execution.py") {
}

void ExecutionModule::testing() { 
  getModule().attr("testing")(1);
}

py::object ExecutionModule::init(const string &task) {
  return getModule().attr("init")(task);
}

py::object ExecutionModule::selectTrajectory(py::object points, py::object currRobotJointVals) {
  return getModule().attr("select_trajectory")(points, currRobotJointVals);
}

} // namespace lfd
