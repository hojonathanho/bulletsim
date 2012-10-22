#include "lfd_python_wrapper.h"

namespace lfd {

ExecutionModule::ExecutionModule() : PyModule("/home/joschu/python/lfd/execution.py") {
}

py::object ExecutionModule::init(const string &task, py::object tableBounds) {
  return getModule().attr("init")(task, tableBounds);
}

py::object ExecutionModule::selectTrajectory(py::object points, py::object currRobotJointVals, py::object currStep) {
  return getModule().attr("select_trajectory")(points, currRobotJointVals, currStep);
}


CurvePerturbation::CurvePerturbation() : PyModule("/home/joschu/python/lfd/curve_perturbation.py") {
}

py::object CurvePerturbation::perturbCurve(py::object curve, double s) {
  return getModule().attr("perturb_curve")(curve, s);
}

vector<btVector3> CurvePerturbation::perturbCurve(const vector<btVector3> &pts, double s) {
  return NPnx3ToPointVec(perturbCurve(pointVecToNP(pts), s));
}

} // namespace lfd
