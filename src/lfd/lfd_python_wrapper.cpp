#include "lfd_python_wrapper.h"

namespace lfd {

ExecutionModule::ExecutionModule() : PyModule("python/lfd/execution.py") {
}

py::object ExecutionModule::init(const string &task, py::object tableBounds) {
  return getModule().attr("init")(task, tableBounds);
}

py::object ExecutionModule::selectTrajectory(py::object points, py::object currRobotJointVals, py::object currStep) {
  return getModule().attr("select_trajectory")(points, currRobotJointVals, currStep);
}


CurvePerturbation::CurvePerturbation() : PyModule("python/lfd/curve_perturbation.py") {
}

py::object CurvePerturbation::perturbCurve(py::object curve, double s) {
  return getModule().attr("perturb_curve")(curve, s);
}

vector<btVector3> CurvePerturbation::perturbCurve(const vector<btVector3> &pts, double s) {
  return NPnx3ToPointVec(perturbCurve(pointVecToNP(pts), s));
}

DemoLoadingModule::DemoLoadingModule() : PyModule("python/lfd/demo_loading.py") {
}

py::object DemoLoadingModule::loadDemos(const string &task, const string &demo_list_file) {
  return getModule().attr("load_demos")(task, demo_list_file);
}

RopeInitModule::RopeInitModule() : PyModule("bulletsim/bulletsim_python/src/tracking_initialization/rope_initialization.py") {
}

py::object RopeInitModule::find_path_through_point_cloud(py::object xyzs, float seg_len) {
  py::list args;
  args.append(xyzs);

  py::dict kwargs;
  kwargs["seg_len"] = seg_len;
  kwargs["plotting"] = false;
 
  return getModule().attr("find_path_through_point_cloud")(*py::tuple(args), **kwargs);
}

} // namespace lfd
