#ifndef _LFD_PYTHON_WRAPPER_H_
#define _LFD_PYTHON_WRAPPER_H_

#include "utils_python.h"

namespace lfd {

class ExecutionModule : public PyModule {
public:
  ExecutionModule();

  py::object init(const string &task, py::object tableBounds);
  py::object selectTrajectory(py::object points, py::object currRobotJointVals, py::object currStep);
};

class CurvePerturbation : public PyModule {
public:
  CurvePerturbation();
  py::object perturbCurve(py::object curve, double s=0.01);
  vector<btVector3> perturbCurve(const vector<btVector3> &pts, double s=0.01);
};

class DemoLoadingModule : public PyModule {
public:
  DemoLoadingModule();
  py::object loadDemos(const string &task, const string &demo_list_file="knot_demos.yaml");
};

struct RopeInitModule : public PyModule {
  RopeInitModule();
  py::object find_path_through_point_cloud(py::object xyzs, float seg_len=0.02);
};

} // namespace lfd

#endif // _LFD_PYTHON_WRAPPER_H_
