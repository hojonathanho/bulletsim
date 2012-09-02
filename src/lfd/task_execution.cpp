#include "task_execution.h"
#include "utils/logging.h"

#include "simulation/bullet_io.h"
#include "utils_python.h"

namespace lfd {

bool TaskExecuter::isTerminalState(State s) const {
  return s == ST_SUCCESS || s == ST_FAILURE;
}

TaskExecuter::State TaskExecuter::nextState(State s, Transition t) const {
  switch (s) {
  case ST_LOOK_AT_OBJ:
    if (t == TR_SUCCESS) return ST_SELECT_TRAJ;
    if (t == TR_FAILURE) return ST_FAILURE;
    break;
  case ST_SELECT_TRAJ:
    if (t == TR_SUCCESS) return ST_SUCCESS;
    if (t == TR_NOT_DONE) return ST_EXEC_TRAJ;
    if (t == TR_FAILURE) return ST_FAILURE;
    break;
  case ST_EXEC_TRAJ:
    if (t == TR_SUCCESS) return ST_LOOK_AT_OBJ;
    if (t == TR_FAILURE) return ST_LOOK_AT_OBJ; 
    break;
  }
  LOG_ERROR("invalid state/transition " << (int)s << '/' << (int)t);
  return ST_FAILURE;
}

TaskExecuter::Transition TaskExecuter::execState(State s) {
  switch (s) {
  case ST_FAILURE:
  case ST_SUCCESS:
    break;
  case ST_LOOK_AT_OBJ:
    return action_lookAtObject();
  case ST_SELECT_TRAJ:
    return action_selectTraj();
  case ST_EXEC_TRAJ:
    return action_execTraj();
  }
  assert(false);
}

TaskExecuter::State TaskExecuter::run(State start) {
  pymod.init("overhand_knot");
  State s = start;
  do {
    Transition t = execState(s);
    State s2 = nextState(s, t);
    LOG_INFO("state machine: transitioning from " << s << " to " << s2 << " via " << t);
    s = s2;
  } while (!isTerminalState(s));
  return s;
}

TaskExecuter::Transition TaskExecuter::transitionFromString(const string &s) {
  if (s == "failure") return TR_FAILURE;
  if (s == "success") return TR_SUCCESS;
  if (s == "not_done") return TR_NOT_DONE;
  LOG_ERROR("invalid transition " << s);
  assert(false);
}

TaskExecuter::Transition TaskExecuter::action_lookAtObject() {
  // look at the rope
  // return a numpy n-by-3 numpy array representing the point cloud
  // in the base_footprint frame
  RaveRobotObject::Ptr pr2 = scene.pr2m->pr2;
  btTransform base_footprint_trans = pr2->getLinkTransform(
    pr2->robot->GetLink("base_footprint")
  );
  vector<btVector3> ropePts = scene.getRope()->getControlPoints();
  py::object nparr = NP.attr("empty")(py::make_tuple(ropePts.size(), 3));
  for (int i = 0; i < ropePts.size(); ++i) {
    btVector3 transformed_pt = base_footprint_trans * ropePts[i];
    nparr[i][0] = transformed_pt.x();
    nparr[i][1] = transformed_pt.y();
    nparr[i][2] = transformed_pt.z();
  }
  saveData("points", nparr);
  return TR_SUCCESS;
}

TaskExecuter::Transition TaskExecuter::action_selectTraj() {
  py::object res = pymod.selectTrajectory(
    readData("points"),
    py::list(scene.pr2m->pr2->getDOFValues())
  );
  string status = py::extract<string>(res["status"]);
  if (status == "not_done") {
    saveData("trajectory", res["trajectory"]);
  }
  return transitionFromString(status);
}

TaskExecuter::Transition TaskExecuter::action_execTraj() {
}

} // namespace lfd
