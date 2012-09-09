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
  LOG_INFO("state machine: executing state " << s);
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

TaskExecuter::State TaskExecuter::run(const string &taskName, State start) {
  py::list tableBounds;
  tableBounds.append(scene.tableCornersWorld[0].x() / GeneralConfig::scale);
  tableBounds.append(scene.tableCornersWorld[2].x() / GeneralConfig::scale);
  tableBounds.append(scene.tableCornersWorld[0].y() / GeneralConfig::scale);
  tableBounds.append(scene.tableCornersWorld[2].y() / GeneralConfig::scale);
  tableBounds.append(scene.tableCornersWorld[0].z() / GeneralConfig::scale);
  tableBounds.append(scene.tableCornersWorld[2].z() / GeneralConfig::scale);
  pymod.init(taskName, tableBounds);

  State s = start;
  currStep = 0;
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
    btVector3 transformed_pt = base_footprint_trans * ropePts[i] / GeneralConfig::scale;
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
    py::list(scene.pr2m->pr2->getDOFValues()),
    py::object(currStep)
  );
  string status = py::extract<string>(res["status"]);
  if (status == "not_done") {
    saveData("trajectory", res["trajectory"]);
  }
  ++currStep;
  return transitionFromString(status);
}

static inline vector<double> pyListToVec(py::object l) {
  vector<double> v;
  for (int i = 0; i < py::len(l); ++i) {
    v.push_back(py::extract<double>(l[i]));
  }
  return v;
}

static inline py::object dictExtract(py::dict d, const string &k) {
  return d.has_key(k) ? d[k] : py::object();
}

static inline double clampGripperAngle(double a) {
  const float MIN_ANGLE = 0.1;
  const float scale = 20./3.;
  return scale*a < MIN_ANGLE ? MIN_ANGLE : scale*a;
}

TaskExecuter::Transition TaskExecuter::action_execTraj() {
  py::dict traj = py::extract<py::dict>(readData("trajectory"));
  // array of numbers (gripper dofs)
  py::object lGripperTraj = dictExtract(traj, "l_gripper");
  py::object rGripperTraj = dictExtract(traj, "r_gripper");
  // array of arrays of numbers (dofs of the whole arm)
  py::object lArmTraj = dictExtract(traj, "l_arm");
  py::object rArmTraj = dictExtract(traj, "r_arm");
  // array of booleans
  py::object lGrabTraj = dictExtract(traj, "l_grab");
  py::object rGrabTraj = dictExtract(traj, "r_grab");

  const int steps = py::extract<int>(traj["steps"]);

  assert(lGripperTraj == py::object() || steps == py::len(lGripperTraj));
  assert(rGripperTraj == py::object() || steps == py::len(rGripperTraj));
  assert(lArmTraj == py::object() || steps == py::len(lArmTraj));
  assert(rArmTraj == py::object() || steps == py::len(rArmTraj));
  assert(lGrabTraj == py::object() || steps == py::len(lGrabTraj));
  assert(rGrabTraj == py::object() || steps == py::len(rGrabTraj));

  RaveRobotObject::Ptr pr2 = scene.pr2m->pr2;
  scene.setGrabBodies(scene.getRope()->getChildren());
  for (int i = 0; i < steps; ++i) {
    if (lArmTraj != py::object()) scene.pr2m->pr2Left->setDOFValues(pyListToVec(lArmTraj[i]));
    if (lGripperTraj != py::object()) scene.pr2m->pr2Left->setGripperAngle(clampGripperAngle(py::extract<double>(lGripperTraj[i])));
    if (lGrabTraj != py::object()) {
      if (py::extract<bool>(lGrabTraj[i]) && (i == 0 || !py::extract<bool>(lGrabTraj[i-1]))) {
        scene.m_lMonitor->grab();
      } else if (!py::extract<bool>(lGrabTraj[i]) && (i == 0 || py::extract<bool>(lGrabTraj[i-1]))) {
        scene.m_lMonitor->release();
      }
    }

    if (rArmTraj != py::object()) scene.pr2m->pr2Right->setDOFValues(pyListToVec(rArmTraj[i]));
    if (rGripperTraj != py::object()) scene.pr2m->pr2Right->setGripperAngle(clampGripperAngle(py::extract<double>(rGripperTraj[i])));
    if (rGrabTraj != py::object()) {
      if (py::extract<bool>(rGrabTraj[i]) && (i == 0 || !py::extract<bool>(rGrabTraj[i-1]))) {
        scene.m_rMonitor->grab();
      } else if (!py::extract<bool>(rGrabTraj[i]) && (i == 0 || py::extract<bool>(rGrabTraj[i-1]))) {
        scene.m_rMonitor->release();
      }
    }

    scene.step(DT);
  }
  scene.m_lMonitor->release();
  scene.m_lMonitor->release();
}

} // namespace lfd
