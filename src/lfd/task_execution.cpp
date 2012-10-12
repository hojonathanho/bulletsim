#include "task_execution.h"
#include "utils/logging.h"

#include "simulation/bullet_io.h"
#include "utils_python.h"
#include "simulation/util.h"
#include "rope_scenes.h"

namespace lfd {

// Trajectory conversion utility functions
static inline py::object dictExtract(py::dict d, const string &k) {
  return d.has_key(k) ? d[k] : py::object();
}

static inline vector<double> pyListToVec(py::object l) {
  int len = py::len(l);
  vector<double> v(len);
  for (int i = 0; i < len; ++i) {
    v[i] = py::extract<double>(l[i]);
  }
  return v;
}

static Trajectory::GripperTraj toGripperTraj(int steps, py::object o) {
  Trajectory::GripperTraj t;
  if (o == py::object()) return t;
  t.resize(steps);
  assert(py::len(o) == steps);
  for (int i = 0; i < steps; ++i) {
    t[i] = py::extract<double>(o[i]);
  }
  return t;
}

static Trajectory::ArmTraj toArmTraj(int steps, py::object o) {
  Trajectory::ArmTraj t;
  if (o == py::object()) return t;
  assert(py::len(o) == steps);
  for (int i = 0; i < steps; ++i) {
    t.push_back(pyListToVec(o[i]));
  }
  return t;
}

static Trajectory::GrabTraj toGrabTraj(int steps, py::object o) {
  Trajectory::GrabTraj t;
  if (o == py::object()) return t;
  t.resize(steps);
  assert(py::len(o) == steps);
  for (int i = 0; i < steps; ++i) {
    t[i] = py::extract<bool>(o[i]);
  }
  return t;
}

static RopeState toRopeState(py::object o) {
  int len = py::len(o);
  assert(len % 3 == 0);
  RopeState rs;
  for (int i = 0; i < len/3; ++i) {
    btScalar x, y, z;
    x = py::extract<btScalar>(o[3*i]);
    y = py::extract<btScalar>(o[3*i+1]);
    z = py::extract<btScalar>(o[3*i+2]);
    assert(std::isfinite(x) && std::isfinite(y) && std::isfinite(z));
    rs.push_back(btVector3(x, y, z));
  }
  return rs;
}

// TODO: this is rope-specific
static Trajectory::TrackedStates toTrackedStates(int steps, py::object o, RaveRobotObject::Ptr pr2) {
  Trajectory::TrackedStates t;
  if (o == py::object()) return t;
  t.resize(steps);
  assert(py::len(o) == steps);
  for (int i = 0; i < steps; ++i) {
    assert(i == 0 || py::len(o[i]) == py::len(o[i-1]));
    t[i] = ropeState_real2sim(toRopeState(o[i]), pr2);
  }
  return t;
}

static PlotLines::Ptr drawRopeState(const RopeState &rs, const Eigen::Vector3f &color, float alpha, Environment::Ptr env) {
  vector<btVector3> points0, points1;
  for (int z = 0; z < rs.size() - 1; ++z) {
    points0.push_back(rs[z]);
    points1.push_back(rs[z+1]);
  }
  return util::drawLines(points0, points1, color, alpha, env);
}

static vector<btVector3> cloudToPoints(py::object cloud, RaveRobotObject::Ptr pr2) {
  btTransform base_footprint_trans = pr2->getLinkTransform(
    pr2->robot->GetLink("base_footprint")
  );
  vector<btVector3> pts;
  int len = py::len(cloud);
  for (int i = 0; i < len; ++i) {
    btScalar x = py::extract<btScalar>(cloud[i][0]);
    btScalar y = py::extract<btScalar>(cloud[i][1]);
    btScalar z = py::extract<btScalar>(cloud[i][2]);
    pts.push_back(base_footprint_trans * btVector3(x, y, z) * METERS);
  }
  return pts;
}

// Read data from the python trajectory representation
Trajectory::Trajectory(py::object pytraj, RaveRobotObject::Ptr pr2) {
  py::dict traj = py::extract<py::dict>(pytraj);

  steps = py::extract<int>(traj["steps"]);

  // array of numbers (gripper dofs)
  lGripperTraj = toGripperTraj(steps, dictExtract(traj, "l_gripper"));
  rGripperTraj = toGripperTraj(steps, dictExtract(traj, "r_gripper"));

  // array of arrays of numbers (dofs of the whole arm)
  lArmTraj = toArmTraj(steps, dictExtract(traj, "l_arm"));
  rArmTraj = toArmTraj(steps, dictExtract(traj, "r_arm"));

  // array of booleans
  lGrabTraj = toGrabTraj(steps, dictExtract(traj, "l_grab"));
  rGrabTraj = toGrabTraj(steps, dictExtract(traj, "r_grab"));

  // array of array of vectors (warped tracked states)
  trackedStates = toTrackedStates(steps, dictExtract(traj, "tracked_states"), pr2);
  origTrackedStates = toTrackedStates(steps, dictExtract(traj, "orig_tracked_states"), pr2);

  assert(checkIntegrity());
}

bool Trajectory::checkIntegrity() {
  if (lGripperTraj.size() != 0 && lGripperTraj.size() != steps) return false;
  if (rGripperTraj.size() != 0 && rGripperTraj.size() != steps) return false;
  if (lArmTraj.size() != 0 && lArmTraj.size() != steps) return false;
  if (rArmTraj.size() != 0 && rArmTraj.size() != steps) return false;
  if (rGrabTraj.size() != 0 && rGrabTraj.size() != steps) return false;
  if (trackedStates.size() != 0 && trackedStates.size() != steps) return false;
  if (origTrackedStates.size() != 0 && origTrackedStates.size() != steps) return false;
  return true;
}


TaskExecuter::TaskExecuter(TableRopeScene &scene_) : scene(scene_), trajExecSlowdown(1.0) {
}

void TaskExecuter::setTrajExecSlowdown(double s) {
  trajExecSlowdown = s;
}

void TaskExecuter::setTrajStepCallback(TrajStepCallback f) {
  stepCallback = f;
}

void TaskExecuter::setSegCallback(SegCallback f) {
  segCallback = f;
}

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
  vector<btVector3> ropePts = ropeState_sim2real(
    scene.getRope()->getControlPoints(),
    scene.pr2m->pr2
  );
  py::object nparr = NP.attr("empty")(py::make_tuple(ropePts.size(), 3));
  for (int i = 0; i < ropePts.size(); ++i) {
    nparr[i][0] = ropePts[i].x();
    nparr[i][1] = ropePts[i].y();
    nparr[i][2] = ropePts[i].z();
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

    if (segCallback) {
      string seg_name = py::extract<string>(res["trajectory"]["seg_name"]);
      if (segCallback(seg_name)) {
        // refit if the rope was changed
        action_lookAtObject(); // sets 'points' data
        py::object res = pymod.selectTrajectory(
          readData("points"),
          py::list(scene.pr2m->pr2->getDOFValues()),
          py::object(currStep)
        );
      }
      string new_seg_name = py::extract<string>(res["trajectory"]["seg_name"]);
      assert(new_seg_name == seg_name);
    }

    saveData("trajectory", res["trajectory"]);
  }
  ++currStep;
  return transitionFromString(status);
}

static inline double clampGripperAngle(double a) {
  const float MIN_ANGLE = 0.08;
  const float scale = 20./3.;
  return scale*a < MIN_ANGLE ? MIN_ANGLE : scale*a;
}

TaskExecuter::Transition TaskExecuter::action_execTraj() {
  RaveRobotObject::Ptr pr2 = scene.pr2m->pr2;
  Trajectory t(readData("trajectory"), pr2);

  // calculate slowdown
  const int simSteps = (int) ceil(trajExecSlowdown * t.steps);
  vector<int> sim2real(simSteps, -1);
  for (int i = 0; i < t.steps; ++i) {
    sim2real[(int)(trajExecSlowdown * i)] = i;
  }

  RopeStatePlot::Ptr warped_tracked_plot(new RopeStatePlot());
  RopeStatePlot::Ptr orig_tracked_plot(new RopeStatePlot());
  PlotPoints::Ptr demo_cloud_plot(new PlotPoints());
  util::getGlobalEnv()->add(warped_tracked_plot);
  util::getGlobalEnv()->add(orig_tracked_plot);
  util::getGlobalEnv()->add(demo_cloud_plot);

  scene.setGrabBodies(scene.getRope()->getChildren());
  for (int k = 0; k < simSteps; ++k) {
    int i = sim2real[k];
    if (i != -1) {

      // rope state plotting
      demo_cloud_plot->setPoints(cloudToPoints(readData("trajectory")["demo"]["cloud_xyz_ds"], pr2));
      if (!t.trackedStates.empty()) {
        warped_tracked_plot->setRope(t.trackedStates[i], Eigen::Vector3f(1, 0, 0), 0.5);
      }
      if (!t.origTrackedStates.empty()) {
        orig_tracked_plot->setRope(t.origTrackedStates[i], Eigen::Vector3f(0, 0, 1), 0.4);
      }

      // left
      if (!t.lArmTraj.empty()) {
        scene.pr2m->pr2Left->setDOFValues(t.lArmTraj[i]);
      }
      if (!t.lGripperTraj.empty()) {
        scene.pr2m->pr2Left->setGripperAngle(clampGripperAngle(t.lGripperTraj[i]));
      }
      if (!t.lGrabTraj.empty()) {
        if (t.lGrabTraj[i] && (i == 0 || !t.lGrabTraj[i-1])) {
          scene.m_lMonitor->grab();
        } else if (!t.lGrabTraj[i] && (i == 0 || t.lGrabTraj[i-1])) {
          scene.m_lMonitor->release();
        }
      }

      // right
      if (!t.rArmTraj.empty()) {
        scene.pr2m->pr2Right->setDOFValues(t.rArmTraj[i]);
      }
      if (!t.rGripperTraj.empty()) {
        scene.pr2m->pr2Right->setGripperAngle(clampGripperAngle(t.rGripperTraj[i]));
      }
      if (!t.rGrabTraj.empty()) {
        if (t.rGrabTraj[i] && (i == 0 || !t.rGrabTraj[i-1])) {
          scene.m_rMonitor->grab();
        } else if (!t.rGrabTraj[i] && (i == 0 || t.rGrabTraj[i-1])) {
          scene.m_rMonitor->release();
        }
      }

      if (stepCallback) {
        // TODO: make stepCallback take the Trajectory, not the py::dict
        stepCallback(py::extract<py::dict>(readData("trajectory")), i);
      }
    }

    scene.step(DT);
  }

  scene.m_lMonitor->release();
  scene.m_rMonitor->release();

  warped_tracked_plot->clear();
  orig_tracked_plot->clear();
  demo_cloud_plot->clear();
  util::getGlobalEnv()->remove(warped_tracked_plot);
  util::getGlobalEnv()->remove(orig_tracked_plot);
  util::getGlobalEnv()->remove(demo_cloud_plot);

  return TR_SUCCESS;
}

} // namespace lfd
