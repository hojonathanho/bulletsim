#ifndef _LFD_TASK_EXECUTION_H_
#define _LFD_TASK_EXECUTION_H_

#include <string>
#include <map>
using namespace std;

#include "lfd_python_wrapper.h"
#include "lfd_rope_common.h"

class TableRopeScene;

namespace lfd {

struct Trajectory {
  int steps;

  typedef vector<double> GripperTraj;
  GripperTraj lGripperTraj, rGripperTraj;

  typedef vector<vector<double> > ArmTraj;
  ArmTraj lArmTraj, rArmTraj;

  typedef vector<bool> GrabTraj;
  GrabTraj lGrabTraj, rGrabTraj;

  typedef vector<RopeState> TrackedStates;
  TrackedStates trackedStates, origTrackedStates;

  Trajectory() { }
  Trajectory(py::object pytraj, RaveRobotObject::Ptr pr2);

  bool checkIntegrity();
};

// (hard-wired) task execution state machine
class TaskExecuter {
public:
  enum State {
    ST_FAILURE = 0,
    ST_SUCCESS,
    ST_LOOK_AT_OBJ,
    ST_SELECT_TRAJ,
    ST_EXEC_TRAJ,
  };
  enum Transition {
    TR_FAILURE = 0,
    TR_SUCCESS,
    TR_NOT_DONE
  };

  TaskExecuter(TableRopeScene &scene_);

  void setTrajExecSlowdown(double s);
  void setExecOneStep(bool b);

  typedef boost::function<void(py::dict, int)> TrajStepCallback;
  void setTrajStepCallback(TrajStepCallback f);
  typedef boost::function<bool(const string&)> SegCallback;
  void setSegCallback(SegCallback f);

  State run(const string &taskName, State start=ST_LOOK_AT_OBJ);

private:
  static Transition transitionFromString(const string &s); 

  TableRopeScene &scene;
  ExecutionModule pymod; // some actions are implemented in python
  int currStep;
  double trajExecSlowdown;
  bool execOneStep;
  TrajStepCallback stepCallback;
  SegCallback segCallback;

  State nextState(State s, Transition t) const;
  Transition execState(State s);
  bool isTerminalState(State s) const;

  // For the states in the state machine to communicate
  map<string, py::object> storage;
  void saveData(const string &key, py::object data) { storage[key] = data; }
  py::object readData(const string &key) { return storage[key]; }

  // Actions for the states
  Transition action_lookAtObject();
  Transition action_selectTraj();
  Transition action_execTraj();
};

} // namespace lfd

#endif // _LFD_TASK_EXECUTION_H_
