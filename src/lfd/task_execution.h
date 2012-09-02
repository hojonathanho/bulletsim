#ifndef _LFD_TASK_EXECUTION_H_
#define _LFD_TASK_EXECUTION_H_

#include <string>
#include <map>
using namespace std;

#include "rope_scenes.h"
#include "lfd_python_wrapper.h"

namespace lfd {

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

  TaskExecuter(TableRopeScene &scene_) : scene(scene_) { }
  State run(State start);
  State run() { return run(ST_LOOK_AT_OBJ); }

private:
  static Transition transitionFromString(const string &s); 

  TableRopeScene &scene;
  ExecutionModule pymod; // some actions are implemented in python

  State nextState(State s, Transition t) const;
  Transition execState(State s);
  bool isTerminalState(State s) const;

  map<string, py::object> storage;
  void saveData(const string &key, py::object data) { storage[key] = data; }
  py::object readData(const string &key) { return storage[key]; }

  Transition action_lookAtObject();
  Transition action_selectTraj();
  Transition action_execTraj();
};

} // namespace lfd

#endif // _LFD_TASK_EXECUTION_H_
