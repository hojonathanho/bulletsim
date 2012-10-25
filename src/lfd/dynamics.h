#ifndef _LFD_DYNAMICS_H_
#define _LFD_DYNAMICS_H_

#include <Eigen/Core>
#include "lfd_rope_common.h"

namespace lfd {

struct RopeRobotSystem {
  typedef boost::shared_ptr<RopeRobotSystem> Ptr;
  ~RopeRobotSystem();

  Environment::Ptr env;
  RaveRobotObject::Ptr robot;
  RaveRobotObject::Manipulator::Ptr manip;
  CapsuleRope::Ptr rope;

  // for debugging
  Scene *scene;
  void enableDrawing(Scene *);
  void enableDrawing();
  void draw();
  void disableDrawing();

  Ptr fork() const;

  static Ptr InitFrom(const LFDRopeScene &);

  void assertIntegrity();
};

class SimpleTrajOptimizer {
public:
  typedef Eigen::VectorXd State; // x1 y1 z1, x2 y2 z2 ...
  typedef Eigen::Matrix<double, 7, 1> Control; // 7 manip dofs
  struct NominalTraj {
    vector<State> x;
    vector<Control> u;
  };

  void setInitSys(RopeRobotSystem::Ptr sys) { initSys = sys; }
  void setInitTraj(NominalTraj t) { initTraj = t; }
  void setDesiredEndState(const State &s) { desiredEndState = s; }
  void setHorizon(int h) { horizon = h; }
  void setDebuggingScene(Scene *s) { dbgScene = s; }

  SimpleTrajOptimizer();

  void run();

  NominalTraj execNominalTraj(RopeRobotSystem::Ptr initSys, const NominalTraj &traj);
  static NominalTraj toNominalTraj(const vector<RopeState> &ropeStates, const vector<vector<double> > &manipDofs);

  class NominalTrajBuilder {
  public:
    void append(RopeRobotSystem::Ptr sys);
    void append(const LFDRopeScene &s) { append(RopeRobotSystem::InitFrom(s)); }
    void clear();
    NominalTraj build();
  private:
    vector<RopeState> ropeStates;
    vector<vector<double> > manipDofs;
  };

protected:
  NominalTraj initTraj;
  RopeRobotSystem::Ptr initSys;
  State desiredEndState;
  Scene *dbgScene;
  int horizon;
};

} // namespace lfd


#endif // _LFD_DYNAMICS_H_
