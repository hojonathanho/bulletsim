#ifndef _LFD_OPTIMIZATION_SIMPLE_H_
#define _LFD_OPTIMIZATION_SIMPLE_H_

#include <Eigen/Core>
#include "lfd_rope_common.h"
#include "dynamics.h"

namespace lfd {

class SimpleTrajOptimizerImpl;
class SimpleTrajOptimizer {
public:
  typedef Eigen::VectorXd State; // x1 y1 z1, x2 y2 z2 ...
  typedef Eigen::Matrix<double, 7, 1> Control; // 7 manip dofs
  struct NominalTraj {
    vector<State> x; vector<Control> u;
  };

  void setInitSys(RopeRobotSystem::Ptr sys);
  void setInitTraj(NominalTraj t);
  void setDesiredEndState(const State &s);
  void setHorizon(int h);
  void setDebuggingScene(Scene *s);

  void debugJacobianCalc(bool);

  SimpleTrajOptimizer(int numCtlPts);

  void run();
  NominalTraj oneStep();

  NominalTraj execNominalTraj(RopeRobotSystem::Ptr initSys, const NominalTraj &traj);
  void displayNominalTraj(const NominalTraj &);
  static State toState(const RopeState &rs);
  static NominalTraj toNominalTraj(const vector<RopeState> &ropeStates, const vector<vector<double> > &manipDofs);

  class NominalTrajBuilder {
  public:
    void append(const RopeState &ropeState, const vector<double> &manipDof);
    void append(RopeRobotSystem::Ptr sys);
    void append(const LFDRopeScene &s) { append(RopeRobotSystem::InitFrom(s)); }
    void clear();
    void downsample(int x);
    NominalTraj build();
    NominalTraj buildFromSlice(int start, int end);
  private:
    vector<RopeState> ropeStates;
    vector<vector<double> > manipDofs;
  };

protected:
  friend class SimpleTrajOptimizerImpl;
  boost::shared_ptr<SimpleTrajOptimizerImpl> impl;
};

} // namespace lfd

#endif // _LFD_OPTIMIZATION_SIMPLE_H_
