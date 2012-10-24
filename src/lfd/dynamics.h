#ifndef _LFD_DYNAMICS_H_
#define _LFD_DYNAMICS_H_

#include <Eigen/Core>

#include "lfd_rope_common.h"

namespace lfd {

/********
class AffineSystem;

template<class State, class Control>
class BasicSystem {
public:
  typedef boost::shared_ptr<BasicSystem> Ptr;
  virtual State apply(const State &, const Control &) = 0;
  virtual boost::shared_ptr<AffineSystem> linearize(const State &, const Control &) = 0;
};

class AffineSystem : public BasicSystem<MatrixXd, MatrixXd>, public boost::enable_shared_from_this<AffineSystem> {
public:
  typedef boost::shared_ptr<AffineSystem> Ptr;
  MatrixXd A, B, c;
  AffineSystem(const MatrixXd &A_, const MatrixXd &B_, const MatrixXd &c_) :
    A(A_), B(B_), c(c_) { }
  virtual MatrixXd apply(const MatrixXd &x, const MatrixXd &u) {
    return A*x + B*u + c;
  }
  virtual boost::shared_ptr<AffineSystem> linearize(const MatrixXd &, const MatrixXd &) {
    return shared_from_this();
  }
};

class RobotRopeSystem : public BasicSystem<RopeState, > {
public:
  typedef boost::shared_ptr<RobotRopeSystem> Ptr;
  virtual State apply(const State &, const Control &);
};

typedef StatePath;

class ILQRTracker {
public:
  ILQRTracker(BasicSystem::Ptr system_) : system(system_) { }
  void setHorizon();
  void setInitControl();
  void setDesire();
protected:
  BasicSystem::Ptr system;
};*/


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
  typedef Eigen::VectorXd State; // mx my mz, mrx mry mrz, x1 y1 z1, x2 y2 z2 ...
  typedef Eigen::Matrix<double, 6, 1> Control; // dx dy rz, drx dry drz
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
  static NominalTraj toNominalTraj(const vector<RopeState> &ropeStates, const vector<btTransform> &manipPoses);

  class NominalTrajBuilder {
  public:
    void append(RopeRobotSystem::Ptr sys);
    void append(const LFDRopeScene &s) { append(RopeRobotSystem::InitFrom(s)); }
    void clear();
    NominalTraj build();
  private:
    vector<RopeState> ropeStates;
    vector<btTransform> manipPoses;
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
