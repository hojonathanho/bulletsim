#ifndef __OPHYS_PARTICLE_SYSTEM_H__
#define __OPHYS_PARTICLE_SYSTEM_H__

#include <Eigen/Dense>
#include <Eigen/StdVector>
using namespace Eigen;
#include <boost/array.hpp>
#include <boost/multi_array.hpp>
#include <vector>
using std::vector;

#include "sqp/sqp.h"
#include "ophys_config.h"

#include "simulation/plotting.h"

class Scene;

namespace ophys {


struct ParticleState {
  boost::array<GRBVar, 3> var_x, var_v, var_a;
  Vector3d x, v, a; // position, velocity, acceleration
  Vector3d x_backup, v_backup, a_backup;
  double invm; // 1/mass
  int idx; // index in the system

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef vector<ParticleState, Eigen::aligned_allocator<ParticleState> > ParticleSystemState;
typedef vector<ParticleSystemState, Eigen::aligned_allocator<ParticleSystemState> > ParticleSystemStateVec;

class ParticleSystemOptimizer : public Optimizer {
public:
  typedef boost::shared_ptr<ParticleSystemOptimizer> Ptr;

  ParticleSystemStateVec m_sys; // system state over time
  const int m_numParticles, m_horizon;
  const string m_varPrefix;

  ParticleSystemOptimizer(int numParticles, int horizon, const string &varPrefix="particlesystem");
  void updateValues();
  void storeValues();

  void rollbackValues();
  void initialize(const ParticleSystemStateVec &init);
  void initializeFromSingleState(const ParticleSystemState &s0);
  void postOptimize();
};

struct ParticleSystemTrustRegion : public TrustRegion {
  // boxes around all variables x, v, a
  typedef boost::shared_ptr<ParticleSystemTrustRegion> Ptr;

  ParticleSystemOptimizer &m_opt;
  boost::multi_array<double, 3> m_x_radii, m_v_radii, m_a_radii;

  bool m_infinite;
  void setInfinite(bool);

  ParticleSystemTrustRegion(ParticleSystemOptimizer &opt);
  void adjustTrustRegion(double ratio);
  ConvexConstraintPtr convexify(GRBModel* model);
};

struct InitialConditionConstraints : public ConvexConstraint {
  typedef boost::shared_ptr<InitialConditionConstraints> Ptr;

  // s0 only needs x,v,a filled in
  InitialConditionConstraints(ParticleSystemOptimizer &o, const ParticleSystemState &s0);
};

struct NoExternalForcesConstraint : public ConvexConstraint {
  typedef boost::shared_ptr<NoExternalForcesConstraint> Ptr;
  NoExternalForcesConstraint(ParticleSystemOptimizer &o);
};

struct GroundConstraint : public ConvexConstraint {
  typedef boost::shared_ptr<GroundConstraint> Ptr;
  GroundConstraint(ParticleSystemOptimizer &o, double groundZ);
};

struct ConvexConstraintWrapper : public Constraint {
  typedef boost::shared_ptr<ConvexConstraintWrapper> Ptr;

  ConvexConstraintPtr m_cnt;
  ConvexConstraintWrapper(ConvexConstraintPtr cnt) : m_cnt(cnt) { }
  ConvexConstraintPtr convexify(GRBModel* model) { return m_cnt; }

  static void AddToOpt(Optimizer &opt, ConvexConstraintPtr cnt);
};

struct PhysicsStepConstraint : public ConvexConstraint {
  typedef boost::shared_ptr<PhysicsStepConstraint> Ptr;
  PhysicsStepConstraint(ParticleSystemOptimizer &o);
};

struct PhysicsStepCost : public Cost {
  typedef boost::shared_ptr<PhysicsStepCost> Ptr;

  ParticleSystemOptimizer &m_opt;
  PhysicsStepCost(ParticleSystemOptimizer &opt) : m_opt(opt) { }

  string getName() { return "physics_step"; }

  double evaluate();
  ConvexObjectivePtr convexify(GRBModel* model);
};

struct PointDistanceCost : public Cost {
  typedef boost::shared_ptr<PointDistanceCost> Ptr;

  ParticleSystemOptimizer &m_opt;
  int m_p, m_q;
  double m_d;

  PointDistanceCost(ParticleSystemOptimizer &opt, int p, int q, double d);

  string getName();
  double evaluate();
  double evaluate(const MatrixX3d &pointsOverTime);
  ConvexObjectivePtr convexify(GRBModel *model);

private:
  MatrixX3d getCurrPointMatrix() const;
};

struct PointAnchorCost : public Cost {
  typedef boost::shared_ptr<PointAnchorCost> Ptr;

  ParticleSystemOptimizer &m_opt;
  int m_p;
  Vector3d m_anchorpt;

  PointAnchorCost(ParticleSystemOptimizer &opt, int p, const Vector3d &anchorpt);
  string getName();
  double evaluate();
  double evaluate(const MatrixX3d &pointOverTime);
  ConvexObjectivePtr convexify(GRBModel *model);

private:
  MatrixX3d getCurrPointMatrix() const;
};

struct PointDistanceConstraint : public Constraint {
	ConvexConstraintPtr convexify(GRBModel* model);
};


class ParticleSystem {
public:
  ParticleSystemState m_currState;
  ParticleSystemStateVec m_currStates;
  PlotSpheres::Ptr m_plotSpheres;
  Scene *m_scene;

  ParticleSystem(const ParticleSystemState &initState);
  void step(double dt, int numSteps=1);
  void step();

  void attachToScene(Scene *);
  void draw();
  void play();

  typedef boost::function<void(ParticleSystemOptimizer *)> PreOptCallback;
  void setPreOptCallback0(PreOptCallback cb);
  void setPreOptCallback(PreOptCallback cb);

protected:
  virtual void setupOpt0(ParticleSystemOptimizer &opt);
  virtual void setupOpt(ParticleSystemOptimizer &opt);
  PreOptCallback m_preOptCallback0;
  PreOptCallback m_preOptCallback;

  ParticleSystemTrustRegion::Ptr m_trustRegion;
  PhysicsStepCost::Ptr m_physicsStepCost;
  PhysicsStepConstraint::Ptr m_physicsStepCnt;
  InitialConditionConstraints::Ptr m_initCondCnt;
  NoExternalForcesConstraint::Ptr m_noExtForcesCnt;
  GroundConstraint::Ptr m_groundCnt;
};


class RopeSystem : public ParticleSystem {
public:
  RopeSystem(const ParticleSystemState &initState, double segrlen);
  const double m_segrlen; // segment resting length

protected:
  virtual void setupOpt(ParticleSystemOptimizer &opt);
};


} // namespace ophys

#endif // __OPHYS_PARTICLE_SYSTEM_H__
