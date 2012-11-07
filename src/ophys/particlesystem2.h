#ifndef __OPHYS_PARTICLE_SYSTEM_2_H__
#define __OPHYS_PARTICLE_SYSTEM_2_H__

#include "sqp/sqp.h"
#include "ophys_config.h"

#include <Eigen/Dense>
using Eigen::Matrix;

#include <boost/multi_array.hpp>
#include <boost/format.hpp>
using boost::multi_array;
#include <vector>
using std::vector;

#include "simulation/plotting.h"
class Scene;


namespace ophys {


struct ConvexConstraintWrapper : public Constraint {
  typedef boost::shared_ptr<ConvexConstraintWrapper> Ptr;

  ConvexConstraintPtr m_cnt;
  ConvexConstraintWrapper(ConvexConstraintPtr cnt) : m_cnt(cnt) { }
  ConvexConstraintPtr convexify(GRBModel* model) { return m_cnt; }

  static void AddToOpt(Optimizer &opt, ConvexConstraintPtr cnt) {
    ConvexConstraintWrapper::Ptr wrapper(new ConvexConstraintWrapper(cnt));
    opt.addConstraint(wrapper);
  }
};


class ParticleSystemOptimizer2 : public Optimizer {
public:
  typedef boost::shared_ptr<ParticleSystemOptimizer2> Ptr;

  // constant system parameters
  const int m_numParticles, m_horizon;
  const string m_varPrefix;

  // variable layout:
  // we want to represent the states of m_numParticles over m_horizon time intervals
  // each row is the (position, velocity, accel) of one particle
  static const int PARTICLE_STATE_DIM = 9;
  typedef Matrix<double, Dynamic, PARTICLE_STATE_DIM> SysState;
  typedef vector<SysState> SysStatesOverTime;
  SysStatesOverTime m_sys;
  SysStatesOverTime m_sys_backup;

  // Gurobi variables
  // indices to VarSysState: m_varsys[time][particle_idx][component]
  typedef multi_array<GRBVar, 3> VarSysState;
  VarSysState m_varsys;


  explicit ParticleSystemOptimizer2(int numParticles, int horizon, const string &varPrefix="particlesystem");

  void initializeVariables();
  void updateValues();
  void storeValues();
  void rollbackValues();
  void preOptimize();
  void postOptimize();

  void initialize(const SysStatesOverTime &init);
  void initializeFromSingleState(const SysState &s0);

  friend class ParticleSystem2;

  struct InitCondCvxConstraint : public ConvexConstraint {
    typedef boost::shared_ptr<InitCondCvxConstraint> Ptr;
    // s0 only needs x,v,a filled in
    InitCondCvxConstraint(ParticleSystemOptimizer2 &opt, const ParticleSystemOptimizer2::SysState &s0);
  };


  struct InitCondConstraint : public Constraint {
    typedef boost::shared_ptr<InitCondConstraint> Ptr;
    ParticleSystemOptimizer2 &m_opt;
    ParticleSystemOptimizer2::SysState m_s0;
    InitCondConstraint(ParticleSystemOptimizer2 &opt, const ParticleSystemOptimizer2::SysState &s0) : m_opt(opt), m_s0(s0) { }
    ConvexConstraintPtr convexify(GRBModel* );
  };


  struct GroundConstraint : public Constraint {
    typedef boost::shared_ptr<GroundConstraint> Ptr;
    ParticleSystemOptimizer2 &m_opt;
    double m_groundZ;
    GroundConstraint(ParticleSystemOptimizer2 &opt, double groundZ) : m_opt(opt), m_groundZ(groundZ) { }
    ConvexConstraintPtr convexify(GRBModel* );
  };

  struct PhysicsStepCost : public Cost {
    typedef boost::shared_ptr<PhysicsStepCost> Ptr;

    ParticleSystemOptimizer2 &m_opt;
    PhysicsStepCost(ParticleSystemOptimizer2 &opt) : m_opt(opt) { }

    string getName() { return "physics_step"; }

    double evaluate();
    ConvexObjectivePtr convexify(GRBModel* model);
  };

private:

  Eigen::Block<SysState, 1, 3> ptPos(int t, int i) { return m_sys[t].block<1,3>(i,0); }
  const Eigen::Block<const SysState, 1, 3> ptPos(int t, int i) const { return m_sys[t].block<1,3>(i,0); }
  Eigen::Block<SysState, 1, 3> ptVel(int t, int i) { return m_sys[t].block<1,3>(i,3); }
  const Eigen::Block<const SysState, 1, 3> ptVel(int t, int i) const { return m_sys[t].block<1,3>(i,3); }
  Eigen::Block<SysState, 1, 3> ptAcc(int t, int i) { return m_sys[t].block<1,3>(i,6); }
  const Eigen::Block<const SysState, 1, 3> ptAcc(int t, int i) const { return m_sys[t].block<1,3>(i,6); }

  typedef boost::multi_array_types::index_range range;
  typedef VarSysState::array_view<1>::type VarVec3View;
  VarVec3View varPtPos(int t, int i) { return m_varsys[ boost::indices[t][i][range(0,3)] ]; }
  VarVec3View varPtVel(int t, int i) { return m_varsys[ boost::indices[t][i][range(3,6)] ]; }
  VarVec3View varPtAcc(int t, int i) { return m_varsys[ boost::indices[t][i][range(6,9)] ]; }
};


struct ParticleSysTrustRegion : public TrustRegion {
  typedef boost::shared_ptr<ParticleSysTrustRegion> Ptr;

  ParticleSystemOptimizer2 &m_opt;
  multi_array<double, 3> m_radii;

  ParticleSysTrustRegion(ParticleSystemOptimizer2 &opt);

  void adjustTrustRegion(double ratio);
  ConvexConstraintPtr convexify(GRBModel* model);
};

class ParticleSystem2 {
public:
  //ParticleSystemOptimizer2::Ptr m_opt;
  ParticleSystemOptimizer2::SysState m_currState;
  ParticleSystemOptimizer2::SysStatesOverTime m_states;

  PlotSpheres::Ptr m_plotSpheres;
  Scene *m_scene;

  ParticleSystem2(const ParticleSystemOptimizer2::SysState &initState);
  void step(double dt, int numSteps=1);
  void step();
  void setupOpt(ParticleSystemOptimizer2 &opt);

  void attachToScene(Scene *);
  void draw();
  void play();

protected:
  ParticleSysTrustRegion::Ptr m_trustRegion;
  ParticleSystemOptimizer2::InitCondConstraint::Ptr m_initCondCnt;
  ParticleSystemOptimizer2::GroundConstraint::Ptr m_groundCnt;
  ParticleSystemOptimizer2::PhysicsStepCost::Ptr m_physicsStepCost;
};


} // namespace ophys


#endif // __OPHYS_PARTICLE_SYSTEM_2_H__
