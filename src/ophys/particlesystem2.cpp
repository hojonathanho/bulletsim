#include "particlesystem2.h"

#include "utils/logging.h"
#include "ophys_common.h"

#include "simulation/simplescene.h"

namespace ophys {

const int ParticleSystemOptimizer2::PARTICLE_STATE_DIM;

ParticleSystemOptimizer2::ParticleSystemOptimizer2(int numParticles, int horizon, const string &varPrefix)
  : m_numParticles(numParticles),
    m_horizon(horizon),
    m_varPrefix(varPrefix),
    m_sys(horizon, SysState::Zero(numParticles, PARTICLE_STATE_DIM)),
    m_varsys(boost::extents[horizon][numParticles][PARTICLE_STATE_DIM])
{
}

void ParticleSystemOptimizer2::initializeVariables() {
  boost::format fmt("%s_t%d_i%d_%d");
  for (int t = 0; t < m_horizon; ++t) {
    for (int i = 0; i < m_numParticles; ++i) {
      for (int j = 0; j < PARTICLE_STATE_DIM; ++j) {
        m_varsys[t][i][j] = m_model->addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, (fmt % m_varPrefix % t % i % j).str());
      }
    }
  }
  m_model->update();
}

void ParticleSystemOptimizer2::initialize(const SysStatesOverTime &init) {
  assert(init.size() == m_horizon);
  for (int t = 0; t < m_horizon; ++t) {
    assert(init[t].rows() == m_numParticles);
  }

  m_sys = init;
  initializeVariables();
}

void ParticleSystemOptimizer2::initializeFromSingleState(const SysState &s0) {
  assert(s0.rows() == m_numParticles);
  for (int t = 0; t < m_horizon; ++t) {
    m_sys[t] = s0;
  }
  initializeVariables();
}

void ParticleSystemOptimizer2::updateValues() {
  for (int t = 0; t < m_horizon; ++t) {
    for (int i = 0; i < m_numParticles; ++i) {
      for (int j = 0; j < PARTICLE_STATE_DIM; ++j) {
        m_sys[t](i,j) = m_varsys[t][i][j].get(GRB_DoubleAttr_X);
      }
    }
  }
}

void ParticleSystemOptimizer2::storeValues() {
  m_sys_backup = m_sys;
}

void ParticleSystemOptimizer2::rollbackValues() {
  m_sys = m_sys_backup;
}

void ParticleSystemOptimizer2::preOptimize() {
  LOG_INFO(">>>>> Pre-optimize <<<<<");
  for (int t = 0; t < m_horizon; ++t) {
    LOG_INFO("t = " << t << ":\n" << m_sys[t]);
  }
  LOG_INFO(">>>>> End Pre-optimize <<<<<");
}

void ParticleSystemOptimizer2::postOptimize() {
  LOG_INFO(">>>>> Post-optimize <<<<<");
  for (int t = 0; t < m_horizon; ++t) {
    LOG_INFO("t = " << t << ":\n" << m_sys[t]);
  }
  LOG_INFO(">>>>> End Post-optimize <<<<<");
}

ParticleSysTrustRegion::ParticleSysTrustRegion(ParticleSystemOptimizer2 &opt)
  : m_opt(opt),
    m_radii(boost::extents[opt.m_horizon][opt.m_numParticles][opt.PARTICLE_STATE_DIM])
{
  for (int t = 0; t < m_opt.m_horizon; ++t) {
    for (int i = 0; i < m_opt.m_numParticles; ++i) {
      for (int j = 0; j < m_opt.PARTICLE_STATE_DIM; ++j) {
        m_radii[t][i][j] = OPhysConfig::trustRadius * METERS;
      }
    }
  }
}

void ParticleSysTrustRegion::adjustTrustRegion(double ratio) {
  m_shrinkage *= ratio;
  for (int t = 0; t < m_opt.m_horizon; ++t) {
    for (int i = 0; i < m_opt.m_numParticles; ++i) {
      for (int j = 0; j < m_opt.PARTICLE_STATE_DIM; ++j) {
        m_radii[t][i][j] *= ratio;
      }
    }
  }
}

ConvexConstraintPtr ParticleSysTrustRegion::convexify(GRBModel* model) {
  for (int t = 0; t < m_opt.m_horizon; ++t) {
    for (int i = 0; i < m_opt.m_numParticles; ++i) {
      for (int j = 0; j < m_opt.PARTICLE_STATE_DIM; ++j) {
        m_opt.m_varsys[t][i][j].set(GRB_DoubleAttr_LB, m_opt.m_sys[t](i,j) - m_radii[t][i][j]);
        m_opt.m_varsys[t][i][j].set(GRB_DoubleAttr_UB, m_opt.m_sys[t](i,j) + m_radii[t][i][j]);
      }
    }
  }
  ConvexConstraintPtr cnt(new ConvexConstraint());
  cnt->m_name = "trust_region";
  return cnt;
}


ParticleSystemOptimizer2::InitCondCvxConstraint::InitCondCvxConstraint(
  ParticleSystemOptimizer2 &opt, const ParticleSystemOptimizer2::SysState &s0) {

  m_name = "init_cond_constraint";
  boost::format fmt("%s_init_cond_constraint_i%d_%d");
  for (int i = 0; i < opt.m_numParticles; ++i) {
    for (int j = 0; j < opt.PARTICLE_STATE_DIM; ++j) {
      m_eqcntNames.push_back((fmt % opt.m_varPrefix % i % j).str());
      m_eqexprs.push_back(opt.m_varsys[0][i][j] - s0(i,j));
    }
  }
}

ConvexConstraintPtr ParticleSystemOptimizer2::InitCondConstraint::convexify(GRBModel *) {
  ConvexConstraintPtr out(new ConvexConstraint());
  out->m_name = "init_cond_constraint";
  boost::format fmt("%s_init_cond_constraint_i%d_%d");
  for (int i = 0; i < m_opt.m_numParticles; ++i) {
    for (int j = 0; j < m_opt.PARTICLE_STATE_DIM; ++j) {
      out->m_eqcntNames.push_back((fmt % m_opt.m_varPrefix % i % j).str());
      out->m_eqexprs.push_back(m_opt.m_varsys[0][i][j] - m_s0(i,j));
    }
  }
  return out;
}

ConvexConstraintPtr ParticleSystemOptimizer2::GroundConstraint::convexify(GRBModel* ) {
  ConvexConstraintPtr out(new ConvexConstraint());
  out->m_name = "ground_constraint";
  boost::format fmt("%s_ground_constraint_i%d");
  for (int t = 0; t < m_opt.m_horizon; ++t) {
    for (int i = 0; i < m_opt.m_numParticles; ++i) {
      out->m_cntNames.push_back((fmt % m_opt.m_varPrefix % i).str());
      out->m_exprs.push_back(-m_opt.m_varsys[t][i][2] + m_groundZ);
    }
  }
  return out;
}

double ParticleSystemOptimizer2::PhysicsStepCost::evaluate() {
  const double dt = OPhysConfig::dt;
  double cost = 0;
  for (int t = 0; t < m_opt.m_horizon - 1; ++t) {
    for (int i = 0; i < m_opt.m_numParticles; ++i) {
      cost += (m_opt.ptPos(t+1, i) - (m_opt.ptPos(t, i) + dt*m_opt.ptVel(t, i))).squaredNorm();
      cost += (m_opt.ptVel(t+1, i) - (m_opt.ptVel(t, i) + dt*(METERS*OPhysConfig::gravity.transpose() + m_opt.ptAcc(t, i)))).squaredNorm();
    }
  }
  return cost;
}

ConvexObjectivePtr ParticleSystemOptimizer2::PhysicsStepCost::convexify(GRBModel *model) {
  const double dt = OPhysConfig::dt;
  ConvexObjectivePtr out(new ConvexObjective());
  out->m_name = "physics_step_cost";
  out->m_objective = 0;
  for (int t = 0; t < m_opt.m_horizon - 1; ++t) {
    for (int i = 0; i < m_opt.m_numParticles; ++i) {
      VarVec3View varPos = m_opt.varPtPos(t, i);
      VarVec3View varVel = m_opt.varPtVel(t, i);
      VarVec3View varAcc = m_opt.varPtAcc(t, i);
      VarVec3View varPos2 = m_opt.varPtPos(t+1, i);
      VarVec3View varVel2 = m_opt.varPtVel(t+1, i);
      //VarVec3View varAcc2 = varPtAcc(t+1, i);
      for (int j = 0; j < 3; ++j) {
        out->m_objective += square(varPos2[j] - (varPos[j] + dt*varVel[j]));
        out->m_objective += square(varVel2[j] - (varVel[j] + dt*(METERS*OPhysConfig::gravity(j) + varAcc[j])));
      }
    }
  }
  return out;
}


ParticleSystem2::ParticleSystem2(const ParticleSystemOptimizer2::SysState &initState) {
  m_currState = initState;
  m_plotSpheres.reset(new PlotSpheres());
}

void ParticleSystem2::attachToScene(Scene *s) {
  m_scene = s;
  m_scene->env->add(m_plotSpheres);
  draw();
}

void ParticleSystem2::draw() {
  osg::ref_ptr<osg::Vec3Array> centers(new osg::Vec3Array());
  osg::ref_ptr<osg::Vec4Array> rgba(new osg::Vec4Array());
  vector<float> radii;
  for (int i = 0; i < m_currState.rows(); ++i) {
    centers->push_back(osg::Vec3(m_currState(i, 0), m_currState(i, 1), m_currState(i, 2)));
    rgba->push_back(osg::Vec4(1, 0, 0, 1));
    radii.push_back(0.05);
  }
  m_plotSpheres->plot(centers, rgba, radii);
}


void ParticleSystem2::setupOpt(ParticleSystemOptimizer2 &opt) {
    m_trustRegion.reset(new ParticleSysTrustRegion(opt));
    opt.setTrustRegion(m_trustRegion);

  m_initCondCnt.reset(new ParticleSystemOptimizer2::InitCondConstraint(opt, m_currState));
  opt.addConstraint(m_initCondCnt);
/*
  ParticleSystemOptimizer2::InitCondCvxConstraint::Ptr asdf(new ParticleSystemOptimizer2::InitCondCvxConstraint(opt, m_currState));
  ConvexConstraintWrapper::AddToOpt(opt, asdf);*/

  m_groundCnt.reset(new ParticleSystemOptimizer2::GroundConstraint(opt, 0.01*METERS));
  opt.addConstraint(m_groundCnt);

  m_physicsStepCost.reset(new ParticleSystemOptimizer2::PhysicsStepCost(opt));
  opt.addCost(m_physicsStepCost);
}

void ParticleSystem2::step(double dt, int numSteps) {
  assert(numSteps >= 1);

  ParticleSystemOptimizer2 opt(m_currState.rows(), numSteps + 1);
  /*
  ParticleSysTrustRegion trustRegion(opt);
  opt.setTrustRegion(m_trustRegion);
  ParticleSystemOptimizer2::InitCondConstraint initCondCnt(opt, m_currState);
  ParticleSystemOptimizer2::PhysicsStepCost physicsStepCost;*/

  try {
    // first optimize subject to hard constraints with infinite trust region

    opt.initializeFromSingleState(m_currState);

    setupOpt(opt);


    

    opt.optimize();
    opt.optimize();

    /*m_trustRegion->setInfinite(true);
    opt.setTrustRegion(m_trustRegion);
    setupOpt0(opt);
    if (m_preOptCallback0) {
      m_preOptCallback0(&opt);
    }
    //opt.optimize();
    LOG_INFO(" ============ STEP0 DONE ================ ");

    // add in soft costs with usual trust region behavior
    m_trustRegion->resetTrustRegion();
    m_trustRegion->setInfinite(false);
    setupOpt(opt);
    if (m_preOptCallback) {
      m_preOptCallback(&opt);
    }
    opt.optimize();

    //cout << "physics step cost: " << m_physicsStepCost->evaluate() << '\n';
    */

  } catch (const GRBException &e) {
    LOG_ERROR("Gurobi exception (" << e.getErrorCode() << "): " << e.getMessage());
    throw;
  }

  m_currState = opt.m_sys[1];
  //m_currStates = opt.m_sys;

  draw();
  LOG_INFO(" ============ STEP DONE ================ ");
}


} // namespace ophys