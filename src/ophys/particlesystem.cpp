#include "particlesystem.h"

#include <boost/format.hpp>

#include "ophys_common.h"
#include "utils/config.h"
#include "utils/logging.h"

ParticleSystemOptimizer::ParticleSystemOptimizer(int numParticles, int horizon, const string &varPrefix)
  : m_numParticles(numParticles),
    m_horizon(horizon),
    m_varPrefix(varPrefix) {

  assert(m_horizon >= 2);

  for (int t = 0; t < m_horizon; ++t) {
    ParticleSystemState sys;
    for (int i = 0; i < m_numParticles; ++i) {
      ParticleState st;
      st.idx = i;
      st.invm = 1.0;
      sys.push_back(st);
    }
    m_sys.push_back(sys);
  }
}

void ParticleSystemOptimizer::updateValues() {
  for (int t = 0; t < m_horizon; ++t) {
    for (int i = 0; i < m_numParticles; ++i) {
      ParticleState &s = m_sys[t][i];
      for (int j = 0; j < 3; ++j) {
        s.x[j] = s.var_x[j].get(GRB_DoubleAttr_X);
        s.v[j] = s.var_v[j].get(GRB_DoubleAttr_X);
        s.a[j] = s.var_a[j].get(GRB_DoubleAttr_X);
      }
    }
  }
}

void ParticleSystemOptimizer::storeValues() {
  for (int t = 0; t < m_horizon; ++t) {
    for (int i = 0; i < m_numParticles; ++i) {
      ParticleState &s = m_sys[t][i];
      s.x_backup = s.x;
      s.v_backup = s.v;
      s.a_backup = s.a;
    }
  }
}

void ParticleSystemOptimizer::rollbackValues() {
  for (int t = 0; t < m_horizon; ++t) {
    for (int i = 0; i < m_numParticles; ++i) {
      ParticleState &s = m_sys[t][i];
      s.x = s.x_backup;
      s.v = s.v_backup;
      s.a = s.a_backup;
    }
  }
}

/*
void ParticleSystemOptimizer::createVars() {
  boost::format fmt("%s_t%d_i%d_%c_%d");
  for (int t = 0; t < m_horizon; ++t) {
    for (int i = 0; i < m_numParticles; ++i) {
      ParticleState &s = m_sys[t][i];
      for (int j = 0; j < 3; ++j) {
        s.var_x[j] = m_model->addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, (fmt % m_varPrefix % t % i % 'x' % j).str());
        s.var_v[j] = m_model->addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, (fmt % m_varPrefix % t % i % 'v' % j).str());
        s.var_a[j] = m_model->addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, (fmt % m_varPrefix % t % i % 'a' % j).str());
      }
    }
  }
  m_model->update();
}

void ParticleSystemOptimizer::setValues(const ParticleSystemStateVec &init) {
  assert(init.size() == m_horizon);
  for (int t = 0; t < m_horizon; ++t) {
    assert(init[t].size() == m_numParticles);
    for (int i = 0; i < m_numParticles; ++i) {
      ParticleState &s = m_sys[t][i];
      s.x = init[t][i].x;
      s.v = init[t][i].v;
      s.a = init[t][i].a;
    }
  }
}
*/

void ParticleSystemOptimizer::initialize(const ParticleSystemStateVec &init) {
  /*
  createVars();
  setValues(init);
  */
  boost::format fmt("%s_t%d_i%d_%c_%d");
  assert(init.size() == m_horizon);
  for (int t = 0; t < m_horizon; ++t) {
    assert(init[t].size() == m_numParticles);
    for (int i = 0; i < m_numParticles; ++i) {
      ParticleState &s = m_sys[t][i];
      s.x = init[t][i].x;
      s.v = init[t][i].v;
      s.a = init[t][i].a;
      for (int j = 0; j < 3; ++j) {
        s.var_x[j] = m_model->addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, (fmt % m_varPrefix % t % i % 'x' % j).str());
        s.var_v[j] = m_model->addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, (fmt % m_varPrefix % t % i % 'v' % j).str());
        s.var_a[j] = m_model->addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, (fmt % m_varPrefix % t % i % 'a' % j).str());
      }
    }
  }
  m_model->update();
}

void ParticleSystemOptimizer::initializeFromSingleState(const ParticleSystemState &s0) {
  ParticleSystemStateVec v;
  assert(s0.size() == m_numParticles);
  v.reserve(m_horizon);
  for (int t = 0; t < m_horizon; ++t) {
    v.push_back(s0);
  }
  initialize(v);
}

/*
void ParticleSystemOptimizer::preOptimize() {
  cout << "pre-"; postOptimize();
}
*/

void ParticleSystemOptimizer::postOptimize() {
  LOG_INFO("System state:");
  for (int t = 0; t < m_horizon; ++t) {
    for (int i = 0; i < m_numParticles; ++i) {
      LOG_INFO(boost::format("x(%d,%d) = ") % t % i << m_sys[t][i].x.transpose());
      LOG_INFO(boost::format("v(%d,%d) = ") % t % i << m_sys[t][i].v.transpose());
      LOG_INFO(boost::format("a(%d,%d) = ") % t % i << m_sys[t][i].a.transpose());
    }
  }
}


ParticleSystemTrustRegion::ParticleSystemTrustRegion(ParticleSystemOptimizer &opt)
  : m_opt(opt),
    m_x_radii(boost::extents[opt.m_horizon][opt.m_numParticles][3]),
    m_v_radii(boost::extents[opt.m_horizon][opt.m_numParticles][3]),
    m_a_radii(boost::extents[opt.m_horizon][opt.m_numParticles][3])
{
  for (int t = 0; t < m_opt.m_horizon; ++t) {
    for (int i = 0; i < m_opt.m_numParticles; ++i) {
      for (int j = 0; j < 3; ++j) {
        m_x_radii[t][i][j] = OPhysConfig::trustRadius * METERS;
        m_v_radii[t][i][j] = OPhysConfig::trustRadius * METERS;
        m_a_radii[t][i][j] = OPhysConfig::trustRadius * METERS;
      }
    }
  }
}

void ParticleSystemTrustRegion::adjustTrustRegion(double ratio) {
  m_shrinkage *= ratio;
  for (int t = 0; t < m_opt.m_horizon; ++t) {
    for (int i = 0; i < m_opt.m_numParticles; ++i) {
      for (int j = 0; j < 3; ++j) {
        m_x_radii[t][i][j] *= ratio;
        m_v_radii[t][i][j] *= ratio;
        m_a_radii[t][i][j] *= ratio;
      }
    }
  }
}

ConvexConstraintPtr ParticleSystemTrustRegion::convexify(GRBModel* model) {
/*
  ConvexConstraintPtr cnt(new ConvexConstraint());
  cnt->m_name = "trust_region";

  boost::format fmt("%s_trust_t%d_i%d_%c_%d_%s");
  for (int t = 1; t < m_opt.m_horizon; ++t) { // start at 1, since t=0 is constrained anyway
    for (int i = 0; i < m_opt.m_numParticles; ++i) {
      ParticleState &s = m_opt.m_sys[t][i];
      for (int j = 0; j < 3; ++j) {
        cnt->m_cntNames.push_back((fmt % m_opt.m_varPrefix % t % i % 'x' % j % "lb").str());
        cnt->m_exprs.push_back(-s.var_x[j] + (s.x[j] - m_x_radii[t][i][j]));
        cnt->m_cntNames.push_back((fmt % m_opt.m_varPrefix % t % i % 'x' % j % "ub").str());
        cnt->m_exprs.push_back(s.var_x[j] - (s.x[j] + m_x_radii[t][i][j]));

        cnt->m_cntNames.push_back((fmt % m_opt.m_varPrefix % t % i % 'v' % j % "lb").str());
        cnt->m_exprs.push_back(-s.var_v[j] + (s.v[j] - m_v_radii[t][i][j]));
        cnt->m_cntNames.push_back((fmt % m_opt.m_varPrefix % t % i % 'v' % j % "ub").str());
        cnt->m_exprs.push_back(s.var_v[j] - (s.v[j] + m_v_radii[t][i][j]));

        cnt->m_cntNames.push_back((fmt % m_opt.m_varPrefix % t % i % 'a' % j % "lb").str());
        cnt->m_exprs.push_back(-s.var_a[j] + (s.a[j] - m_a_radii[t][i][j]));
        cnt->m_cntNames.push_back((fmt % m_opt.m_varPrefix % t % i % 'a' % j % "ub").str());
        cnt->m_exprs.push_back(s.var_a[j] - (s.a[j] + m_a_radii[t][i][j]));
      }
    }
  }

  return cnt;*/

  for (int t = 1; t < m_opt.m_horizon; ++t) {
    for (int i = 0; i < m_opt.m_numParticles; ++i) {
      ParticleState &s = m_opt.m_sys[t][i];
      for (int j = 0; j < 3; ++j) {
        s.var_x[j].set(GRB_DoubleAttr_LB, s.x[j] - m_x_radii[t][i][j]);
        s.var_x[j].set(GRB_DoubleAttr_UB, s.x[j] + m_x_radii[t][i][j]);

        s.var_v[j].set(GRB_DoubleAttr_LB, s.v[j] - m_v_radii[t][i][j]);
        s.var_v[j].set(GRB_DoubleAttr_UB, s.v[j] + m_v_radii[t][i][j]);

        s.var_a[j].set(GRB_DoubleAttr_LB, s.a[j] - m_a_radii[t][i][j]);
        s.var_a[j].set(GRB_DoubleAttr_UB, s.a[j] + m_a_radii[t][i][j]);
      }
    }
  }

  ConvexConstraintPtr cnt(new ConvexConstraint());
  cnt->m_name = "trust_region";
  return cnt;
}

// s0 only needs x,v,a filled in
InitialConditionConstraints::InitialConditionConstraints(ParticleSystemOptimizer &o, const ParticleSystemState &s0) {
  m_name = "initialconditionconstraints";
  boost::format fmt("%s_initcond_i%d_%c_%d");
  for (int i = 0; i < o.m_numParticles; ++i) {
    for (int j = 0; j < 3; ++j) {
      m_eqcntNames.push_back((fmt % o.m_varPrefix % i % 'x' % j).str());
      m_eqexprs.push_back(o.m_sys[0][i].var_x[j] - s0[i].x[j]);

      m_eqcntNames.push_back((fmt % o.m_varPrefix % i % 'v' % j).str());
      m_eqexprs.push_back(o.m_sys[0][i].var_v[j] - s0[i].v[j]);

      m_eqcntNames.push_back((fmt % o.m_varPrefix % i % 'a' % j).str());
      m_eqexprs.push_back(o.m_sys[0][i].var_a[j] - s0[i].a[j]);
    }
  }
}

NoExternalForcesConstraint::NoExternalForcesConstraint(ParticleSystemOptimizer &o) {
  m_name = "noexternalforcesconstraint";
  boost::format fmt("%s_noextforces_t%d_i%d_%d");
  for (int t = 0; t < o.m_horizon; ++t) {
    for (int i = 0; i < o.m_numParticles; ++i) {
      for (int j = 0; j < 3; ++j) {
        m_eqcntNames.push_back((fmt % o.m_varPrefix % t % i % j).str());
        m_eqexprs.push_back(o.m_sys[t][i].var_a[j]);
      }
    }
  }
}

double PhysicsStepCost::evaluate() {
  const double dt = OPhysConfig::dt;
  double cost = 0;
  for (int t = 0; t < m_opt.m_horizon - 1; ++t) {
    for (int i = 0; i < m_opt.m_numParticles; ++i) {
      for (int j = 0; j < 3; ++j) {
        // position step cost
        cost += ophys::square(m_opt.m_sys[t+1][i].x[j] - (m_opt.m_sys[t][i].x[j] + dt*m_opt.m_sys[t][i].v[j]));
        // velocity step cost
        cost += ophys::square(m_opt.m_sys[t+1][i].v[j] - (m_opt.m_sys[t][i].v[j] + dt*(m_opt.m_sys[t][i].a[j] + METERS*OPhysConfig::gravity[j])));
      }
    }
  }
  return cost;
}

ConvexObjectivePtr PhysicsStepCost::convexify(GRBModel* model) {
  const double dt = OPhysConfig::dt;
  ConvexObjectivePtr out(new ConvexObjective());
  out->m_name = "physics_step_cost";
  out->m_objective = 0;
  for (int t = 0; t < m_opt.m_horizon - 1; ++t) {
    for (int i = 0; i < m_opt.m_numParticles; ++i) {
      for (int j = 0; j < 3; ++j) {
        // position step cost
        out->m_objective += ophys::square(m_opt.m_sys[t+1][i].var_x[j] - (m_opt.m_sys[t][i].var_x[j] + dt*m_opt.m_sys[t][i].var_v[j]));
        // velocity step cost
        out->m_objective += ophys::square(m_opt.m_sys[t+1][i].var_v[j] - (m_opt.m_sys[t][i].var_v[j] + dt*(m_opt.m_sys[t][i].var_a[j] + METERS*OPhysConfig::gravity[j])));
      }
    }
  }
  return out;
}


ParticleSystem::ParticleSystem(const ParticleSystemState &initState) {
 //   m_plotSpheres(new PlotSpheres(3)) {

  m_currState = initState;
}

void ParticleSystem::step(double dt) {
  // update initial condition (start simulating from current state)
  ParticleSystemOptimizer opt(m_currState.size(), 2);
  opt.initializeFromSingleState(m_currState);
  opt.setTrustRegion(ParticleSystemTrustRegion::Ptr(new ParticleSystemTrustRegion(opt)));

  opt.addCost(PhysicsStepCost::Ptr(new PhysicsStepCost(opt)));

  opt.addConstraint(ConvexConstraintWrapper::Ptr(new ConvexConstraintWrapper(InitialConditionConstraints::Ptr(new InitialConditionConstraints(opt, m_currState)))));
  opt.addConstraint(ConvexConstraintWrapper::Ptr(new ConvexConstraintWrapper(NoExternalForcesConstraint::Ptr(new NoExternalForcesConstraint(opt)))));

  opt.optimize();

  m_currState = opt.m_sys[1];

  updateOSG();
}


/*
ParticleSystem(const ParticleSystemState &initState) {
 //   m_plotSpheres(new PlotSpheres(3)) {

  m_currState = initState;

  // this is a 2-step horizon (the initial state, and the state at the next timestep)
  m_opt.reset(new ParticleSystemOptimizer(initState.size(), 2));
  m_opt->initializeFromSingleState(m_currState);
  m_opt->setTrustRegion(ParticleSystemTrustRegion::Ptr(new ParticleSystemTrustRegion(*m_opt)));

  m_opt->addConstraint(ConvexConstraintWrapper::Ptr(new ConvexConstraintWrapper(NoExternalForcesConstraint::Ptr(new NoExternalForcesConstraint(*m_opt)))));

  m_opt->addCost(PhysicsStepCost::Ptr(new PhysicsStepCost(*m_opt)));
}

void step(double dt) {
  updateOSG();
  // update initial condition (start simulating from current state)
  //m_opt->setValuesFromSingleState(m_currState);
  if (m_initCondConstraints) {
    m_opt->removeConstraint(m_initCondConstraints);
  }
  m_initCondConstraints.reset(new ConvexConstraintWrapper(InitialConditionConstraints::Ptr(new InitialConditionConstraints(*m_opt, m_currState))));
  m_opt->addConstraint(m_initCondConstraints);

  m_opt->optimize();

  m_currState = m_opt->m_sys[1];

  updateOSG();
}
*/

void ParticleSystem::step() {
  step(OPhysConfig::dt);
}

void ParticleSystem::updateOSG() {
  for (int i = 0; i < m_currState.size(); ++i) {
    LOG_INFO("x(" << i << "): " << m_currState[i].x.transpose());
    LOG_INFO("v(" << i << "): " << m_currState[i].v.transpose());
    LOG_INFO("a(" << i << "): " << m_currState[i].a.transpose());
  }
}
