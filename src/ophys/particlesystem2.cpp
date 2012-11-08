#include "particlesystem2.h"

#include "utils/logging.h"
#include "ophys_common.h"

#include "simulation/simplescene.h"

namespace ophys {

const int ParticleSystemOptimizer2::PARTICLE_STATE_DIM;
const double TABLE_HEIGHT = 0.01;

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
    m_infinite(false),
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
        m_opt.m_varsys[t][i][j].set(GRB_DoubleAttr_LB, m_infinite ? -GRB_INFINITY : m_opt.m_sys[t](i,j) - m_radii[t][i][j]);
        m_opt.m_varsys[t][i][j].set(GRB_DoubleAttr_UB, m_infinite ? GRB_INFINITY : m_opt.m_sys[t](i,j) + m_radii[t][i][j]);
      }
    }
  }
  ConvexConstraintPtr cnt(new ConvexConstraint());
  cnt->m_name = "trust_region";
  return cnt;
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

#if 0
ConvexConstraintPtr ParticleSystemOptimizer2::PhysicsStepConstraint::convexify(GRBModel *model) {
  const double dt = OPhysConfig::dt;
  ConvexConstraintPtr out(new ConvexConstraint());
  out->m_name = "physics_step_constraint";
  boost::format fmt("%s_physics_step_constraint_t%d_i%d_%s_%d");
  // position-based dynamics steps
  for (int t = 0; t < m_opt.m_horizon - 1; ++t) {
    for (int i = 0; i < m_opt.m_numParticles; ++i) {
      VarVec3View varPos = m_opt.varPtPos(t, i);
      VarVec3View varVel = m_opt.varPtVel(t, i);
      VarVec3View varAcc = m_opt.varPtAcc(t, i);
      VarVec3View varPos2 = m_opt.varPtPos(t+1, i);
      VarVec3View varVel2 = m_opt.varPtVel(t+1, i);
      //VarVec3View varAcc2 = m_opt.varPtAcc(t+1, i);
      for (int j = 0; j < 3; ++j) {
        // position update
        out->m_eqcntNames.push_back((fmt % m_opt.m_varPrefix % t % i % "pos" % j).str());
        out->m_eqexprs.push_back(varPos2[j] - (varPos[j] + dt*(varVel[j] + dt*(varAcc[j] + METERS*OPhysConfig::gravity(j)))));

        // velocity update
        out->m_eqcntNames.push_back((fmt % m_opt.m_varPrefix % t % i % "vel" % j).str());
        out->m_eqexprs.push_back(varVel2[j] - (varPos2[j] - varPos[j])/dt);
      }
    }
  }
  return out;
}
#endif

ConvexConstraintPtr ParticleSystemOptimizer2::SemiImplicitEulerConstraint::convexify(GRBModel *) {
  const double dt = OPhysConfig::dt;
  ConvexConstraintPtr out(new ConvexConstraint());
  out->m_name = "physics_step_constraint";
  boost::format fmt("%s_physics_step_constraint_t%d_i%d_%s_%d");
  // position-based dynamics steps
  for (int t = 0; t < m_opt.m_horizon - 1; ++t) {
    for (int i = 0; i < m_opt.m_numParticles; ++i) {
      VarVec3View varPos = m_opt.varPtPos(t, i);
      VarVec3View varVel = m_opt.varPtVel(t, i);
      VarVec3View varAcc = m_opt.varPtAcc(t, i);
      VarVec3View varPos2 = m_opt.varPtPos(t+1, i);
      VarVec3View varVel2 = m_opt.varPtVel(t+1, i);
      //VarVec3View varAcc2 = m_opt.varPtAcc(t+1, i);
      for (int j = 0; j < 3; ++j) {
        // position update
        out->m_eqcntNames.push_back((fmt % m_opt.m_varPrefix % t % i % "pos" % j).str());
        out->m_eqexprs.push_back(varPos2[j] - (varPos[j] + dt*varVel2[j]));

        // velocity update
        out->m_eqcntNames.push_back((fmt % m_opt.m_varPrefix % t % i % "vel" % j).str());
        out->m_eqexprs.push_back(varVel2[j] - (varVel[j] + dt*(METERS*OPhysConfig::gravity(j) + varAcc[j])));
      }
    }
  }
  return out;
}


ConvexConstraintPtr ParticleSystemOptimizer2::ForceConstraint::convexify(GRBModel* ) {
  boost::format fmt("%s_force_constraint_t%d_i%d_%d");
  ConvexConstraintPtr out(new ConvexConstraint());
  out->m_name = "force_constraint";
  for (int t = 1; t < m_opt.m_horizon; ++t) { // start at 1 to ignore initial conditions
    for (int i = 0; i < m_opt.m_numParticles; ++i) {
      for (int j = 0; j < 3; ++j) {
        GRBVar &a = m_opt.varPtAcc(t, i)[j];
        switch (m_mode) {
        case UNCONSTRAINED:
          break;
        case GRAVITY_ONLY:
          out->m_eqcntNames.push_back((fmt % m_opt.m_varPrefix % t % i % j).str());
          out->m_eqexprs.push_back(a);
          //out->m_eqexprs.push_back(a - METERS*OPhysConfig::gravity(j));
          break;
        }
      }
    }
  }
  return out;
}


#if 0
// this is the explicit euler update

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
#endif

#if 0
// position-based dynamics step violations

double ParticleSystemOptimizer2::PhysicsStepCost::evaluate() {
  const double dt = OPhysConfig::dt;
  double cost = 0;
  for (int t = 0; t < m_opt.m_horizon - 1; ++t) {
    for (int i = 0; i < m_opt.m_numParticles; ++i) {
      cost += (m_opt.ptPos(t+1,i) -
                (m_opt.ptPos(t,i) + dt*(m_opt.ptVel(t,i) + dt*(METERS*OPhysConfig::gravity.transpose()))) // TODO: add accel
              ).squaredNorm();
      cost += (m_opt.ptVel(t+1,i) -
                (m_opt.ptPos(t+1,i) - m_opt.ptPos(t,i))/dt
              ).squaredNorm();
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
      //VarVec3View varAcc = m_opt.varPtAcc(t, i);
      VarVec3View varPos2 = m_opt.varPtPos(t+1, i);
      VarVec3View varVel2 = m_opt.varPtVel(t+1, i);
      //VarVec3View varAcc2 = m_opt.varPtAcc(t+1, i);
      for (int j = 0; j < 3; ++j) {
        out->m_objective += square(
          varPos2[j] -
            (varPos[j] + dt*(varVel[j] + dt*(METERS*OPhysConfig::gravity(j))))
        );
        out->m_objective += square(
          varVel2[j] -
            (varPos2[j] - varPos[j])/dt
        );
      }
    }
  }
  return out;
}
#endif


GRBLinExpr ParticleSystemOptimizer2::linearize(const FuncOnStatesOverTime &fn, SysStatesOverTime &x0) {
  const double eps = 1e-6;
  double y0 = fn(&x0);
  GRBLinExpr y(y0);
  for (int t = 0; t < m_horizon; ++t) {
    for (int i = 0; i < m_numParticles; ++i) {
      for (int j = 0; j < PARTICLE_STATE_DIM; ++j) {
        x0[t](i,j) -= eps/2.;
        double yminus = fn(&x0);
        x0[t](i,j) += eps;
        double yplus = fn(&x0);
        x0[t](i,j) -= eps/2.;
        double yp = (yplus - yminus)/eps;
        GRBLinExpr dx = m_varsys[t][i][j] - x0[t](i,j);
        y += yp*dx;
      }
    }
  }
  return y;
}


GRBQuadExpr ParticleSystemOptimizer2::quadraticize(const FuncOnStatesOverTime &fn, SysStatesOverTime &x0) {
  const double eps = 1e-6;
  double y0 = fn(&x0);
  GRBQuadExpr y(y0);
  for (int t = 0; t < m_horizon; ++t) {
    for (int i = 0; i < m_numParticles; ++i) {
      for (int j = 0; j < PARTICLE_STATE_DIM; ++j) {
        x0[t](i,j) -= eps/2.;
        double yminus = fn(&x0);
        x0[t](i,j) += eps;
        double yplus = fn(&x0);
        x0[t](i,j) -= eps/2.;
        double yp = (yplus - yminus)/eps;
        double ypp = (yplus + yminus - 2*y0)/(eps*eps/4.);
        GRBLinExpr dx = m_varsys[t][i][j] - x0[t](i,j);
        y += 0.5*ypp*dx*dx + yp*dx;
      }
    }
  }
  return y;
}



double ParticleSystemOptimizer2::ConstrainedPhysicsStepCost::evaluate() {
  return evaluateAt(&m_opt.m_sys);
}

double ParticleSystemOptimizer2::ConstrainedPhysicsStepCost::evaluateAt(const SysStatesOverTime *sys) {
  const double dt = OPhysConfig::dt;
  double cost = 0;
  for (int t = 0; t < m_opt.m_horizon - 1; ++t) {
    for (int i = 0; i < m_opt.m_numParticles; ++i) {
      // position-based dynamics step violations
      //double alpha = max(0.0, ptPos((*sys)[t],i)(2) - METERS*TABLE_HEIGHT);
      //double alpha = square(ptPos((*sys)[t],i)(2) - METERS*TABLE_HEIGHT);
      double alpha = 1;
      cost += (ptPos((*sys)[t+1],i) -
                (ptPos((*sys)[t],i) + dt*(ptVel((*sys)[t],i) + dt*(METERS*OPhysConfig::gravity.transpose()))) // TODO: add accel
              ).squaredNorm() * alpha;
      /*
      cost += (m_opt.ptVel(t+1,i) -
                (m_opt.ptPos(t+1,i) - m_opt.ptPos(t,i))/dt
              ).squaredNorm();*/
    }
  }
  return cost;
}

ConvexObjectivePtr ParticleSystemOptimizer2::ConstrainedPhysicsStepCost::convexify(GRBModel *model) {
  const double dt = OPhysConfig::dt;

  boost::format fmt("%s_physics_step_constraint_t%d_i%d_%s_%d");

  ConvexObjectivePtr out(new ConvexObjective());
  out->m_name = "physics_step_cost";

  // hard constraints
  for (int t = 0; t < m_opt.m_horizon - 1; ++t) {
    for (int i = 0; i < m_opt.m_numParticles; ++i) {
      VarVec3View varPos = m_opt.varPtPos(t, i);
      VarVec3View varVel = m_opt.varPtVel(t, i);
      //VarVec3View varAcc = m_opt.varPtAcc(t, i);
      VarVec3View varPos2 = m_opt.varPtPos(t+1, i);
      VarVec3View varVel2 = m_opt.varPtVel(t+1, i);
      //VarVec3View varAcc2 = m_opt.varPtAcc(t+1, i);
      for (int j = 0; j < 3; ++j) {
        // velocity update
        out->m_eqcntNames.push_back((fmt % m_opt.m_varPrefix % t % i % "vel" % j).str());
        out->m_eqexprs.push_back(varVel2[j] - (varPos2[j] - varPos[j])/dt);
      }
    }
  }

  // convexified position violation cost
  //out->m_objective = m_opt.quadraticize(boost::bind(&ConstrainedPhysicsStepCost::evaluateAt, this, _1), m_opt.m_sys);
  //out->m_objective = m_opt.linearize(boost::bind(&ConstrainedPhysicsStepCost::evaluateAt, this, _1), m_opt.m_sys);

  for (int t = 0; t < m_opt.m_horizon - 1; ++t) {
    for (int i = 0; i < m_opt.m_numParticles; ++i) {
      VarVec3View varPos = m_opt.varPtPos(t, i);
      VarVec3View varVel = m_opt.varPtVel(t, i);
      //VarVec3View varAcc = m_opt.varPtAcc(t, i);
      VarVec3View varPos2 = m_opt.varPtPos(t+1, i);
      VarVec3View varVel2 = m_opt.varPtVel(t+1, i);
      //VarVec3View varAcc2 = m_opt.varPtAcc(t+1, i);
      for (int j = 0; j < 3; ++j) {
        // position update
        // (as l2 loss)
        /*out->m_objective += square(
          varPos2[j] -
            (varPos[j] + dt*(varVel[j] + dt*(METERS*OPhysConfig::gravity(j))))
        );*/

        // (as l1 loss)
        /*GRBLinExpr diff = varPos2[j] -
            (varPos[j] + dt*(varVel[j] + dt*(METERS*OPhysConfig::gravity(j))));
        GRBVar posViolation = model->addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS);
        out->m_vars.push_back(posViolation);
        out->m_cntNames.push_back((fmt % m_opt.m_varPrefix % t % i % "pos_l1_lb" % j).str());
        out->m_exprs.push_back(-posViolation - diff);
        out->m_cntNames.push_back((fmt % m_opt.m_varPrefix % t % i % "pos_l1_ub" % j).str());
        out->m_exprs.push_back(-posViolation + diff);
        out->m_objective += posViolation;*/

        
        // (as equality constraint with slack)
        GRBVar posSlack = model->addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS);
        out->m_vars.push_back(posSlack);
        out->m_eqcntNames.push_back((fmt % m_opt.m_varPrefix % t % i % "pos" % j).str());
        out->m_eqexprs.push_back(varPos2[j] - (varPos[j] + dt*(varVel[j] + dt*(METERS*OPhysConfig::gravity(j)))) + posSlack);
        // position violations should be nonzero only when the object is in/near contact
        /*if (j == 2) {
          if (m_opt.ptPos(t,i)[2] > TABLE_HEIGHT*METERS) {
            out->m_eqcntNames.push_back((fmt % m_opt.m_varPrefix % t % i % "pos_slack" % j).str());
            out->m_eqexprs.push_back(posSlack);
          }
        }*/
        out->m_objective += posSlack*posSlack;
/*
        if (j == 2) {
          // gives posAux == max(0, threshold - signedViolation)
          GRBVar posAux = model->addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS);
          out->m_vars.push_back(posAux);
          out->m_objective += posAux;
          out->m_cntNames.push_back((fmt % m_opt.m_varPrefix % t % i % "pos_aux" % j).str());
          out->m_exprs.push_back(TABLE_HEIGHT*METERS - varPos[2] - posAux);
          // now posAux will be 0 when far from the ground but > 0 when close
          // so constrain posSlack to be 0 when posAux is 0, but don't constrain otherwise
          out->m_exprs.push_back();
        }
        out->m_objective += posAux*posAux * posSlack*posSlack;*/

        // (as equality constraint)
        /*out->m_eqcntNames.push_back((fmt % m_opt.m_varPrefix % t % i % "pos" % j).str());
        out->m_eqexprs.push_back(varPos2[j] - (varPos[j] + dt*(varVel[j] + dt*(METERS*OPhysConfig::gravity(j)))));*/

      }
    }
  }

  return out;
}


double ParticleSystemOptimizer2::PDPosViolationCost::evaluate() {
  return evaluateAt(&m_opt.m_sys);
}

double ParticleSystemOptimizer2::PDPosViolationCost::evaluateAt(const SysStatesOverTime *s) {
  const SysStatesOverTime &sys = *s;
  const double dt = OPhysConfig::dt;
  double cost = 0;
  for (int t = 0; t < m_opt.m_horizon - 1; ++t) {
    for (int i = 0; i < m_opt.m_numParticles; ++i) {
      // position-based dynamics step violations
      //double alpha = max(0.0, ptPos((*sys)[t],i)(2) - METERS*TABLE_HEIGHT);
      //double alpha = square(ptPos((*sys)[t],i)(2) - METERS*TABLE_HEIGHT);
      double alpha = 1;
      cost += (ptPos(sys[t+1],i) -
                (ptPos(sys[t],i) + dt*(ptVel(sys[t],i) + dt*(METERS*OPhysConfig::gravity.transpose()))) // TODO: add accel
              ).squaredNorm() * alpha;
    }
  }
  return cost;
}

ConvexObjectivePtr ParticleSystemOptimizer2::PDPosViolationCost::convexify(GRBModel* model) {
  const double dt = OPhysConfig::dt;
  boost::format fmt("%s_pd_pos_violation_cost_t%d_i%d_%d");
  ConvexObjectivePtr out(new ConvexObjective());
  out->m_name = "pd_pos_violation_cost";
  for (int t = 0; t < m_opt.m_horizon - 1; ++t) {
    for (int i = 0; i < m_opt.m_numParticles; ++i) {
      VarVec3View varPos = m_opt.varPtPos(t, i);
      VarVec3View varVel = m_opt.varPtVel(t, i);
      //VarVec3View varAcc = m_opt.varPtAcc(t, i);
      VarVec3View varPos2 = m_opt.varPtPos(t+1, i);
      VarVec3View varVel2 = m_opt.varPtVel(t+1, i);
      //VarVec3View varAcc2 = m_opt.varPtAcc(t+1, i);
      for (int j = 0; j < 3; ++j) {
        // (as equality constraint with slack)
        GRBVar posViolation = model->addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS);
        out->m_vars.push_back(posViolation);
        out->m_eqcntNames.push_back((fmt % m_opt.m_varPrefix % t % i % j).str());
        out->m_eqexprs.push_back(varPos2[j] - (varPos[j] + dt*(varVel[j] + dt*(METERS*OPhysConfig::gravity(j)))) + posViolation);
        out->m_objective += posViolation*posViolation;
      }
    }
  }
  return out;
}

ConvexConstraintPtr ParticleSystemOptimizer2::PDVelocityConstraint::convexify(GRBModel *) {
  const double dt = OPhysConfig::dt;
  boost::format fmt("%s_pd_vel_constraint_t%d_i%d_%d");
  ConvexConstraintPtr out(new ConvexConstraint());
  out->m_name = "pd_cel_constraint";
  // hard constraints
  for (int t = 0; t < m_opt.m_horizon - 1; ++t) {
    for (int i = 0; i < m_opt.m_numParticles; ++i) {
      VarVec3View varPos = m_opt.varPtPos(t, i);
      VarVec3View varVel = m_opt.varPtVel(t, i);
      //VarVec3View varAcc = m_opt.varPtAcc(t, i);
      VarVec3View varPos2 = m_opt.varPtPos(t+1, i);
      VarVec3View varVel2 = m_opt.varPtVel(t+1, i);
      //VarVec3View varAcc2 = m_opt.varPtAcc(t+1, i);
      for (int j = 0; j < 3; ++j) {
        // velocity update
        out->m_eqcntNames.push_back((fmt % m_opt.m_varPrefix % t % i % j).str());
        out->m_eqexprs.push_back(varVel2[j] - (varPos2[j] - varPos[j])/dt);
      }
    }
  }
  return out;
}

double ParticleSystemOptimizer2::GroundCost::evaluate() {
  double cost = 0;
  for (int t = 0; t < m_opt.m_horizon; ++t) {
    for (int i = 0; i < m_opt.m_numParticles; ++i) {
      double z = m_opt.ptPos(t,i)(2);
      if (z < m_groundZ) {
        cost += square(abs(z - m_groundZ));
      }
    }
  }
  return cost;
}

ConvexObjectivePtr ParticleSystemOptimizer2::GroundCost::convexify(GRBModel *model) {
  ConvexObjectivePtr out(new ConvexObjective());
  out->m_name = "ground_cost";
  out->m_objective = 0;
  GRBLinExpr err(0);
  boost::format fmt("ground_hinge_cost_t%d_i%d");
  for (int t = 0; t < m_opt.m_horizon; ++t) {
    for (int i = 0; i < m_opt.m_numParticles; ++i) {
      GRBLinExpr err = m_groundZ - m_opt.varPtPos(t, i)[2];
      addHingeCostSq(out, 1, err, model, (fmt % t % i).str());
    }
  }
  return out;
}


double ParticleSystemOptimizer2::AccelCost::evaluate() {
  double cost = 0;
  for (int t = 0; t < m_opt.m_horizon - 1; ++t) {
    for (int i = 0; i < m_opt.m_numParticles; ++i) {
      cost += /*(m_opt.ptAcc(t+1,i) - m_opt.ptAcc(t,i)).squaredNorm() + */m_opt.ptAcc(t,i).squaredNorm();
    }
  }
  return cost;
}

ConvexObjectivePtr ParticleSystemOptimizer2::AccelCost::convexify(GRBModel *) {
  ConvexObjectivePtr out(new ConvexObjective());
  out->m_name = "accel_cost";
  out->m_objective = 0;
  boost::format fmt("accel_cost_t%d_i%d");
  for (int t = 0; t < m_opt.m_horizon - 1; ++t) {
    for (int i = 0; i < m_opt.m_numParticles; ++i) {
      for (int j = 0; j < 3; ++j) {
       // GRBLinExpr diff = m_opt.varPtAcc(t+1,i)[j] - m_opt.varPtAcc(t,i)[j];
        out->m_objective += /*diff*diff +*/ m_opt.varPtAcc(t,i)[j]*m_opt.varPtAcc(t,i)[j];
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

void ParticleSystem2::draw(const ParticleSystemOptimizer2::SysState &state) {
  osg::ref_ptr<osg::Vec3Array> centers(new osg::Vec3Array());
  osg::ref_ptr<osg::Vec4Array> rgba(new osg::Vec4Array());
  vector<float> radii;
  for (int i = 0; i < state.rows(); ++i) {
    centers->push_back(osg::Vec3(state(i, 0), state(i, 1), state(i, 2)));
    rgba->push_back(osg::Vec4(1, 0, 0, 1));
    radii.push_back(0.05);
  }
  m_plotSpheres->plot(centers, rgba, radii);
}

void ParticleSystem2::draw() {
  draw(m_currState);
}

void ParticleSystem2::setupOpt(ParticleSystemOptimizer2 &opt) {
  m_initCondCnt.reset(new ParticleSystemOptimizer2::InitCondConstraint(opt, m_currState));
  opt.addConstraint(m_initCondCnt);

  //m_constrainedPhysicsStepCost.reset(new ParticleSystemOptimizer2::ConstrainedPhysicsStepCost(opt));
  //opt.addCost(m_constrainedPhysicsStepCost);


  m_pdVelCnt.reset(new ParticleSystemOptimizer2::PDVelocityConstraint(opt));
  opt.addConstraint(m_pdVelCnt);

  //m_physicsStepCnt.reset(new ParticleSystemOptimizer2::SemiImplicitEulerConstraint(opt));
  //opt.addConstraint(m_physicsStepCnt);

  //m_groundCnt.reset(new ParticleSystemOptimizer2::GroundConstraint(opt, TABLE_HEIGHT*METERS));
  //opt.addConstraint(m_groundCnt);

  //m_forceConstraint.reset(new ParticleSystemOptimizer2::ForceConstraint(opt));
  //m_forceConstraint->setMode(ParticleSystemOptimizer2::ForceConstraint::GRAVITY_ONLY);
  //opt.addConstraint(m_forceConstraint);
}

void ParticleSystem2::setupOpt2(ParticleSystemOptimizer2 &opt) {
  //m_forceConstraint->setMode(ParticleSystemOptimizer2::ForceConstraint::UNCONSTRAINED);

  m_groundCost.reset(new ParticleSystemOptimizer2::GroundCost(opt, TABLE_HEIGHT*METERS));
  opt.addCost(m_groundCost);

  m_pdPosViolationCost.reset(new ParticleSystemOptimizer2::PDPosViolationCost(opt));
  opt.addCost(m_pdPosViolationCost);

  //m_accelCost.reset(new ParticleSystemOptimizer2::AccelCost(opt));
  //opt.addCost(m_accelCost);

}

void ParticleSystem2::step(double dt, int numSteps) {
  assert(numSteps >= 1);

  ParticleSystemOptimizer2 opt(m_currState.rows(), numSteps + 1);
  try {
    //boost::timer timer;
    opt.initializeFromSingleState(m_currState);

    // phase 1: find a nominal solution that satisfies all hard NON-APPROXIMATED constraints
    // (need to disable trust region constraints)
    m_trustRegion.reset(new ParticleSysTrustRegion(opt));
    opt.setTrustRegion(m_trustRegion);
    m_trustRegion->setInfinite(true);
    setupOpt(opt);
    setupOpt2(opt);
    opt.optimize();
/*
    cout << "phase 2" << endl;

    // phase 2: optimize costs, with trust region enabled
    m_trustRegion->resetTrustRegion();
    m_trustRegion->setInfinite(false);
    setupOpt2(opt);
    opt.optimize();
*/

    //opt.postOptimize();
    //opt.m_model->write("/tmp/sqp_fail.lp");
    //cout << "step time: " << timer.elapsed();
    //cout << " | physics step cost: " << m_physicsStepCost->evaluate() << '\n';

  } catch (const GRBException &e) {
    LOG_ERROR("Gurobi exception (" << e.getErrorCode() << "): " << e.getMessage());
    throw;
  }

  m_currState = opt.m_sys[1];
  m_currAllStates = opt.m_sys;

  draw();
}


} // namespace ophys