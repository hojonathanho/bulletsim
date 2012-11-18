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

  for (int t = 0; t < m_horizon; ++t) {
    for (int i = 0; i < m_numParticles; ++i) {
      for (int j = 0; j < PARTICLE_STATE_DIM; ++j) {
        if (isContactAuxVar(j)) {
          m_varsys[t][i][j].set(GRB_DoubleAttr_LB, 0);
          m_varsys[t][i][j].set(GRB_DoubleAttr_UB, 1);
        }
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
  for (int i = 0; i < m_costs.size(); ++i) {
    cout << "cost " << m_costs[i]->getName() << " has value: " << m_costs[i]->evaluate() << '\n';
  }
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
        if (!m_opt.isContactAuxVar(j)) {
          m_opt.m_varsys[t][i][j].set(GRB_DoubleAttr_LB, m_infinite ? -GRB_INFINITY : m_opt.m_sys[t](i,j) - m_radii[t][i][j]);
          m_opt.m_varsys[t][i][j].set(GRB_DoubleAttr_UB, m_infinite ? GRB_INFINITY : m_opt.m_sys[t](i,j) + m_radii[t][i][j]);
        }
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
      out->m_exprs.push_back(-m_opt.varPtPos(t, i)[2] + m_groundZ);
    }
  }
  return out;
}


GRBLinExpr ParticleSystemOptimizer2::linearize(const FuncOnStatesOverTime &fn, SysStatesOverTime &x0) {
  const double eps = 1e-4;
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
  const double eps = 1e-4;
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

MatrixXd ParticleSystemOptimizer2::hessian(const FuncOnStatesOverTime &f, SysStatesOverTime &x0) {
  /*
function h = hessian(f, x, eps)

nx = length(x);
e = eye(nx);
h = zeros(nx);

for j = 1:nx
  dxj = e(:,j) * eps;
  for i = 1:nx
    dxi = e(:,i) * eps;
    f_xj_1 = (f(x-dxi+dxj) - f(x-dxi-dxj))/(2*eps);
    f_xj_2 = (f(x+dxi+dxj) - f(x+dxi-dxj))/(2*eps);
    h(i, j) = (f_xj_2 - f_xj_1)/(2*eps);
  end
end

end*/

  static const double eps = 1e-4;
  const int dim = m_horizon*m_numParticles*PARTICLE_STATE_DIM;
  MatrixXd H = MatrixXd::Zero(dim, dim);

  for (int tx = 0; tx < m_horizon; ++tx) { for (int ix = 0; ix < m_numParticles; ++ix) { for (int jx = 0; jx < PARTICLE_STATE_DIM; ++jx) {
    const int row = m_numParticles*PARTICLE_STATE_DIM*tx + PARTICLE_STATE_DIM*ix + jx;
    for (int ty = 0; ty < m_horizon; ++ty) { for (int iy = 0; iy < m_numParticles; ++iy) { for (int jy = 0; jy < PARTICLE_STATE_DIM; ++jy) {
      const int col = m_numParticles*PARTICLE_STATE_DIM*ty + PARTICLE_STATE_DIM*iy + jy;

      double x_orig = x0[tx](ix, jx), y_orig = x0[ty](iy, jy);

      x0[tx](ix, jx) -= eps; x0[ty](iy, jy) += eps;
      double fnp = f(&x0);
      x0[tx](ix, jx) = x_orig; x0[ty](iy, jy) = y_orig;

      x0[tx](ix, jx) -= eps; x0[ty](iy, jy) -= eps;
      double fnn = f(&x0);
      x0[tx](ix, jx) = x_orig; x0[ty](iy, jy) = y_orig;

      x0[tx](ix, jx) += eps; x0[ty](iy, jy) += eps;
      double fpp = f(&x0);
      x0[tx](ix, jx) = x_orig; x0[ty](iy, jy) = y_orig;

      x0[tx](ix, jx) += eps; x0[ty](iy, jy) -= eps;
      double fpn = f(&x0);
      x0[tx](ix, jx) = x_orig; x0[ty](iy, jy) = y_orig;

      H(row,col) = ((fnp - fnn) - (fpp - fpn))/(4*eps*eps);
    }}}
  }}}

  return H;
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

ConvexConstraintPtr ParticleSystemOptimizer2::PosUpdateConstraint::convexify(GRBModel* ) {
  ConvexConstraintPtr out(new ConvexConstraint());
  out->m_name = "pos_update_constraint";
  boost::format fmt("%s_pos_update_constraint_t%d_i%d_%d");
  for (int t = 0; t < m_opt.m_horizon - 1; ++t) {
    for (int i = 0; i < m_opt.m_numParticles; ++i) {
      for (int j = 0; j < 3; ++j) {
        out->m_eqcntNames.push_back((fmt % m_opt.m_varPrefix % t % i % j).str());
        out->m_eqexprs.push_back(m_opt.varPtPos(t+1,i)[j] - (m_opt.varPtPos(t,i)[j] + OPhysConfig::dt*m_opt.varPtVel(t,i)[j]));
      }
    }
  }
  return out;
}

double ParticleSystemOptimizer2::PhysicsCost::evaluateAt(const SysStatesOverTime *s0) const {
  double cost = 0;
  static const double LAMBDA = 0.1;
  const SysStatesOverTime &sys = *s0;
  for (int t = 0; t < m_opt.m_horizon - 1; ++t) {
    for (int i = 0; i < m_opt.m_numParticles; ++i) {
      //Vector3d fapplied = m_opt.ptGroundCont(sys[t],i) * m_opt.ptGroundContForce(sys[t],i);
      Vector3d ftotal = /*fapplied +*/ METERS*OPhysConfig::gravity;
      Vector3d dp = /* mass * */ (m_opt.ptVel(sys[t+1],i) - m_opt.ptVel(sys[t],i))/OPhysConfig::dt;
      cost += (ftotal - dp).squaredNorm();
      //cost += LAMBDA*m_opt.ptGroundContForce(sys[t],i).squaredNorm();
    }
  }
  return cost;
}

double ParticleSystemOptimizer2::PhysicsCost::evaluate() {
  return evaluateAt(&m_opt.m_sys);
}

ConvexObjectivePtr ParticleSystemOptimizer2::PhysicsCost::convexify(GRBModel *model) {
  ConvexObjectivePtr out(new ConvexObjective());
  out->m_name = "physics_cost";
  //out->m_objective = m_opt.linearize(boost::bind(&PhysicsCost::evaluateAt, this, _1), m_opt.m_sys);
  
  /*out->m_objective = 0;
  for (int t = 0; t < m_opt.m_horizon - 1; ++t) {
    for (int i = 0; i < m_opt.m_numParticles; ++i) {
      for (int j = 0; j < 3; ++j) {
        out->m_objective += square(METERS*OPhysConfig::gravity[j] - (m_opt.varPtVel(t+1,i)[j] - m_opt.varPtVel(t,i)[j])/OPhysConfig::dt);
      }
    }
  }*/

  double y = evaluate();
  out->m_objective = y;
  const double eps = 1e-4;
  for (int t = 0; t < m_opt.m_horizon - 1; ++t) {
    for (int i = 0; i < m_opt.m_numParticles; ++i) {
      for (int j = 0; j < m_opt.PARTICLE_STATE_DIM; ++j) {
        double bkp = m_opt.m_sys[t](i,j);
        m_opt.m_sys[t](i,j) -= eps/2.;
        double yminus = evaluate();
        m_opt.m_sys[t](i,j) += eps;
        double yplus = evaluate();
        m_opt.m_sys[t](i,j) = bkp;
        //cout << yplus << ' ' << yminus << endl;
        double yp = (yplus - yminus)/eps;

        GRBLinExpr dx = m_opt.m_varsys[t][i][j] - bkp;
        out->m_objective +=  yp*dx;
      }
    }
  }

  MatrixXd H = m_opt.hessian(boost::bind(&PhysicsCost::evaluateAt, this, _1), m_opt.m_sys);
  const int dim = m_opt.m_horizon*m_opt.m_numParticles*PARTICLE_STATE_DIM;
  for (int tx = 0; tx < m_opt.m_horizon; ++tx) { for (int ix = 0; ix < m_opt.m_numParticles; ++ix) { for (int jx = 0; jx < m_opt.PARTICLE_STATE_DIM; ++jx) {
    const int row = m_opt.m_numParticles*PARTICLE_STATE_DIM*tx + m_opt.PARTICLE_STATE_DIM*ix + jx;
    for (int ty = 0; ty < m_opt.m_horizon; ++ty) { for (int iy = 0; iy < m_opt.m_numParticles; ++iy) { for (int jy = 0; jy < m_opt.PARTICLE_STATE_DIM; ++jy) {
      const int col = m_opt.m_numParticles*PARTICLE_STATE_DIM*ty + m_opt.PARTICLE_STATE_DIM*iy + jy;
      GRBLinExpr dxx = m_opt.m_varsys[tx][ix][jx] - m_opt.m_sys[tx](ix,jx);
      GRBLinExpr dxy = m_opt.m_varsys[ty][iy][jy] - m_opt.m_sys[ty](iy,jy);
      out->m_objective += H(row, col) * dxx * dxy;
    }}}
  }}}

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

  //m_groundCost.reset(new ParticleSystemOptimizer2::GroundCost(opt, TABLE_HEIGHT*METERS));
  //opt.addCost(m_groundCost);

  m_posUpdateCnt.reset(new ParticleSystemOptimizer2::PosUpdateConstraint(opt));
  opt.addConstraint(m_posUpdateCnt);

  m_physicsCost.reset(new ParticleSystemOptimizer2::PhysicsCost(opt));
  opt.addCost(m_physicsCost);
}

void ParticleSystem2::setupOpt2(ParticleSystemOptimizer2 &opt) {
  //m_pdPosViolationCost->setGroundViolationPenalty(1.0);
}

void ParticleSystem2::step(double dt, int numSteps) {
  assert(numSteps >= 1);

  ParticleSystemOptimizer2 opt(m_currState.rows(), numSteps + 1);
  try {
    opt.initializeFromSingleState(m_currState);
    m_trustRegion.reset(new ParticleSysTrustRegion(opt));
    opt.setTrustRegion(m_trustRegion);
    setupOpt(opt);
    setupOpt2(opt);
    opt.optimize();

#if 0
    opt.initializeFromSingleState(m_currState);

    // phase 1: find a nominal solution that satisfies all hard NON-APPROXIMATED constraints
    // (need to disable trust region constraints)
    m_trustRegion.reset(new ParticleSysTrustRegion(opt));
    opt.setTrustRegion(m_trustRegion);
    m_trustRegion->setInfinite(true);
    setupOpt(opt);
    opt.optimize();

    cout << "phase 2" << endl;

    // phase 2: optimize costs, with trust region enabled
    m_trustRegion->setInfinite(false);
    //setupOpt2(opt);
    for (int z = 1; z < 2; ++z) {
      m_trustRegion->resetTrustRegion();
      m_pdPosViolationCost->setGroundViolationPenalty(pow(10.0, (double)z));
      opt.optimize();
    }
    //opt.postOptimize();
    //opt.m_model->write("/tmp/sqp_fail.lp");
    //cout << "step time: " << timer.elapsed();
    //cout << " | physics step cost: " << m_physicsStepCost->evaluate() << '\n';
#endif
  } catch (const GRBException &e) {
    LOG_ERROR("Gurobi exception (" << e.getErrorCode() << "): " << e.getMessage());
    throw;
  }

  m_currState = opt.m_sys[1];
  m_currAllStates = opt.m_sys;

  draw();
}


} // namespace ophys