#include "particlesystem.h"

#include <boost/format.hpp>

#include "ophys_common.h"
#include "utils/config.h"
#include "utils/logging.h"

#include "simulation/simplescene.h"
#include "simulation/plotting.h"

namespace ophys {

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
  m_name = "initial_condition_constraints";
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
  m_name = "no_external_forces_constraint";
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

GroundConstraint::GroundConstraint(ParticleSystemOptimizer &o, double groundZ) {
  m_name = "ground_constraint";
  boost::format fmt("%s_ground_t%d_i%d");
  for (int t = 1; t < o.m_horizon; ++t) { // start at 1
    for (int i = 0; i < o.m_numParticles; ++i) {
      m_cntNames.push_back((fmt % o.m_varPrefix % t % i).str());
      m_exprs.push_back(-o.m_sys[t][i].var_x[2] + groundZ);
    }
  }
}

PointDistanceCost::PointDistanceCost(ParticleSystemOptimizer &opt, int p, int q, double d)
  : m_opt(opt), m_p(p), m_q(q), m_d(d) {
  assert(0 <= m_p && m_p < m_opt.m_numParticles);
  assert(0 <= m_q && m_q < m_opt.m_numParticles);
  assert(0 <= m_d);
}

string PointDistanceCost::getName() {
  return (boost::format("point_distance_%d_%d") % m_p % m_q).str();
}

double PointDistanceCost::evaluate() {
  /*
  double cost = 0;
  for (int t = 0; t < m_opt.m_horizon; ++t) {
    cost += square((m_opt.m_sys[t][m_p].x - m_opt.m_sys[t][m_q].x).norm() - m_d);
  }
  return cost;
  */
  return evaluate(getCurrPointMatrix());
}

MatrixX3d PointDistanceCost::getCurrPointMatrix() const {
  MatrixX3d points(m_opt.m_horizon*2, 3); // p/q points interleaved over time
  for (int t = 0; t < m_opt.m_horizon; ++t) {
    points.row(2*t  ) = m_opt.m_sys[t][m_p].x.transpose();
    points.row(2*t+1) = m_opt.m_sys[t][m_q].x.transpose();
  }
  return points;
}

double PointDistanceCost::evaluate(const MatrixX3d &points) {
  // points is p/q points interleaved over time
  // i.e. p(t0), q(t0), p(t1), q(t1), ...
  assert(points.rows() == 2*m_opt.m_horizon);
  double cost = 0;
  for (int i = 0; i < points.rows(); i += 2) {
    cost += square((points.row(i) - points.row(i+1)).norm() - m_d);
  }
  return cost;
}

ConvexObjectivePtr PointDistanceCost::convexify(GRBModel *model) {
  const double eps = 1e-6;

  ConvexObjectivePtr out(new ConvexObjective());
  out->m_name = getName();

  double y = evaluate();
  out->m_objective = y;

  MatrixX3d points = getCurrPointMatrix();
  for (int i = 0; i < points.rows(); ++i) {
    for (int j = 0; j < points.cols(); ++j) {
      points(i,j) -= eps/2.;
      double yminus = evaluate(points);
      points(i,j) += eps;
      double yplus = evaluate(points);
      points(i,j) -= eps/2.;
      double yp = (yplus - yminus)/eps;
      double ypp = (yplus + yminus - 2*y)/(eps*eps/4.);
      GRBLinExpr dx = m_opt.m_sys[i/2][i % 2 == 0 ? m_p : m_q].var_x[j] - points(i,j);
      out->m_objective += 0.5*ypp*dx*dx + yp*dx;
    }
  }

  return out;
}

PointAnchorCost::PointAnchorCost(ParticleSystemOptimizer &opt, int p, const Vector3d &anchorpt)
  : m_opt(opt), m_p(p), m_anchorpt(anchorpt) {
  assert(0 <= m_p && m_p < opt.m_numParticles);
}

string PointAnchorCost::getName() {
  return (boost::format("point_anchor_%d") % m_p).str();
}

MatrixX3d PointAnchorCost::getCurrPointMatrix() const {
  MatrixX3d points(m_opt.m_horizon, 3);
  for (int t = 0; t < m_opt.m_horizon; ++t) {
    points.row(t) = m_opt.m_sys[t][m_p].x.transpose();
  }
  return points;
}

double PointAnchorCost::evaluate() {
  return evaluate(getCurrPointMatrix());
}

double PointAnchorCost::evaluate(const MatrixX3d &pointOverTime) {
  double cost = 0;
  assert(pointOverTime.rows() == m_opt.m_horizon);
  for (int t = 0; t < m_opt.m_horizon; ++t) {
    cost += (m_anchorpt.transpose() - pointOverTime.row(t)).squaredNorm();
  }
  return cost;
}

ConvexObjectivePtr PointAnchorCost::convexify(GRBModel *model) {
  const double eps = 1e-6;

  ConvexObjectivePtr out(new ConvexObjective());
  out->m_name = getName();

  double y = evaluate();
  out->m_objective = y;

  MatrixX3d points = getCurrPointMatrix();
  for (int i = 0; i < points.rows(); ++i) {
    for (int j = 0; j < points.cols(); ++j) {
      points(i,j) -= eps/2.;
      double yminus = evaluate(points);
      points(i,j) += eps;
      double yplus = evaluate(points);
      points(i,j) -= eps/2.;
      double yp = (yplus - yminus)/eps;
      double ypp = (yplus + yminus - 2*y)/(eps*eps/4.);
      GRBLinExpr dx = m_opt.m_sys[i][m_p].var_x[j] - points(i,j);
      out->m_objective += 0.5*ypp*dx*dx + yp*dx;
    }
  }

  return out;
}


double PhysicsStepCost::evaluate() {
  const double dt = OPhysConfig::dt;
  double cost = 0;
  for (int t = 0; t < m_opt.m_horizon - 1; ++t) {
    for (int i = 0; i < m_opt.m_numParticles; ++i) {
      for (int j = 0; j < 3; ++j) {
        // position step cost
        cost += square(m_opt.m_sys[t+1][i].x[j] - (m_opt.m_sys[t][i].x[j] + dt*m_opt.m_sys[t][i].v[j]));
        // velocity step cost
        cost += square(m_opt.m_sys[t+1][i].v[j] - (m_opt.m_sys[t][i].v[j] + dt*(m_opt.m_sys[t][i].a[j] + METERS*OPhysConfig::gravity[j])));
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
  m_currState = initState;
  m_plotSpheres.reset(new PlotSpheres());
}

void ParticleSystem::step(double dt) {
  // update initial condition (start simulating from current state)
  ParticleSystemOptimizer opt(m_currState.size(), 2);
  try {
    setupOpt(opt);
    if (m_preOptCallback) {
      m_preOptCallback(&opt);
    }
    opt.optimize();
  } catch (const GRBException &e) {
    LOG_ERROR("Gurobi exception (" << e.getErrorCode() << "): " << e.getMessage());
  }

  m_currState = opt.m_sys[1];
  draw();
}

void ConvexConstraintWrapper::AddToOpt(Optimizer &opt, ConvexConstraintPtr cnt) {
  ConvexConstraintWrapper::Ptr wrapper(new ConvexConstraintWrapper(cnt));
  opt.addConstraint(wrapper);
}

void ParticleSystem::setupOpt(ParticleSystemOptimizer &opt) {
  opt.initializeFromSingleState(m_currState);

  ParticleSystemTrustRegion::Ptr trustRegion(new ParticleSystemTrustRegion(opt));
  opt.setTrustRegion(trustRegion);

  PhysicsStepCost::Ptr physicsStepCost(new PhysicsStepCost(opt));
  opt.addCost(physicsStepCost);

  InitialConditionConstraints::Ptr initCondCnt(new InitialConditionConstraints(opt, m_currState));
  ConvexConstraintWrapper::AddToOpt(opt, initCondCnt);

  NoExternalForcesConstraint::Ptr noExtForcesCnt(new NoExternalForcesConstraint(opt));
  ConvexConstraintWrapper::AddToOpt(opt, noExtForcesCnt);

  GroundConstraint::Ptr groundCnt(new GroundConstraint(opt, 0.01*METERS));
  ConvexConstraintWrapper::AddToOpt(opt, groundCnt);
}

void ParticleSystem::step() {
  step(OPhysConfig::dt);
}

void ParticleSystem::attachToScene(Scene *scene) {
  m_scene = scene;
  m_scene->env->add(m_plotSpheres);
  draw();
}

void ParticleSystem::draw() {
  for (int i = 0; i < m_currState.size(); ++i) {
    LOG_INFO("x(" << i << "): " << m_currState[i].x.transpose());
    LOG_INFO("v(" << i << "): " << m_currState[i].v.transpose());
    LOG_INFO("a(" << i << "): " << m_currState[i].a.transpose());
  }

  osg::ref_ptr<osg::Vec3Array> centers(new osg::Vec3Array());
  osg::ref_ptr<osg::Vec4Array> rgba(new osg::Vec4Array());
  vector<float> radii;
  for (int i = 0; i < m_currState.size(); ++i) {
    centers->push_back(osg::Vec3(m_currState[i].x[0], m_currState[i].x[1], m_currState[i].x[2]));
    rgba->push_back(osg::Vec4(1, 0, 0, 1));
    radii.push_back(0.05);
  }
  m_plotSpheres->plot(centers, rgba, radii);
}


void ParticleSystem::setPreOptCallback(PreOptCallback cb) {
  m_preOptCallback = cb;
}


RopeSystem::RopeSystem(const ParticleSystemState &initState, double segrlen)
  : m_segrlen(segrlen), ParticleSystem(initState) { }

void RopeSystem::setupOpt(ParticleSystemOptimizer &opt) {
  ParticleSystem::setupOpt(opt);

  for (int p = 0; p < opt.m_numParticles - 1; ++p) {
    PointDistanceCost::Ptr distCost(new PointDistanceCost(opt, p, p+1, m_segrlen));
    opt.addCost(distCost);
  }
}

} // namespace ophys
