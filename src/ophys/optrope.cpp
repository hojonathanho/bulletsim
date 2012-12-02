#include "optrope.h"

#include "ophys_common.h"
using namespace ophys;

#include <iostream>
using namespace std;

OptRope::OptRope(const MatrixX3d &initPos, const Vector3d &initManipPos, int T, int N, double linkLen)
  : m_N(N), m_T(T), m_initPos(initPos), m_initManipPos(initManipPos), m_linkLen(linkLen),
  	m_lb(createLowerBound()), m_ub(createUpperBound()),
  	m_lbvec(createLowerBound().toColumn()), m_ubvec(createUpperBound().toColumn()),
    m_costCalls(0)
{
  assert(m_N >= 1);
  assert(m_T >= 1);
  assert(initPos.rows() == m_N);
  setCoeffs1();
}

void OptRope::setCoeffs1() {
  m_coeffs.groundPenetration = 1.;
  m_coeffs.velUpdate = 0.1;
  m_coeffs.contact = 0.1;
  m_coeffs.forces = 1;
  m_coeffs.linkLen = 0.1;
  m_coeffs.goalPos = 1;
  m_coeffs.manipSpeed = 0.001;
}

void OptRope::setCoeffs2() {
  m_coeffs.groundPenetration = 1.;
  m_coeffs.velUpdate = 10.;
  m_coeffs.contact = 10;
  m_coeffs.forces = 1;
  m_coeffs.linkLen = 1;
  m_coeffs.goalPos = 1;
  m_coeffs.manipSpeed = 0.001;
}

OptRopeState OptRope::createLowerBound() const {
  OptRopeState lb(m_T, m_N);
  for (int t = 0; t < m_T; ++t) {
    if (t == 0) lb.atTime[t].manipPos = m_initManipPos; else lb.atTime[t].manipPos.setConstant(-infty());
    if (t == 0) lb.atTime[t].x = m_initPos; else lb.atTime[t].x.setConstant(-infty());
    lb.atTime[t].vel.setConstant(t == 0 ? 0 : -infty());
    lb.atTime[t].groundForce_f.setZero();
    lb.atTime[t].groundForce_c.setConstant(0);
    lb.atTime[t].manipForce.setConstant(-infty());
    lb.atTime[t].manipForce_c.setConstant(0);
  }
  return lb;
}

OptRopeState OptRope::createUpperBound() const {
  OptRopeState ub(m_T, m_N);
  for (int t = 0; t < m_T; ++t) {
    if (t == 0) ub.atTime[t].manipPos = m_initManipPos; else ub.atTime[t].manipPos.setConstant(infty());
    if (t == 0) ub.atTime[t].x = m_initPos; else ub.atTime[t].x.setConstant(infty());
    ub.atTime[t].vel.setConstant(t == 0 ? 0 : infty());
    ub.atTime[t].groundForce_f.setConstant(infty());
    ub.atTime[t].groundForce_c.setConstant(1);
    ub.atTime[t].manipForce.setConstant(infty());
    ub.atTime[t].manipForce_c.setConstant(1);
  }
  return ub;
}

OptRopeState OptRope::createInitState() const {
  OptRopeState s(m_T, m_N);
  for (int t = 0; t < m_T; ++t) {
    s.atTime[t].manipPos = m_initManipPos;
    s.atTime[t].x = m_initPos;
    s.atTime[t].groundForce_c.setConstant(0.5);
    s.atTime[t].manipForce_c.setConstant(0.5);
    // everything else should be zero.
  }
  return s;
}


double OptRope::costfunc_wrapper(const Eigen::Map<const VectorXd> &x) {
  OptRopeState state(m_T, m_N);
  state.initFromColumn(x);
  return costfunc(state);
}

double OptRope::nlopt_costWrapper(const vector<double> &x, vector<double> &grad) const {
  static int calls = 0;
  ++calls;
  if (calls % 1000 == 0)
    cout << "cost evaluations: " << calls;

  Eigen::Map<VectorXd> ex(const_cast<double*>(x.data()), x.size(), 1);
  OptRopeState state(m_T, m_N); state.initFromColumn(ex);

  OptRopeState expansion(state.getExpandedT(OPhysConfig::interpPerTimestep), m_N);
  expansion.expanded = true;
  state.fillExpansion(OPhysConfig::interpPerTimestep, expansion);

  double val = costfunc_on_expanded(expansion);
  if (calls % 1000 == 0)
    cout << " | val: " << val << endl;

  if (!grad.empty()) {
    static const double eps = 1e-3;

    for (int i = 0; i < ex.size(); ++i) {
      double xorig = ex[i];

      ex[i] = xorig + eps;
      state.initFromColumn(ex);
      state.fillExpansion(OPhysConfig::interpPerTimestep, expansion);
      double b = costfunc_on_expanded(expansion);

      ex[i] = xorig - eps;
      state.initFromColumn(ex);
      state.fillExpansion(OPhysConfig::interpPerTimestep, expansion);
      double a = costfunc_on_expanded(expansion);

      ex[i] = xorig;
      grad[i] = (b - a) / (2*eps);
    }
  }

  return val;
}

///// Cost function terms /////
static double GROUND_HEIGHT = 0;

static inline double cost_groundPenetration(const OptRopeState &es) { // es is an expanded state (interpolated)
  // sum of squares of positive parts
  double cost = 0;
  for (int t = 0; t < es.atTime.size(); ++t) {
    for (int n = 0; n < es.m_N; ++n) {
      cost += square(max(0., GROUND_HEIGHT - es.atTime[t].x(n,2)));
    }
  }
  return cost;
}

static inline double cost_velUpdate(const OptRopeState &es) {
  assert(es.expanded);
  double cost = 0;
  for (int t = 0; t < es.atTime.size() - 1; ++t) {
    for (int n = 0; n < es.m_N; ++n) {
      // compute total force on particle n at time t
      Vector3d gravity(0, 0, OPhysConfig::gravity);
      Vector3d groundForce(0, 0, es.atTime[t].groundForce_c[n] * es.atTime[t].groundForce_f[n]);
      Vector3d manipForce = es.atTime[t].manipForce_c[n] * es.atTime[t].manipForce.row(n).transpose(); //(state.atTime[t].manipPos - pos[t].row(n).transpose()).normalized() * state.atTime[t].manipForce_f[n];
      Vector3d totalForce = gravity + groundForce + manipForce;
      Vector3d accel = es.atTime[t].derived_accel.row(n).transpose();
      cost += (accel - totalForce /* / mass */).squaredNorm();
    }
  }
  return cost;
}

static inline double cost_contact(const OptRopeState &es) {
  double cost = 0;
  // ground force: groundContact^2 * (ground non-violation)^2
  for (int t = 0; t < es.atTime.size(); ++t) {
    for (int n = 0; n < es.m_N; ++n) {
      // ground contact
      cost += es.atTime[t].groundForce_c[n] * square(max(0., es.atTime[t].x(n,2) - GROUND_HEIGHT));

      // manipulator contact
      cost += es.atTime[t].manipForce_c[n] * (es.atTime[t].manipPos - es.atTime[t].x.row(n).transpose()).squaredNorm();
    }
  }
  return cost;
}

// TODO: use f_total = sum(c*f), and get rid of f/c costs.
static inline double cost_forces(const OptRopeState &es) {
  double cost = 0;
  for (int t = 0; t < es.atTime.size(); ++t) {
    for (int n = 0; n < es.m_N; ++n) {
      // ground force (force^2 / contact^2) + force^2
      //cost += square(es.atTime[t].groundForce_f[n]) / (1e-5 + square(es.atTime[t].groundForce_c[n]));
      cost += 1e-6*square(es.atTime[t].groundForce_f[n]);

      // manipulator force
      double manip_fsqnorm = es.atTime[t].manipForce.row(n).squaredNorm();
      //cost += manip_fsqnorm / (1e-5 + square(es.atTime[t].manipForce_c[n]));
      cost += 1e-6*manip_fsqnorm;
    }
  }
  return cost;
}

// rope segment distance violation cost
static inline double cost_linkLen(const OptRopeState &es, double linkLen) {
  double cost = 0;
  for (int t = 0; t < es.atTime.size(); ++t) {
    for (int n = 0; n < es.m_N - 1; ++n) {
      double dist = (es.atTime[t].x.row(n) - es.atTime[t].x.row(n+1)).norm();
      cost += square(dist - linkLen);
    }
  }
  return cost;
}

// goal cost: point 0 should be at (0, 0, 1)
static inline double cost_goalPos(const OptRopeState &es) {
  static const Vector3d desiredPos0(0, 0, 1);
  return (es.atTime.back().x.row(0).transpose() - desiredPos0).squaredNorm();
}

static inline double cost_manipSpeed(const OptRopeState &es) {
  double cost = 0;
  for (int t = 0; t < es.atTime.size() - 1; ++t) {
    cost += (es.atTime[t+1].manipPos - es.atTime[t].manipPos).squaredNorm();
  }
  return cost;
}

double OptRope::costfunc_on_expanded(const OptRopeState &expandedState) const {
  assert(expandedState.expanded);
  double cost = 0;
  cost += m_coeffs.groundPenetration * cost_groundPenetration(expandedState);
  cost += m_coeffs.velUpdate * cost_velUpdate(expandedState);
  cost += m_coeffs.contact * cost_contact(expandedState);
  cost += m_coeffs.forces * cost_forces(expandedState);
  cost += m_coeffs.linkLen * cost_linkLen(expandedState, m_linkLen);
  cost += m_coeffs.goalPos * cost_goalPos(expandedState);
  cost += m_coeffs.manipSpeed * cost_manipSpeed(expandedState);
  return cost;
}

double OptRope::costfunc(const OptRopeState &state) const {
  assert(!state.expanded);
  // expansion is very expensive!
  return costfunc_on_expanded(state.expandByInterp(OPhysConfig::interpPerTimestep));
}
