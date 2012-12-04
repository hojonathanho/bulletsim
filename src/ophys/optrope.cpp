#include "optrope.h"

#include "ophys_common.h"
using namespace ophys;

#include <iostream>
using namespace std;

OptRope::OptRope(int T, int N, double linkLen)
  : m_N(N), m_T(T), m_linkLen(linkLen),
    m_costCalls(0),
    m_useRobot(false),
    m_dummyState(this, m_T, m_N),
    m_initPos(MatrixX3d::Zero(N, 3)),
    m_initManipDofs(Vector7d::Zero()),
    m_fkCalls(0)
{
  assert(m_N >= 1);
  assert(m_T >= 1);
  calcBounds();
  setCoeffs1();
}

void OptRope::setInitPositions(const MatrixX3d &initPos) {
  assert(initPos.rows() == m_N);
  m_initPos = initPos;
  calcBounds();
}

void OptRope::setInitManipDofs(const Vector7d &initManipDofs) {
  m_initManipDofs = initManipDofs;
  calcBounds();
}

void OptRope::calcBounds() {
  m_lb.reset(createLowerBound()); m_ub.reset(createUpperBound());
  m_lbvec = m_lb->toColumn(); m_ubvec = m_ub->toColumn();
}

void OptRope::setCoeffs1() {
  m_coeffs.groundPenetration = 1.;
  m_coeffs.velUpdate = 0.1;
  m_coeffs.contact = 0.1;
  m_coeffs.forces = 1;
  m_coeffs.linkLen = 1;
  m_coeffs.goalPos = 1;
  m_coeffs.manipSpeed = 0.001;
  m_coeffs.damping = 0.01;
}

void OptRope::setCoeffs2() {
  m_coeffs.groundPenetration = 1.;
  m_coeffs.velUpdate = 10.;
  m_coeffs.contact = 10;
  m_coeffs.forces = 1;
  m_coeffs.linkLen = 10;
  m_coeffs.goalPos = 1;
  m_coeffs.manipSpeed = 0.001;
  m_coeffs.damping = 0.1;
}

void OptRope::setRobot(RaveRobotObject::Ptr robot, RobotManipulator::Ptr manip) {
  m_robot = robot;
  m_manip = manip;
  m_useRobot = true;
  calcBounds();
}

Vector3d OptRope::calcManipPos(const Vector7d &dofs) {
  Vector3d p;
  if (m_useRobot) {
    //cout << "settign dof values to " << dofs.transpose() << endl;
    //m_manip->setDOFValues(toStlVec(dofs));
    m_robot->robot->SetActiveDOFs(m_manip->manip->GetArmIndices());
    m_robot->robot->SetActiveDOFValues(toStlVec(dofs));
    if (m_fkCalls % 100000 == 0) {
      cout << "forward kinematics calls: " << m_fkCalls << endl;
    }
    ++m_fkCalls;
    //cout << "done." << endl;
    p = toEigVec(m_manip->getTransform().getOrigin());
  } else {
    p = dofs.head<3>();
  }
  return p;
}

static void getDOFLimits(RaveRobotObject::Ptr robot, const vector<int> &indices,
  Vector7d &out_lower, Vector7d &out_upper) {

  vector<double> lb(7), ub(7);
  robot->robot->GetDOFLimits(lb, ub, indices);
  out_lower = toEigVec(lb);
  out_upper = toEigVec(ub);
  cout << "dof limits: " << out_lower.transpose() << " | " << out_upper.transpose() << endl;
}

OptRopeState *OptRope::createLowerBound() {
  OptRopeState *out = new OptRopeState(this, m_T, m_N);
  OptRopeState &lb = *out;

  Vector7d lbDof, ubDof;
  if (m_useRobot) {
    getDOFLimits(m_robot, m_manip->manip->GetArmIndices(), lbDof, ubDof);
  }

  for (int t = 0; t < m_T; ++t) {
    if (t == 0) {
      lb.atTime[t].manipDofs = m_initManipDofs;
    } else {
      if (m_useRobot) {
        lb.atTime[t].manipDofs = lbDof;
      } else {
        lb.atTime[t].manipDofs.setConstant(-infty());
      }
    }
    if (t == 0) lb.atTime[t].x = m_initPos; else lb.atTime[t].x.setConstant(-infty());
    lb.atTime[t].vel.setConstant(t == 0 ? 0 : -infty());
    lb.atTime[t].ropeCntForce_f.setConstant(-infty());
    lb.atTime[t].groundForce_f.setZero();
    lb.atTime[t].groundForce_c.setConstant(0);
    lb.atTime[t].manipForce.setConstant(-infty());
    lb.atTime[t].manipForce_c.setConstant(0);
  }
  return out;
}

OptRopeState *OptRope::createUpperBound() {
  OptRopeState *out = new OptRopeState(this, m_T, m_N);
  OptRopeState &ub = *out;

  Vector7d lbDof, ubDof;
  if (m_useRobot) {
    getDOFLimits(m_robot, m_manip->manip->GetArmIndices(), lbDof, ubDof);
  }

  for (int t = 0; t < m_T; ++t) {
    if (t == 0) {
      ub.atTime[t].manipDofs = m_initManipDofs;
    } else {
      if (m_useRobot) {
        ub.atTime[t].manipDofs = ubDof;
      } else {
        ub.atTime[t].manipDofs.setConstant(infty());
      }
    }
    if (t == 0) ub.atTime[t].x = m_initPos; else ub.atTime[t].x.setConstant(infty());
    ub.atTime[t].vel.setConstant(t == 0 ? 0 : infty());
    ub.atTime[t].ropeCntForce_f.setConstant(infty());
    ub.atTime[t].groundForce_f.setConstant(infty());
    ub.atTime[t].groundForce_c.setConstant(1);
    ub.atTime[t].manipForce.setConstant(infty());
    ub.atTime[t].manipForce_c.setConstant(1);
  }
  return out;
}

OptRopeState OptRope::createInitState() {
  OptRopeState s(this, m_T, m_N);
  for (int t = 0; t < m_T; ++t) {
    s.atTime[t].manipDofs = m_initManipDofs;
    s.atTime[t].x = m_initPos;
    s.atTime[t].groundForce_c.setConstant(0.5);
    s.atTime[t].manipForce_c.setConstant(0.5);
    // everything else should be zero.
  }
  return s;
}


double OptRope::costfunc_wrapper(const Eigen::Map<const VectorXd> &x) {
  OptRopeState state(this, m_T, m_N);
  state.initFromColumn(x);
  return costfunc(state);
}

double OptRope::nlopt_costWrapper(const vector<double> &x, vector<double> &grad) {
  static int calls = 0;
  ++calls;
  if (calls % 1000 == 0)
    cout << "cost evaluations: " << calls;

  Eigen::Map<VectorXd> ex(const_cast<double*>(x.data()), x.size(), 1);
  OptRopeState state(this, m_T, m_N); state.initFromColumn(ex);
  OptRopeState mask(this, m_T, m_N);
  VectorXd maskVal(VectorXd::Zero(x.size()));

  OptRopeState expansion(this, state.getExpandedT(OPhysConfig::interpPerTimestep), m_N);
  expansion.expanded = true;
  state.fillExpansion(OPhysConfig::interpPerTimestep, expansion);

  double val = costfunc_on_expanded(expansion);
  if (calls % 1000 == 0)
    cout << " | val: " << val << endl;

  if (!grad.empty()) {
    static const double eps = 1e-3;

    for (int i = 0; i < ex.size(); ++i) {
#if 1
      double xorig = ex[i];
      maskVal[i] = 1; mask.initFromColumn(maskVal);

      double xb = ex[i] = min(m_ubvec[i], xorig + eps);
      state.initFromColumn(ex);
      state.fillExpansion(OPhysConfig::interpPerTimestep, expansion, &mask);
      double b = costfunc_on_expanded(expansion);

      double xa = ex[i] = max(m_lbvec[i], xorig - eps);
      state.initFromColumn(ex);
      state.fillExpansion(OPhysConfig::interpPerTimestep, expansion, &mask);
      double a = costfunc_on_expanded(expansion);

      ex[i] = xorig;
      maskVal[i] = 0;
#else
      double xorig = ex[i];

      double xb = ex[i] = min(m_ubvec[i], xorig + eps);
      state.initFromColumn(ex);
      state.fillExpansion(OPhysConfig::interpPerTimestep, expansion);
      double b = costfunc_on_expanded(expansion);

      double xa = ex[i] = max(m_lbvec[i], xorig - eps);
      state.initFromColumn(ex);
      state.fillExpansion(OPhysConfig::interpPerTimestep, expansion);
      double a = costfunc_on_expanded(expansion);

      ex[i] = xorig;
#endif
      double xdiff = xb - xa;
      if (xdiff < eps/2.) {
        grad[i] = 0;
      } else {
        grad[i] = (b - a) / xdiff;
      }
    }
  }

  return val;
}

///// Cost function terms /////

inline double OptRope::cost_groundPenetration(const OptRopeState &es) { // es is an expanded state (interpolated)
  // sum of squares of positive parts
  double cost = 0;
  for (int t = 0; t < es.atTime.size(); ++t) {
    for (int n = 0; n < es.m_N; ++n) {
      //if (m_mask.atTime[t].x(n, 2) == 0) continue;
      cost += square(max(0., OPhysConfig::tableHeight - es.atTime[t].x(n,2)));
    }
  }
  return cost;
}

inline double OptRope::cost_velUpdate(const OptRopeState &es) {
  assert(es.expanded);
  double cost = 0;
  for (int t = 0; t < es.atTime.size() - 1; ++t) {
    for (int n = 0; n < es.m_N; ++n) {
      // compute total force on particle n at time t
      Vector3d gravity(0, 0, OPhysConfig::gravity);
      Vector3d groundForce(0, 0, es.atTime[t].groundForce_c[n] * es.atTime[t].groundForce_f[n]);
      Vector3d manipForce = es.atTime[t].manipForce_c[n] * es.atTime[t].manipForce.row(n).transpose(); //(state.atTime[t].manipDofs - pos[t].row(n).transpose()).normalized() * state.atTime[t].manipForce_f[n];

      Vector3d ropeCntForce(0, 0, 0);
      if (n == 0) {
        ropeCntForce = es.atTime[t].ropeCntForce_f[n] * (es.atTime[t].x.row(n+1) - es.atTime[t].x.row(n)).normalized().transpose();
      } else if (n == es.m_N - 1) {
        ropeCntForce = es.atTime[t].ropeCntForce_f[n-1] * (es.atTime[t].x.row(n-1) - es.atTime[t].x.row(n)).normalized().transpose();
      } else {
        ropeCntForce =
          es.atTime[t].ropeCntForce_f[n] * (es.atTime[t].x.row(n+1) - es.atTime[t].x.row(n)).normalized().transpose() +
          es.atTime[t].ropeCntForce_f[n-1] * (es.atTime[t].x.row(n-1) - es.atTime[t].x.row(n)).normalized().transpose();
      }

      Vector3d totalForce = gravity + groundForce + manipForce + ropeCntForce;
      Vector3d accel = es.atTime[t].derived_accel.row(n).transpose();
      cost += (accel - totalForce /* / mass */).squaredNorm();
    }
  }
  return cost;
}

inline double OptRope::cost_contact(const OptRopeState &es) {
  double cost = 0;
  // ground force: groundContact^2 * (ground non-violation)^2
  for (int t = 0; t < es.atTime.size(); ++t) {
    //Vector3d manipPos = calcManipPos(es.atTime[t].manipDofs);
    Vector3d manipPos = es.atTime[t].derived_manipPos;
    for (int n = 0; n < es.m_N; ++n) {
      // ground contact
      cost += es.atTime[t].groundForce_c[n] * square(max(0., es.atTime[t].x(n,2) - OPhysConfig::tableHeight));

      // manipulator contact
      cost += es.atTime[t].manipForce_c[n] * (manipPos - es.atTime[t].x.row(n).transpose()).squaredNorm();

      // not really a contact, but you get the idea
      // if (n < es.m_N - 1) {
      //   double linkLenViolation = square((es.atTime[t].x.row(n) - es.atTime[t].x.row(n+1)).squaredNorm() - square(m_linkLen));
      //   cost += square(es.atTime[t].ropeCntForce_f[n]) / (1e-6 + linkLenViolation);
      // }
    }

    cost += 1e-3*es.atTime[t].manipForce_c.sum(); // sparse
    cost += square(max(0., es.atTime[t].manipForce_c.sum() - 1.)); // total manip contact should be <= 1
  }
  return cost;
}

// TODO: use f_total = sum(c*f), and get rid of f/c costs.
inline double OptRope::cost_forces(const OptRopeState &es) {
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

      if (n < es.m_N - 1) {
        cost += square(es.atTime[t].ropeCntForce_f[n]);
      }
    }
  }
  return cost;
}

// rope segment distance violation cost
inline double OptRope::cost_linkLen(const OptRopeState &es) {
  double cost = 0;
  for (int t = 0; t < es.atTime.size(); ++t) {
    for (int n = 0; n < es.m_N - 1; ++n) {
      double dist = (es.atTime[t].x.row(n) - es.atTime[t].x.row(n+1)).norm();
      cost += square(dist - m_linkLen);
    }
  }
  return cost;
}

// goal cost: point 0 should be at (0, 0, 1)
inline double OptRope::cost_goalPos(const OptRopeState &es, const Vector3d &goal) {
  return (es.atTime.back().x.row(0).transpose() - goal).squaredNorm();
}

inline double OptRope::cost_manipSpeed(const OptRopeState &es) {
  double cost = 0;
  for (int t = 0; t < es.atTime.size() - 1; ++t) {
    cost += (es.atTime[t+1].manipDofs - es.atTime[t].manipDofs).squaredNorm();
  }
  return cost;
}

inline double OptRope::cost_damping(const OptRopeState &es) {
  double cost = 0;
  for (int t = 0; t < es.atTime.size(); ++t) {
    cost += es.atTime[t].vel.squaredNorm() + es.atTime[t].derived_accel.squaredNorm();
  }
  return cost;
}

double OptRope::costfunc_on_expanded(const OptRopeState &expandedState) {
  assert(expandedState.expanded);
  double cost = 0;
  cost += m_coeffs.groundPenetration * cost_groundPenetration(expandedState);
  cost += m_coeffs.velUpdate * cost_velUpdate(expandedState);
  cost += m_coeffs.contact * cost_contact(expandedState);
  cost += m_coeffs.forces * cost_forces(expandedState);
  cost += m_coeffs.linkLen * cost_linkLen(expandedState);
  cost += m_coeffs.goalPos * cost_goalPos(expandedState, m_initPos.row(0).transpose() + Vector3d(0, 0, 1));
  cost += m_coeffs.manipSpeed * cost_manipSpeed(expandedState);
  cost += m_coeffs.damping * cost_damping(expandedState);
  return cost;
}

double OptRope::costfunc(const OptRopeState &state) {
  assert(!state.expanded);
  // expansion is very expensive!
  return costfunc_on_expanded(state.expandByInterp(OPhysConfig::interpPerTimestep));
}
