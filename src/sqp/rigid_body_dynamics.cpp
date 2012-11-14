#include "rigid_body_dynamics.h"
#include "simulation/config_bullet.h"
#include <boost/format.hpp>
#include <boost/assign/list_of.hpp>
#include "utils_sqp.h"
#include "expr_ops.h"
#include "utils/logging.h"

using namespace std;
using namespace boost::assign;

RigidBody::RigidBody(btRigidBody* body, const string& name) :
  m_rb(body),
  m_name(name),
  m_x0(toVector3d(body->getCenterOfMassTransform().getOrigin())),
  m_q0(toVector4d(body->getCenterOfMassTransform().getRotation())),
  m_v0(Vector3d::Zero()),
  m_w0(Vector3d::Zero()),
  m_x1_val(m_x0),
  m_q1_val(m_q0),
  m_v1_val(m_v0),
  m_w1_val(m_w0)
{
}

void RigidBody::addToModel(GRBModel* model) {
  for (int i=0; i < 3; ++i) {
    m_x1_var.push_back(model->addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, (boost::format("%s_x_%i")%m_name%i).str()));
    m_v1_var.push_back(model->addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, (boost::format("%s_v_%i")%m_name%i).str()));
    m_r1_var.push_back(model->addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, (boost::format("%s_dq_%i")%m_name%i).str()));
    m_w1_var.push_back(model->addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, (boost::format("%s_w_%i")%m_name%i).str()));
  }
  model->update();
}

void removeVariables(GRBModel* model, const VarVector& v) {
  for (int i=0; i < v.size(); ++i) model->remove(v[i]);
}

void RigidBody::removeFromModel(GRBModel* model) {
  removeVariables(model, m_x1_var);
  removeVariables(model, m_r1_var);
  removeVariables(model, m_v1_var);
  removeVariables(model, m_w1_var);
}

void RigidBody::backup() {
  m_x1_backup = m_x1_val;
  m_q1_backup = m_q1_val;
  m_v1_backup = m_v1_val;
  m_w1_backup = m_w1_val;
}

void RigidBody::restore() {
  m_x1_val = m_x1_backup;
  m_v1_val = m_v1_backup;
}

double RigidBody::getMass() {
  return 1./m_rb->getInvMass();
}

btQuaternion toBulletQuat(const Vector4d& q) {
  return btQuaternion(q[0], q[1], q[2], q[3]);
}
btVector3 toBulletVector(const Eigen::Vector3d& in) {
  return btVector3(in(0), in(1), in(2));
}

void RigidBody::updateBullet() {
  btTransform tf;
  tf.setOrigin(toBulletVector(m_x0));
  tf.setRotation(toBulletQuat(m_q0));
  m_rb->setCenterOfMassTransform(tf);
  m_rb->getMotionState()->setWorldTransform(tf);
}

Matrix4d quatPropagatorMat(const VectorXd& w) {
  // see http://www.lce.hut.fi/~ssarkka/pub/quat.pdf
  Matrix4d out;
  out << 0, -w[0], -w[1], -w[2],
      w[0], 0, w[2], -w[1],
      w[1], -w[2], 0, w[0],
      w[2], w[1], -w[0], 0;
  return .5*out;
}

Vector4d quatMult(const Vector4d& q1, const Vector4d& q2) {
  // copied from bullet
  return Vector4d(q1[3] * q2[0] + q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1],
      q1[3] * q2[1] + q1[1] * q2[3] + q1[2] * q2[0] - q1[0] * q2[2],
      q1[3] * q2[2] + q1[2] * q2[3] + q1[0] * q2[1] - q1[1] * q2[0],
      q1[3] * q2[3] - q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2]);
}

Vector4d propagatorQuat(const Vector3d& w, double dt) {
  // see http://www.lce.hut.fi/~ssarkka/pub/quat.pdf
  double normw = w.norm();
  Vector4d phiq;
  phiq(0) = cos(normw * DT/2);
  if (normw > SIMD_EPSILON) phiq.bottomRows(3) = w * (sin(normw*DT/2)/normw);
  else phiq.bottomRows(3) = Vector3d::Zero();
  return phiq;
}

void RigidBody::updateValues() {
  LOG_DEBUG(boost::format("%s vel: %s")%(m_name)%m_v1_val.transpose());
  LOG_DEBUG(boost::format("%s pis: %s")%(m_name)%m_x1_val.transpose());
  setValsToVars(m_x1_var, m_x1_val);
  setValsToVars(m_v1_var, m_v1_val);
  setValsToVars(m_w1_var, m_w1_val);
  Vector4d phiq = propagatorQuat(m_w1_val, DT);
  m_q1_val = quatMult(m_q0, phiq);
  LOG_DEBUG(boost::format("%s vel: %s")%(m_name)%m_v1_val.transpose());
  LOG_DEBUG(boost::format("%s pis: %s")%(m_name)%m_x1_val.transpose());
}

void RigidBody::advanceTime() {
  m_x0 = m_x1_val;
  m_q0 = m_q1_val;
  m_v0 = m_v1_val;
  m_w0 = m_w1_val;
}

RigidBody::IntegrationConstraint::IntegrationConstraint(RigidBody* rb) :
  m_rb(rb) {}

ConvexConstraintPtr RigidBody::IntegrationConstraint::convexify(GRBModel* model) {
  ConvexConstraintPtr out(new ConvexConstraint());
  for (int i=0; i < 3; ++i) out->m_eqcnts.push_back(model->addConstr(
      m_rb->m_x1_var[i] == m_rb->m_x0[i] + m_rb->m_v1_var[i]*DT));
#if 1
  for (int i=0; i < 3; ++i) out->m_eqcnts.push_back(model->addConstr(
      m_rb->m_r1_var[i] == (DT/2)*(m_rb->m_w1_var[i] - m_rb->m_w1_val[i])));
  // xxx not exactly true, use exponential instead
#else
  // todo
#endif
  return out;
}

Contact::Contact(RigidBodyPtr bodyA, RigidBodyPtr bodyB, const VectorXd& pt, const VectorXd& normal) :
  m_bodyA(bodyA),
  m_bodyB(bodyB),
  m_pt(pt),
  m_normal(normal)
{}

void Contact::addToModel(GRBModel* model) {
  m_fn = model->addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, (boost::format("fn_%s_%s")%m_bodyA->m_name%m_bodyB->m_name).str());
  m_fc = exprMult(m_normal, m_fn);
}

void Contact::removeFromModel(GRBModel* model) {
  model->remove(m_fn);
}

DynSolver::DynSolver(btCollisionWorld* world) : m_world(world)
{}

void DynSolver::updateValues() {
  BOOST_FOREACH(RigidBodyPtr& body, m_bodies)
    body->updateValues();
}

void DynSolver::storeValues() {
  BOOST_FOREACH(RigidBodyPtr& body, m_bodies)
    body->backup();
}

void DynSolver::rollbackValues() {
  BOOST_FOREACH(RigidBodyPtr& body, m_bodies)
    body->restore();
}

void DynSolver::addObject(RigidBodyPtr body) {
  m_bodies.push_back(body);
  body->addToModel(m_model);
  m_cnts.push_back(ConstraintPtr(new RigidBody::IntegrationConstraint(body.get())));
}

void DynSolver::updateBulletWorld() {
  BOOST_FOREACH(RigidBodyPtr& body, m_bodies) {
    body->updateBullet();
  }
}

void DynSolver::finishStep() {
  BOOST_FOREACH(RigidBodyPtr& body, m_bodies) {
    body->advanceTime();
    body->updateBullet();
  }
}

PoseConstraint::PoseConstraint(RigidBodyPtr body) :
  m_body(body) {
  setPose(body->m_x0, body->m_q0);
}

ConvexConstraintPtr PoseConstraint::convexify(GRBModel* model) {
  ConvexConstraintPtr out(new ConvexConstraint());
  for (int i=0; i < 3; ++i)
    out->m_eqcnts.push_back(model->addConstr(m_body->m_x1_var[i] == m_x[i]));
  for (int i=0; i < 4; ++i)
    out->m_eqcnts.push_back(model->addConstr(m_body->m_r1_var[i] == 0));
  return out;
}

void PoseConstraint::setPose(const Vector3d& x, const Vector4d& q) {
  m_x = x;
  m_q = q;
}

Vector3d a_grav(0,0,-9.8);

DynErrCost::DynErrCost(RigidBodyPtr body) :
    m_body(body),
    m_forceErrCoeff(10)
{}

ConvexObjectivePtr DynErrCost::convexify(GRBModel* model) {
  ExprVector dp(3), dl(3); // change in momentum, angular momentum
  // todo: loop over contacts;
  exprInc(dp, m_body->getMass()* a_grav * DT);

  ConvexObjectivePtr out(new ConvexObjective());
  out->m_objective += m_forceErrCoeff * exprNorm2(exprSub(dp, exprSub(m_body->m_v1_var, m_body->m_v0)));
  return out;
}

double DynErrCost::evaluate() {
  Vector3d dp = Vector3d::Zero();
  Vector3d dl = Vector3d::Zero();
  dp += m_body->getMass() * a_grav * DT;
  double out = 0;
  out += m_forceErrCoeff* (dp - (m_body->m_v1_val - m_body->m_v0)).squaredNorm();
  return out;
}

DynTrustRegion::DynTrustRegion(DynSolver* solver) : m_solver(solver) {
}
void DynTrustRegion::adjustTrustRegion(double ratio) {

}

ConvexConstraintPtr DynTrustRegion::convexify(GRBModel* model) {
  return ConvexConstraintPtr(new ConvexConstraint());
}
