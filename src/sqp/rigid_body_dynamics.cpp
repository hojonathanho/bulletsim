#include "rigid_body_dynamics.h"
#include "simulation/config_bullet.h"
#include <boost/format.hpp>
#include <boost/assign/list_of.hpp>
#include "utils_sqp.h"
#include "expr_ops.h"
#include "utils/logging.h"
#include "utils/clock.h"
#include "simulation/bullet_io.h"

using namespace std;
using namespace boost::assign;

#define ZERO_FRICTION
//#define ZERO_ROTATION

Matrix3d leftCrossProdMat(const Vector3d& x) {
  Matrix3d out;
  out << 0,   -x[2],   x[1],
      x[2],   0,     -x[0],
      -x[1],  x[0],    0;
  return out;
}
Matrix3d rightCrossProdMat(const Vector3d& x) {
  return leftCrossProdMat(x).transpose();
}

ExprVector exprMatMult(const MatrixXd& A, const VarVector& x) {
  ExprVector y(x.size());
  for (int i=0; i < y.size(); ++i) {
    y[i] = varDot(A.row(i), x);
  }
  return y;
}
ExprVector exprCross(const VectorXd& x, const VarVector& y) {
  return exprMatMult(leftCrossProdMat(x), y);
}
ExprVector exprCross(const VarVector& x, const VectorXd& y) {
  return exprMatMult(rightCrossProdMat(y), x);
}
ExprVector exprMatMult(const MatrixXd& A, const ExprVector& x) {
  ExprVector y(x.size());
  for (int i=0; i < y.size(); ++i) {
    y[i] = exprDot(A.row(i), x);
  }
  return y;
}
ExprVector exprCross(const VectorXd& x, const ExprVector& y) {
  return exprMatMult(leftCrossProdMat(x), y);
}
ExprVector exprCross(const ExprVector& x, const VectorXd& y) {
  return exprMatMult(rightCrossProdMat(y), x);
}



Matrix4d quatPropagatorMat(const VectorXd& w) {
  // see http://www.lce.hut.fi/~ssarkka/pub/quat.pdf
  Matrix4d out;
  out <<
         0, w[2], -w[1], w[0],
       -w[2], 0,    w[0], w[1],
       w[1], -w[0], 0,    w[2],
      -w[0], -w[1], -w[2], 0;
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
  if (normw > 0) phiq.topRows(3) = (w/normw) * sin(normw*dt/2);
  else phiq.topRows(3) = Vector3d::Zero();
  phiq(3) = cos(normw * dt/2);
  return phiq;
}

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
  m_w1_val(m_w0),
  m_externalImpulse(Vector3d::Zero())
{
  assert(body->getUserPointer()==NULL);
  body->setUserPointer(this);
}

void RigidBody::addToModel(GRBModel* model) {
  for (int i=0; i < 3; ++i) {
    m_x1_var.push_back(model->addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, (boost::format("%s_x_%i")%m_name%i).str()));
    m_v1_var.push_back(model->addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, (boost::format("%s_v_%i")%m_name%i).str()));
#ifdef ZERO_ROTATION
    m_r1_var.push_back(model->addVar(0, 0, 0, GRB_CONTINUOUS, (boost::format("%s_dq_%i")%m_name%i).str()));
    m_w1_var.push_back(model->addVar(0, 0, 0, GRB_CONTINUOUS, (boost::format("%s_w_%i")%m_name%i).str()));
#else
    m_r1_var.push_back(model->addVar(-GRB_INFINITY,GRB_INFINITY, 0, GRB_CONTINUOUS, (boost::format("%s_dq_%i")%m_name%i).str()));
    m_w1_var.push_back(model->addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, (boost::format("%s_w_%i")%m_name%i).str()));
#endif
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

void RigidBody::updateBullet0() {
  btTransform tf;
  tf.setOrigin(toBulletVector(m_x0));
  tf.setRotation(toBulletQuat(m_q0));
  m_rb->setCenterOfMassTransform(tf);
  m_rb->getMotionState()->setWorldTransform(tf);
//  LOG_ERROR("updateBullet0 setting " << m_name << " " << tf);
}


void RigidBody::updateBullet1() {
  btTransform tf;
  tf.setOrigin(toBulletVector(m_x1_val));
  tf.setRotation(toBulletQuat(m_q1_val));
  m_rb->setCenterOfMassTransform(tf);
  m_rb->getMotionState()->setWorldTransform(tf);
//  LOG_ERROR("updateBullet1 setting " << m_name << " " << tf);
}


void RigidBody::updateValues() {
  Vector3d x1old = m_x1_val;
  Vector3d v1old = m_v1_val;
  Vector3d w1old = m_w1_val;
  Vector4d q1old = m_q1_val;
  setValsToVars(m_x1_var, m_x1_val);
  setValsToVars(m_v1_var, m_v1_val);
  setValsToVars(m_w1_var, m_w1_val);
  Vector3d r1;
  setValsToVars(m_r1_var, r1);
  Vector4d phiq = propagatorQuat(m_w1_val, DT);
  m_q1_val = quatMult(m_q0, phiq);
  LOG_DEBUG(boost::format("%s x: %s->%s")%m_name%x1old.transpose()%m_x1_val.transpose());
  LOG_DEBUG(boost::format("%s v: %s->%s")%m_name%v1old.transpose()%m_v1_val.transpose());
  LOG_DEBUG(boost::format("%s w: %s->%s")%m_name%w1old.transpose()%m_w1_val.transpose());
  LOG_DEBUG(boost::format("%s q: %s->%s")%m_name%q1old.transpose()%m_q1_val.transpose());
  LOG_DEBUG(boost::format("%s r: %s")%m_name%r1.transpose());
}

void RigidBody::advanceTime() {
  m_x0 = m_x1_val;
  m_q0 = m_q1_val;
  m_v0 = m_v1_val;
  m_w0 = m_w1_val;
  m_externalImpulse.setZero();
}

void RigidBody::predictMotion() {
  m_x1_val = m_x0 + m_v0*DT;
  m_q1_val = quatMult(m_q0,propagatorQuat(m_w0, DT));
}

IntegrationConstraint::IntegrationConstraint(RigidBodyPtr rb) :
  m_rb(rb) {}

ConvexConstraintPtr IntegrationConstraint::convexify(GRBModel* model) {
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

PoseConstraint::PoseConstraint(RigidBodyPtr body) :
  m_body(body) {
  setPose(body->m_x0, body->m_q0);
}

ConvexConstraintPtr PoseConstraint::convexify(GRBModel* model) {
  ConvexConstraintPtr out(new ConvexConstraint());
  for (int i=0; i < 3; ++i)
    out->m_eqcnts.push_back(model->addConstr(m_body->m_x1_var[i] == m_x[i]));
  for (int i=0; i < 3; ++i)
    out->m_eqcnts.push_back(model->addConstr(m_body->m_r1_var[i] == 0));
  return out;
}

void PoseConstraint::setPose(const Vector3d& x, const Vector4d& q) {
  m_x = x;
  m_q = q;
}

string Contact::getName() {
  return (boost::format("%s->%s")%m_bodyA->m_name%m_bodyB->m_name).str();
}

void Contact::addToModel(GRBModel* model) {
  assert(!m_inModel);
  m_fn = model->addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS, "fn_"+getName());
  m_ffr.resize(3);
#ifdef ZERO_FRICTION
  for (int i=0; i < 3; ++i) m_ffr[i] = model->addVar(0,0, 0, GRB_CONTINUOUS, (boost::format("ffr_%s_%i")%getName()%i).str());
#else
  for (int i=0; i < 3; ++i) m_ffr[i] = model->addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, (boost::format("ffr_%s_%i")%getName()%i).str());
#endif
  m_inModel = true;
}

void Contact::calcDistExpr() {
  GRBLinExpr dDistOfA = makeDerivExpr(m_normalB2A, m_bodyA->m_x1_var, m_bodyA->m_x1_val)
                      + exprDot(m_normalB2A, exprCross(m_bodyA->m_r1_var, m_worldA - m_bodyA->m_x1_val));
  GRBLinExpr dDistOfB = makeDerivExpr(-m_normalB2A, m_bodyB->m_x1_var, m_bodyB->m_x1_val)
                      + exprDot(-m_normalB2A, exprCross(m_bodyB->m_r1_var, m_worldB - m_bodyB->m_x1_val));
  m_distExpr = (dDistOfA + dDistOfB + m_dist);
//  cout << getName() << " " << m_distExpr << endl;
}

void Contact::removeFromModel(GRBModel* model) {
  assert(m_inModel);
  model->remove(m_fn);
  m_inModel = false;
}

Contact::~Contact() {
  if (m_inModel) LOG_WARN_FMT("warning: contact %s deleted but still in model!", getName().c_str());
}


DynSolver::DynSolver(btCollisionWorld* world) : m_world(world)
{}

void DynSolver::updateValues() {
  BOOST_FOREACH(RigidBodyPtr& body, m_bodies)
    body->updateValues();
  BOOST_FOREACH(ContactPtr& contact, m_contacts)
    contact->updateValues();
  updateContacts();
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
  addConstraint(ConstraintPtr(new IntegrationConstraint(body)));
  if (body->m_name != "floor") {
    CostPtr dynCost(new DynErrCost(body, this));
    addCost(dynCost);
  }
}

Contact::Contact(RigidBody* bodyA, RigidBody* bodyB, const btVector3& worldA, const btVector3& worldB,
                 const btVector3& normalB2A, double dist) :
  m_inModel(false),
  m_bodyA(bodyA),
  m_bodyB(bodyB),
  m_worldA(toVector3d(worldA)),
  m_worldB(toVector3d(worldB)),
  m_normalB2A(toVector3d(normalB2A)),
  m_dist(dist),
  m_fn_val(0),
  m_ffr_val(Vector3d::Zero())
{
  calcDistExpr();
}

void Contact::setData(const btVector3& worldA, const btVector3& worldB, const btVector3& normalB2A, double dist) {
  m_worldA = toVector3d(worldA);
  m_worldB = toVector3d(worldB);
  m_normalB2A = toVector3d(normalB2A);
  m_dist = dist;
  calcDistExpr();
}

void Contact::updateValues() {
  Vector3d ffr_old = m_ffr_val;
  double fn_old = m_fn_val;
  setValsToVars(m_ffr,m_ffr_val);
  m_fn_val = m_fn.get(GRB_DoubleAttr_X);
  LOG_DEBUG(boost::format("%s ffr %s->%s")%getName()%ffr_old.transpose()%m_ffr_val.transpose());
  LOG_DEBUG(boost::format("%s fn %s->%s")%getName()%fn_old%m_fn_val);
}

btManifoldPoint& getDeepestPoint(btPersistentManifold* contactManifold) {
  int numContacts = contactManifold->getNumContacts();
  assert(numContacts > 0);
  int iDeepest = -1;
  float minDist = 9999;
  for (int j = 0; j < numContacts; ++j) {
    btManifoldPoint& pt = contactManifold->getContactPoint(j);
    if (pt.getDistance() < minDist) {
      minDist = pt.getDistance();
      iDeepest = j;
    }
  }
  return contactManifold->getContactPoint(iDeepest);
}

void DynSolver::updateContacts() {
  BOOST_FOREACH(RigidBodyPtr& body, m_bodies) body->updateBullet1();
  vector<ContactPtr> newContacts;


//  set<> created, destroyed, preserved;
  vector<char> oldContactSurvival(m_contacts.size(), false), newContactWasCreated;
  typedef pair<btRigidBody*, btRigidBody*> RBPair;

  m_world->performDiscreteCollisionDetection();
  btDispatcher* dispatcher = m_world->getDispatcher();
  int numManifolds = dispatcher->getNumManifolds();
  LOG_INFO_FMT("%i manifolds", numManifolds);
  for (int i = 0; i < numManifolds; ++i) {
    btPersistentManifold* contactManifold = dispatcher->getManifoldByIndexInternal(i);
    btRigidBody* objA = static_cast<btRigidBody*> (contactManifold->getBody0());
    btRigidBody* objB = static_cast<btRigidBody*> (contactManifold->getBody1());
    RigidBody* rbA = static_cast<RigidBody*> (objA->getUserPointer());
    RigidBody* rbB = static_cast<RigidBody*> (objB->getUserPointer());
    if (rbA == NULL || rbB == NULL) {
      LOG_INFO("found collision but solver doesn't know about both objects");
      continue; // iterate over manifolds
    }

    int numContacts = contactManifold->getNumContacts();

    for (int i=0; i < numContacts; ++i) {

      btManifoldPoint& pt = contactManifold->getContactPoint(i);


      if (pt.m_userPersistentData != NULL) {
        int ind = *static_cast<int*>(pt.m_userPersistentData);
        ContactPtr newContact = m_contacts[ind];
        newContact->setData(pt.m_positionWorldOnA, pt.m_positionWorldOnB,
                         pt.m_normalWorldOnB, pt.m_distance1);
        newContactWasCreated.push_back(false);
        oldContactSurvival[ind]=true;
  //      LOG_ERROR("set contact distance to " << pt.m_distance1);
  //      LOG_ERROR("trans " << objA->getCenterOfMassPosition() << "|" << objB->getCenterOfMassPosition());
        *static_cast<int*>(pt.m_userPersistentData) = newContacts.size();
        newContacts.push_back(newContact);
      }
      else {
        pt.m_userPersistentData = new int(newContacts.size());
        ContactPtr newContact(new Contact(rbA, rbB, pt.m_positionWorldOnA, pt.m_positionWorldOnB,
                                          pt.m_normalWorldOnB, pt.m_distance1));
        newContactWasCreated.push_back(true);
        newContacts.push_back(newContact);
      }
    }
  }

  for (int i=0; i < oldContactSurvival.size(); ++i)
    if (oldContactSurvival[i]) {
      LOG_DEBUG(boost::format("contact %s survived")%m_contacts[i]->getName());
    }
    else {
      LOG_INFO(boost::format("contact %s destroyed")%m_contacts[i]->getName());
      m_contacts[i]->removeFromModel(m_model);
    }
  for (int i=0; i < newContactWasCreated.size(); ++i)
    if (newContactWasCreated[i]) {
      newContacts[i]->addToModel(m_model);
      LOG_INFO(boost::format("contact %s created")%newContacts[i]->getName());
    }

  m_contacts = newContacts;
  m_model->update();
  plotContacts(m_contacts);
}

void DynSolver::postOptimize() {
}

void DynSolver::finishStep() {
  BOOST_FOREACH(RigidBodyPtr& body, m_bodies) {
    body->updateBullet1();
    body->advanceTime();
    body->predictMotion();
  }
}

vector<SignedContact> DynSolver::getSignedContacts(RigidBody* body) {
  vector<SignedContact> out;
  BOOST_FOREACH(ContactPtr& contact, m_contacts) {
    if (body == contact->m_bodyA) out.push_back(SignedContact(contact.get(),1));
    else if (body == contact->m_bodyB) out.push_back(SignedContact(contact.get(),-1));
  }
  return out;
}

void DynSolver::step() {
  LOG_INFO("starting step");
  double tStart = GetClock();
  updateContacts();
  optimize();
  finishStep();
  LOG_INFO_FMT("finished step (%.2f ms)", GetClock()-tStart);

}

void DynSolver::fixVariables() {
  vector<GRBConstr> cnts;

  BOOST_FOREACH(RigidBodyPtr& body, m_bodies) {
    for (int i = 0; i < 3; ++i) {
      cnts.push_back(m_model->addConstr(body->m_x1_var[i] == body->m_x1_val[i]));
      cnts.push_back(m_model->addConstr(body->m_v1_var[i] == body->m_v1_val[i]));
      cnts.push_back(m_model->addConstr(body->m_r1_var[i] == 0));
      cnts.push_back(m_model->addConstr(body->m_w1_var[i] == body->m_w1_val[i]));
    }
  }
  BOOST_FOREACH(ContactPtr& contact, m_contacts) {
    cnts.push_back(m_model->addConstr(contact->m_fn == contact->m_fn_val));
    for (int i = 0; i < 3; ++i) {
      cnts.push_back(m_model->addConstr(contact->m_ffr[i] == contact->m_ffr_val[i]));
    }
  }

}

Vector3d a_grav(0,0,-9.8);

DynErrCost::DynErrCost(RigidBodyPtr body, DynSolver* solver) :
    m_body(body),
    m_solver(solver),
    m_forceErrCoeff(1000)
{}

ConvexObjectivePtr DynErrCost::convexify(GRBModel* model) {
  ExprVector dp(3), dl(3); // change in momentum, angular momentum
  double mass = m_body->getMass();
  // todo: loop over contacts;
  exprInc(dp, a_grav * DT + m_body->m_externalImpulse);
//  exprInc(dl, Vector3d(0,0,.1));
//  LOG_ERROR(m_solver->getSignedContacts(m_body.get()).size() << " contacts");

  BOOST_FOREACH(const SignedContact& sc,m_solver->getSignedContacts(m_body.get())) {
    ExprVector contactForce = exprMult(sc.sign*sc.contact->m_normalB2A, sc.contact->m_fn);
    exprInc(contactForce, exprMult(sc.contact->m_ffr, sc.sign));
    exprInc(dp, contactForce);
    // xxx using worldA
    // xxx missing term
    exprInc(dl, exprCross(sc.contact->m_worldA - m_body->m_x1_val, contactForce));
  }

//  cout << m_body->m_name << " dp" << dp << endl;

  ConvexObjectivePtr out(new ConvexObjective());
  ExprVector pErr = exprSub(dp,exprSub(m_body->m_v1_var, m_body->m_v0));
  ExprVector lErr = exprSub(dl,exprSub(m_body->m_w1_var, m_body->m_w0));
//  out->m_objective += m_forceErrCoeff * exprNorm2(dynErr);
#ifdef CONE_ERROR
  addNormCost(out, m_forceErrCoeff, pErr, model, "momentum_error");
  addNormCost(out, m_forceErrCoeff, lErr, model, "ang_mom_error");
#else
  for (int i=0; i < 3; ++i) {
    addAbsCost(out, m_forceErrCoeff, pErr[i], model, "momentum_error");
    addAbsCost(out, m_forceErrCoeff, lErr[i], model, "ang_mom_error");
  }
#endif
//  cout << m_body->m_name << " obj" << exprSub(dp,exprSub(m_body->m_v1_var, m_body->m_v0))[2] << endl;
//  cout << m_body->m_name << " dv" << exprSub(dp,exprSub(m_body->m_v1_var, m_body->m_v0))[2] << endl;
  return out;
}

double DynErrCost::evaluate() {
  LOG_DEBUG("external impulse: " << m_body->m_externalImpulse.transpose());
  Vector3d dp = Vector3d::Zero();
  Vector3d dl = Vector3d::Zero();
//  dl += Vector3d(.0,0,.1);
  double mass = m_body->getMass();
  dp += mass * a_grav * DT + m_body->m_externalImpulse;

  BOOST_FOREACH(const SignedContact& sc,m_solver->getSignedContacts(m_body.get())) {
    Vector3d contactForce = sc.contact->m_fn_val * (sc.sign * sc.contact->m_normalB2A);
    contactForce += sc.sign* sc.contact->m_ffr_val;
    dp += contactForce;
    dl += (sc.contact->m_worldA - m_body->m_x1_val).cross(contactForce); // xxx WTF
  }

  double out = 0;
  Vector3d pErr = dp - (m_body->m_v1_val - m_body->m_v0);
  Vector3d lErr = dl - (m_body->m_w1_val - m_body->m_w0);
#ifdef CONE_ERROR
  out +=  m_forceErrCoeff * (pErr.norm() + lErr.norm());
#else
  out +=  m_forceErrCoeff * (pErr.lpNorm<1>() + lErr.lpNorm<1>());
#endif
  return out;
}

ForcePenalty::ForcePenalty(DynSolver* solver) : m_solver(solver), m_coeff(10) {}
double ForcePenalty::evaluate() {
  double out=0;
  BOOST_FOREACH(ContactPtr& contact, m_solver->m_contacts) {
    out += m_coeff*contact->m_fn_val;
  }
  return out;
}
ConvexObjectivePtr ForcePenalty::convexify(GRBModel* model) {
  ConvexObjectivePtr out(new ConvexObjective());
  BOOST_FOREACH(ContactPtr& contact, m_solver->m_contacts) {
    out->m_objective += m_coeff * contact->m_fn;
//    out->m_objective += m_coeff * contact->m_fn * contact->m_fn;
  }
  return out;
}

ComplementarityCost::ComplementarityCost(DynSolver* solver) : m_solver(solver) {}
double ComplementarityCost::evaluate() {
  double out = 0;
  BOOST_FOREACH(ContactPtr& contact, m_solver->m_contacts) {
    out += pospart(contact->m_dist * contact->m_fn_val);
  }
  return out;
}
ConvexObjectivePtr ComplementarityCost::convexify(GRBModel* model) {
  ConvexObjectivePtr out(new ConvexObjective());
  BOOST_FOREACH(ContactPtr& contact, m_solver->m_contacts) {
    GRBLinExpr compExpr = contact->m_distExpr * contact->m_fn_val + contact->m_dist * (contact->m_fn - contact->m_fn_val);
    addHingeCost(out, m_coeff, compExpr, model, "compl");
  }
  return out;
}



FrictionConstraint::FrictionConstraint(DynSolver* solver) : m_solver(solver), m_mu2(1) {}
ConvexConstraintPtr FrictionConstraint::convexify(GRBModel* model) {
  ConvexConstraintPtr out(new ConvexConstraint());
  BOOST_FOREACH(ContactPtr& contact, m_solver->m_contacts) {
#ifndef ZERO_FRICTION
    out->m_eqcnts.push_back(model->addConstr(varDot(contact->m_normalB2A, contact->m_ffr) == 0));
//    out->m_qexprs.push_back(exprNorm2(contact->m_ffr) - m_mu2 * contact->m_fn * contact->m_fn);
    out->m_qexprs.push_back(varNorm2(contact->m_ffr) - m_mu2 * contact->m_fn * contact->m_fn);
//    out->m_qcnts.push_back(model->addQConstr(varNorm2(contact->m_ffr) <= m_mu2 * contact->m_fn * contact->m_fn));
    out->m_qcntNames.push_back("fric_cone");
#endif
  }
  return out;
}



OverlapPenalty::OverlapPenalty(DynSolver* solver) : m_solver(solver), m_coeff(999) {}
  
double OverlapPenalty::evaluate() {
  double out=0;
  BOOST_FOREACH(ContactPtr& contact, m_solver->m_contacts) {
//    LOG_ERROR_FMT("contact: %s. dist: %.2f", contact->getName().c_str(), contact->m_dist);
    out += m_coeff * pospart(-contact->m_dist);
  }
  return out;
}

ConvexObjectivePtr OverlapPenalty::convexify(GRBModel* model) {
  ConvexObjectivePtr out(new ConvexObjective());
  BOOST_FOREACH(ContactPtr& contact, m_solver->m_contacts) {
    addHingeCost(out, m_coeff, -contact->m_distExpr, model, (boost::format("%s_overlap")%contact->getName()).str());
  }
  return out;
}

OverlapConstraint::OverlapConstraint(DynSolver* solver) : m_solver(solver) {}

ConvexConstraintPtr OverlapConstraint::convexify(GRBModel* model) {
  ConvexConstraintPtr out(new ConvexConstraint());
  BOOST_FOREACH(ContactPtr& contact, m_solver->m_contacts) {
    out->m_exprs.push_back(-contact->m_distExpr);
    out->m_cntNames.push_back((boost::format("%s_overlap")%contact->getName()).str());
  }
  return out;
}


TangentMotion::TangentMotion(DynSolver* solver) : m_coeff(.01), m_solver(solver) {}

double TangentMotion::evaluate() {
  double out = 0;
  BOOST_FOREACH(ContactPtr& contact, m_solver->m_contacts) {
    Matrix3d tanProj = Matrix3d::Identity() - contact->m_normalB2A * contact->m_normalB2A.transpose();
    Vector3d vtan_val = tanProj*(
       contact->m_bodyA->m_v1_val
      + contact->m_bodyA->m_w1_val.cross(contact->m_worldA - contact->m_bodyA->m_x1_val)
      - contact->m_bodyB->m_v1_val
      - contact->m_bodyB->m_w1_val.cross(contact->m_worldB - contact->m_bodyB->m_x1_val));
    out += m_coeff * vtan_val.norm();
  }
  return out;
}


ConvexObjectivePtr TangentMotion::convexify(GRBModel* model) {
  ConvexObjectivePtr out(new ConvexObjective());
  BOOST_FOREACH(ContactPtr& contact, m_solver->m_contacts) {
    Matrix3d tanProj = Matrix3d::Identity() - contact->m_normalB2A * contact->m_normalB2A.transpose();


    ExprVector tanvel(3);
    exprInc(tanvel, contact->m_bodyA->m_v1_var);
    exprInc(tanvel, exprCross(contact->m_bodyA->m_w1_var, contact->m_worldA - contact->m_bodyA->m_x1_val));
    exprDec(tanvel, contact->m_bodyB->m_v1_var);
    exprDec(tanvel, exprCross(contact->m_bodyB->m_w1_var, contact->m_worldB - contact->m_bodyB->m_x1_val));
//    for (int i=0; i < 3; ++i) {
//      GRBLinExpr vtani = exprMatMult(tanProj, exprSub(contact->m_bodyA->m_v1_val, contact->m_bodyB->m_v1_val))
//          + makeDerivExpr(tanProj.row(i), contact->m_bodyA->m_v1_var, contact->m_bodyA->m_v1_val)
//          - makeDerivExpr(tanProj.row(i), contact->m_bodyB->m_v1_var, contact->m_bodyB->m_v1_val)
//          + makeDerivExpr(dvtan_dwA.row(i), contact->m_bodyA->m_v1_var, Vector3d::Zero())
//          - makeDerivExpr(dvtan_dwB.row(i), contact->m_bodyB->m_v1_var, Vector3d::Zero());
//      LOG_ERROR("vtan " << i << " " << vtani);
    addNormCost(out, m_coeff, exprMatMult(tanProj, tanvel), model, "tangent_vel");
//      out->m_objective += m_coeff * exprNorm2(exprMatMult(tanProj, tanvel));
  }
  return out;
}

DynTrustRegion::DynTrustRegion(DynSolver* solver) : m_solver(solver) {}
void DynTrustRegion::adjustTrustRegion(double ratio) {}

ConvexConstraintPtr DynTrustRegion::convexify(GRBModel* model) {
  return ConvexConstraintPtr(new ConvexConstraint());
}




#include "config_sqp.h"
inline double sum(const vector<double>& x) {
  double out=0;
  for (int i=0; i < x.size(); ++i) out += x[i];
  return out;
}
extern vector<double> evalApproxObjectives(const vector<ConvexObjectivePtr>& in);



Optimizer::OptStatus DynSolver::optimize() {

  ////////////
  double trueImprove, approxImprove, improveRatio; // will be used to check for convergence
  vector<double> newObjectiveVals, objectiveVals;
  bool grbFail=false;
  /////////


  for (int iter = 0; iter < 2; ++iter) {

    LOG_INFO_FMT("iteration: %i", iter);

    ////// convexification /////////
    // slight optimization: don't evaluate objectives
    // if you just did so while checking for improvement
    vector<ConvexObjectivePtr> objectives = convexifyObjectives();
    if (newObjectiveVals.size() == 0) objectiveVals = evaluateObjectives();
    else objectiveVals = newObjectiveVals;
    double objectiveVal = sum(objectiveVals);
    vector<ConvexConstraintPtr> constraints = convexifyConstraints();
    setupConvexProblem(objectives, constraints);
//    for (int i=0; i < objectives.size(); ++i) {
//      LOG_ERROR(m_costs[i]->getName() << ":     " << objectives[i]->m_objective);
//    }
//    LOG_ERROR("total: " << m_model->getObjective());
    ///////////////////////////////////


      ////// convex optimization /////////////////////
      preOptimize();
      int grbStatus = convexOptimize();
//      m_model->write("/tmp/fuk.lp");
//      m_model->write("/tmp/fuk.sol");
      if (grbStatus != GRB_OPTIMAL) {
        LOG_ERROR_FMT("bad GRB status: %s. problem written to /tmp/sqp_fail.lp", getGRBStatusString(grbStatus));
        m_model->write("/tmp/sqp_fail.lp");
        grbFail = true;
        break;
      }
      vector<double> approxObjectiveVals = evalApproxObjectives(objectives);
      double approxObjectiveVal = sum(approxObjectiveVals);

      storeValues();
      updateValues();
      postOptimize();
      //////////////////////////////////////////////

      ////////// calculate new objectives ////////////
      newObjectiveVals = evaluateObjectives();
      double newObjectiveVal = sum(newObjectiveVals);
      printObjectiveInfo(objectiveVals, approxObjectiveVals, newObjectiveVals);
      trueImprove = objectiveVal - newObjectiveVal;
      approxImprove = objectiveVal - approxObjectiveVal;
      improveRatio = trueImprove / approxImprove;
      LOG_INFO_FMT("%15s | %10.3e | %10.3e | %10.3e | %10.3e", "TOTAL", objectiveVal, approxImprove, trueImprove, improveRatio);
      //////////////////////////////////////////////


    clearConvexProblem(objectives, constraints);

    //// exit conditions ///
    if (grbFail) {
      return GRB_FAIL;
    }
    if (iter >= SQPConfig::maxIter) {
      LOG_WARN("reached iteration limit");
      return ITERATION_LIMIT;
    }
    if (trueImprove < SQPConfig::doneIterThresh && improveRatio > SQPConfig::trThresh) {
      LOG_INFO_FMT("cost improvement below convergence threshold (%.3e < %.3e). stopping", SQPConfig::doneIterThresh, SQPConfig::trThresh);
      return CONVERGED; // xxx should probably check that it improved multiple times in a row
    }

    ///////////////////////

  }
}


#include "simulation/plotting.h"
#include "simulation/simulation_fwd.h"
#include "simulation/util.h"
using namespace util;
PlotPointsPtr collisions;
PlotLinesPtr escapes;

btVector3 toBtVector(const Vector3d& x) {return btVector3(x[0], x[1], x[2]);}

void plotContacts(const vector<ContactPtr>& contacts) {
  if (!collisions) {
    collisions.reset(new PlotPoints(10));
    escapes.reset(new PlotLines(5));
    getGlobalEnv()->add(collisions);
    getGlobalEnv()->add(escapes);
//    collisions->getOSGNode()->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
    escapes->getOSGNode()->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
  }

  double safeDist = 0;
  const osg::Vec4 GREEN(0, 1, 0, 1), YELLOW(1, 1, 0, 1), RED(1, 0, 0, 1);
  osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
  osg::ref_ptr<osg::Vec3Array> escPts = new osg::Vec3Array;
  osg::ref_ptr<osg::Vec3Array> collPts = new osg::Vec3Array;
	BOOST_FOREACH(const ContactPtr& contact, contacts)
	{
		collPts->push_back(toOSGVector(toBtVector(contact->m_worldA)));
		escPts->push_back(toOSGVector(toBtVector(contact->m_worldA)));
		escPts->push_back( toOSGVector(toBtVector(contact->m_worldA - contact->m_normalB2A )));
		if (contact->m_dist < 0)
			colors->push_back(RED);
		else
			colors->push_back(GREEN);
	}

  assert(collPts->size() == colors->size());
  assert(escPts->size() == 2*colors->size());

  if (collPts->size() == 1) {
    collPts->push_back(osg::Vec3(0,0,0));
    colors->push_back(osg::Vec4(0,0,0,0));
    escPts->push_back(osg::Vec3(0,0,0));
    escPts->push_back(osg::Vec3(1,0,0));
  }
  collisions->setPoints(collPts, colors);
  escapes->setPoints(escPts, colors);
  LOG_DEBUG_FMT("plotting %i proximities", collPts->size());
}

