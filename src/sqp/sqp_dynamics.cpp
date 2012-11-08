#include "sqp_dynamics.h"
#include "sqp.h"
#include <btBulletDynamicsCommon.h>
#include "state_setter.h"
#include <boost/foreach.hpp>
#include "config_sqp.h"
#include "expr_ops.h"
#include "utils/logging.h"
using std::cout;
using std::endl;
using Eigen::Matrix3d;
using Eigen::Vector3d;

extern Eigen::VectorXd toXYZROD(const btTransform& tf);
extern btTransform fromXYZROD(const Eigen::VectorXd& xyzrod);
Matrix3d rotJacWorld(const Vector3d& ptWorld, const Vector3d& centerWorld, const Vector3d& rod);

double unwrap_angle(double x, double targ) {
  while (true) {
    if (x - targ > SIMD_PI) x -= 2*SIMD_PI;
    else if (x - targ < -SIMD_PI) x+= 2*SIMD_PI;
    else break;
  }
  return x;
}

Eigen::VectorXd toXYZROD(const btTransform& t, const Eigen::VectorXd& xyzrod) {
  VectorXd out = toXYZROD(t);
  for (int i=3; i < 6; ++i) {
    out(i) = unwrap_angle(out(i), xyzrod(i));
  }
  return out;

}

template<typename S, typename T>
vector<T> sliceVec(const vector<T>& in, int start, int end) {
  return vector<T> (in.begin() + start, in.begin() + end);
}


void TrajDynSolver::updateValues() {
  BOOST_FOREACH(btRigidBody* o, m_bodies) {
    VarArray& poseVars = m_obj2poseVars[o];
    MatrixXd& poses = m_obj2poses[o];
    for (int i = 0; i < poses.rows(); ++i) {
      for (int j = 0; j < poses.cols(); ++j) {
        poses(i, j) = poseVars(i, j).get(GRB_DoubleAttr_X);
      }
    }
  }
}

void TrajDynSolver::storeValues() {
  m_obj2poses_backup = m_obj2poses;
}

void TrajDynSolver::rollbackValues() {
  m_obj2poses = m_obj2poses_backup;
}

double TrajDynSolver::fixPermanentVarsAndOptimize() {
  vector<GRBConstr> cnts;

  BOOST_FOREACH(btRigidBody* o, m_bodies) {
    VarArray& poseVars = m_obj2poseVars[o];
    MatrixXd& poses = m_obj2poses[o];
    for (int i = 0; i < poses.rows(); ++i) {
      for (int j = 0; j < poses.cols(); ++j) {
        cnts.push_back(m_model->addConstr(poseVars(i, j) == poses(i, j)));
      }
    }
  }

  m_model->optimize();
  double out = getApproxObjective();

  BOOST_FOREACH(GRBConstr& cnt, cnts) {
    m_model->remove(cnt);
  }

  return out;
}

VarVector TrajDynSolver::getPoseVars(btRigidBody* o, int t) {
  if (t == CW_TIME)
    t = m_t;
  return m_obj2poseVars[o].row(t);
}

VectorXd TrajDynSolver::getPoseValues(btRigidBody* o, int t) {
  if (t == CW_TIME)
    t = m_t;
  return m_obj2poses[o].row(t);
}

ExprVector TrajDynSolver::getVelVars(btRigidBody* o, int t) {
  if (t == CW_TIME)
    t = m_t;
  int thi, tlo;
  if (t == getNumTimesteps() - 1) {
    thi = t;
    tlo = t - 1;
  } else {
    thi = t + 1;
    tlo = t;
  }
  ExprVector out(POSE_DIM);
  VarArray& poses = m_obj2poseVars[o];
  for (int i = 0; i < POSE_DIM; ++i) {
    out[i] = poses(thi, i) - poses(tlo, i);
  }
  return out;
}

VectorXd TrajDynSolver::getVelValues(btRigidBody* o, int t) {
  if (t == CW_TIME)
    t = m_t;
  int thi, tlo;
  if (t == getNumTimesteps() - 1) {
    thi = t;
    tlo = t - 1;
  } else {
    thi = t + 1;
    tlo = t;
  }
  MatrixXd& poses = m_obj2poses[o];
  return poses.row(thi) - poses.row(tlo);
}

void TrajDynSolver::setTimestep(int t) {
  VectorXd stateVec(m_bodies.size() * POSE_DIM);
  int iStart = 0;
  BOOST_FOREACH(btRigidBody* o, m_bodies) {
    stateVec.middleRows(iStart, POSE_DIM) = m_obj2poses[o].row(t);
    iStart += POSE_DIM;
  }
  m_ss->setState(stateVec);
  m_t = t;
}

int TrajDynSolver::getNumTimesteps() {
  return m_numTimesteps;
}

VectorXd TrajDynSolver::getPose(btRigidBody* o) {
  return toVector3d(o->getCenterOfMassPosition());
}

void TrajDynSolver::addObject(btRigidBody* o) {
  assert(POSE_DIM==3);
  addObject(o, toVector3d(o->getCenterOfMassPosition()));
}

void TrajDynSolver::addObject(btRigidBody* o, const VectorXd& initPose) {
  assert(!m_initialized);
  m_bodies.push_back(o);
  m_obj2poseVars[o] = VarArray(getNumTimesteps(), POSE_DIM);
  m_obj2poses[o] = MatrixXd::Zero(getNumTimesteps(), POSE_DIM);
  VarArray& poseVars = m_obj2poseVars[o];

  for (int i = 0; i < getNumTimesteps(); ++i) {
    for (int j = 0; j < POSE_DIM; ++j) {
      char namebuf[10];
      sprintf(namebuf, "j_%i_%i", i, j);
      poseVars(i, j) = m_model->addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, namebuf);
    }
  }
  m_model->update();
}

void TrajDynSolver::setStatic(btRigidBody* o) {
  assert(!m_initialized);
  assert(m_obj2poses.find(o) != m_obj2poses.end());
  constrainPoses(o, m_obj2poses[o]); // assuming poses were initialized with current position
}

void TrajDynSolver::constrainPoses(btRigidBody* o, const MatrixXd& poses) {
  assert(!m_initialized);
  VarArray& poseVars = m_obj2poseVars[o];
  for (int i = 0; i < poses.rows(); ++i) {
    for (int j = 0; j < poseVars.cols(); ++j) {
      m_model->addConstr(poseVars(i, j) == poses(i, j));
    }
  }
}

TrajDynSolver::TrajDynSolver(btCollisionWorld* world, int numTimesteps) :
  m_world(world), m_t(0), m_numTimesteps(numTimesteps), m_initialized(false) {
}

void TrajDynSolver::initialize() {
  m_initialized = true;
}


TrajDynSolver* TrajDynComponent::getSolver() {
  return m_solver;
}

VectorXd calcDistJacobian(const btVector3& penpt, const btVector3& normal, const VectorXd& pose) {
  VectorXd out(6);
  out.topRows(3) = toVector3d(normal);
  out.bottomRows(3) = toVector3d(normal).transpose()*rotJacWorld(toVector3d(penpt), pose.topRows(3), pose.bottomRows(3));
  return out;

}

ConvexObjectivePtr TrajOverlapCost::convexify(GRBModel* model) {

  ConvexObjectivePtr out(new ConvexObjective());

  for (int s = 0; s < getSolver()->getNumTimesteps(); ++s) {
    getSolver()->setTimestep(s);

    getSolver()->m_world->performDiscreteCollisionDetection();
    btDispatcher* dispatcher = getSolver()->m_world->getDispatcher();
    int numManifolds = dispatcher->getNumManifolds();
    for (int i = 0; i < numManifolds; ++i) {
      btPersistentManifold* contactManifold = dispatcher->getManifoldByIndexInternal(i);
      int numContacts = contactManifold->getNumContacts();
      btRigidBody* objA = static_cast<btRigidBody*> (contactManifold->getBody0());
      btRigidBody* objB = static_cast<btRigidBody*> (contactManifold->getBody1());
      for (int j = 0; j < numContacts; ++j) {
        btManifoldPoint& pt = contactManifold->getContactPoint(j);
        GRBVar overlap_cost = model->addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS, "overlap_cost"); // positive
        out->m_vars.push_back(overlap_cost);
        // depth = current depth + (anglevar - currentangle) * dpose/dangle * ddist/depose
        // normalWorldOnB points from B to A, so if you move A in that direction, it increases dist
        VectorXd dDist_dPoseA = calcDistJacobian(pt.m_positionWorldOnA, pt.m_normalWorldOnB,
                                                 getSolver()->getPoseValues(objA));
        VectorXd dDist_dPoseB = calcDistJacobian(pt.m_positionWorldOnB, -pt.m_normalWorldOnB,
                                                 getSolver()->getPoseValues(objB));
        GRBLinExpr dDistOfA = makeDerivExpr(dDist_dPoseA, getSolver()->getPoseVars(objA),
                                               getSolver()->getPoseValues(objA)); // XXX CHECK SIGN
        GRBLinExpr dDistOfB = makeDerivExpr(dDist_dPoseB, getSolver()->getPoseVars(objB),
                                               getSolver()->getPoseValues(objB));
        GRBLinExpr overlap = - (dDistOfA + dDistOfB + pt.getDistance()/METERS);
        out->m_exprs.push_back(overlap - overlap_cost);
        out->m_cntNames.push_back("hingefunc");
        out->m_objective += overlap_cost;
      }
    }

  }

  return out;
}

double TrajOverlapCost::evaluate() {

  double out = 0;

  for (int s = 0; s < getSolver()->getNumTimesteps(); ++s) {
    getSolver()->setTimestep(s);

    getSolver()->m_world->performDiscreteCollisionDetection();
    btDispatcher* dispatcher = getSolver()->m_world->getDispatcher();
    int numManifolds = dispatcher->getNumManifolds();
    for (int i = 0; i < numManifolds; ++i) {
      btPersistentManifold* contactManifold = dispatcher->getManifoldByIndexInternal(i);
      int numContacts = contactManifold->getNumContacts();
      for (int j = 0; j < numContacts; ++j) {
        btManifoldPoint& pt = contactManifold->getContactPoint(j);
        out += pospart(-pt.getDistance()/METERS);
      }
    }

  }
  return out;
}

ConvexObjectivePtr TrajDynErrCost::convexify(GRBModel* model) {
  typedef map<btRigidBody*, ExprVector> CO2Force;
  CO2Force netForces;
  ConvexObjectivePtr out(new ConvexObjective());

  for (int s = 0; s < getSolver()->getNumTimesteps(); ++s) {
    getSolver()->setTimestep(s);

    getSolver()->m_world->performDiscreteCollisionDetection();
    btDispatcher* dispatcher = getSolver()->m_world->getDispatcher();
    int numManifolds = dispatcher->getNumManifolds();
    for (int i = 0; i < numManifolds; ++i) {
      btPersistentManifold* contactManifold = dispatcher->getManifoldByIndexInternal(i);
      int numContacts = contactManifold->getNumContacts();
      btRigidBody* objA = static_cast<btRigidBody*> (contactManifold->getBody0());
      btRigidBody* objB = static_cast<btRigidBody*> (contactManifold->getBody1());
      for (int j = 0; j < numContacts; ++j) {
        btManifoldPoint& pt = contactManifold->getContactPoint(j);
        GRBVar fAB = model->addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS, "normal_force");
        out->m_objective += fAB * fAB;
        out->m_vars.push_back(fAB);
        exprInc(netForces[objA],exprMult(toVector3d(pt.m_normalWorldOnB), fAB)); // xxx sign?
        exprDec(netForces[objB],exprMult(toVector3d(pt.m_normalWorldOnB), fAB));
      }
    }

    BOOST_FOREACH(const CO2Force::value_type& co_force, netForces) {
      ExprVector forceErr = exprSub(co_force.second, getSolver()->getVelVars(co_force.first));
      out->m_objective += exprNorm2(forceErr);
    }
  }

  return out;
}

double TrajDynErrCost::evaluate() {
 	assert(0);
	return 0;
}

class ObjectPositionSetter : public StateSetter {
  btRigidBody* m_body;
public:
  int getNumDof() {
    return 3;
  }
  ObjectPositionSetter(btRigidBody* body) :
    m_body(body) {
  }
  void setState(const Eigen::VectorXd& x) {
    btTransform tfCur = m_body->getCenterOfMassTransform();
    m_body->setCenterOfMassTransform(btTransform(tfCur.getRotation(), btVector3(x(0), x(1), x(2))));
  }
  Eigen::VectorXd getState() {
    btVector3 pos = m_body->getCenterOfMassPosition();
    return toVector3d(pos);
  }
};

StateSetterPtr makeStateSetter(vector<btRigidBody*>& bodies) {
  assert (TrajDynSolver::POSE_DIM==3);
  ComboStateSetterPtr combo(new ComboStateSetter());
  BOOST_FOREACH(btRigidBody* o, bodies) {
    combo->addSetter(StateSetterPtr(new ObjectPositionSetter(o)));
  }
  return combo;
}

//////////////////////

DynSolver::DynSolver(btCollisionWorld* world) :
  m_world(world), m_initialized(false) {
}

btVector3 toBulletVector(const Eigen::Vector3d& in) {
  return btVector3(in(0), in(1), in(2));
}

void DynSolver::updateValues() {
  BOOST_FOREACH(btRigidBody* body, m_bodies) {
    setValsToVars(m_obj2poseVars[body], m_obj2poses[body]);
  }
  updateBodies();
}

void DynSolver::getPosesFromWorld() {
  BOOST_FOREACH(btRigidBody* body, m_bodies) {
    m_obj2prevPoses[body] = toXYZROD(body->getCenterOfMassTransform(), m_obj2prevPoses[body]);
  }
}

void DynSolver::updateBodies() {
  BOOST_FOREACH(btRigidBody* body, m_bodies) {
    btTransform tf = fromXYZROD(m_obj2poses[body]);
    body->setCenterOfMassTransform(tf);
    body->getMotionState()->setWorldTransform(tf);

  }
}

void DynSolver::storeValues() {
  m_obj2poses_backup = m_obj2poses;
}
void DynSolver::rollbackValues() {
  m_obj2poses = m_obj2poses_backup;
  updateBodies();
}

double DynSolver::fixPermanentVarsAndOptimize() {
  vector<GRBConstr> cnts;

  BOOST_FOREACH(btRigidBody* o, m_bodies) {
    VarVector& poseVars = m_obj2poseVars[o];
    VectorXd& poses = m_obj2poses[o];
    for (int i = 0; i < poses.rows(); ++i) {
        cnts.push_back(m_model->addConstr(poseVars[i] == poses[i]));
      }
    }

  m_model->optimize();
  double out = getApproxObjective();

  BOOST_FOREACH(GRBConstr& cnt, cnts) {
    m_model->remove(cnt);
  }

  return out;
}


void DynSolver::initialize() {
  m_initialized = true;
}

void DynSolver::addObject(btRigidBody* o, const string& name) {
  assert(!m_initialized);
  m_bodies.push_back(o);
  m_obj2name[o] = name;
  m_obj2poseVars[o] = VarVector(POSE_DIM);
  m_obj2poses[o] = toXYZROD(o->getCenterOfMassTransform());
  m_obj2prevPoses[o] = toXYZROD(o->getCenterOfMassTransform());
  VarVector& poseVars = m_obj2poseVars[o];

  for (int j = 0; j < POSE_DIM; ++j) {
    char namebuf[20];
    sprintf(namebuf, "pos_%s_%i", name.c_str(), j);
    poseVars[j] = m_model->addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, namebuf);
  }
  m_model->update();
}

void DynSolver::constrainPose(btRigidBody* o) {
  release(o);
  vector<GRBConstr>& holdCnts = m_holdCnts[o];
  for (int j = 0; j < POSE_DIM; ++j) {
    holdCnts.push_back(m_model->addConstr(getPoseVars(o)[j] == m_obj2prevPoses[o][j]));
  }
}
void DynSolver::release(btRigidBody* o) {
  BOOST_FOREACH(GRBConstr cnt, m_holdCnts[o]) {
    m_model->remove(cnt);
  }
  m_holdCnts[o].clear();
}

VarVector& DynSolver::getPoseVars(btRigidBody* o) {
  return m_obj2poseVars[o];
}
VectorXd& DynSolver::getPoseValues(btRigidBody* o) {
  return m_obj2poses[o];
}
ExprVector DynSolver::getVelVars(btRigidBody* o) {
  return exprSub(m_obj2poseVars[o], m_obj2prevPoses[o]);
}

DynComponent::DynComponent(DynSolver* solver) :
  m_solver(solver) {
}


VelCost::VelCost(DynSolver* solver, double coeff) :
  DynComponent(solver) {
  m_weights.resize(6);
  m_weights << 1,1,1,.3,.3,.3;
  m_weights *= coeff;
}

ConvexObjectivePtr VelCost::convexify(GRBModel* model) {
  ConvexObjectivePtr out(new ConvexObjective());
  BOOST_FOREACH(btRigidBody* body, getSolver()->m_bodies) {
    out->m_objective += exprNorm2(
        exprMult(exprSub(getSolver()->m_obj2poseVars[body], getSolver()->m_obj2prevPoses[body]),
                 m_weights));
  }
  return out;
}

VectorXd modTwoPi(const VectorXd& x) {
  VectorXd out = x;
  for (int i=0; i < out.size(); ++i) {
    out(i) = unwrap_angle(x(i), 0);
  }
  return out;
}

double VelCost::evaluate() {
  double out=0;
  BOOST_FOREACH(btRigidBody* body, getSolver()->m_bodies) {
    out +=  modTwoPi(m_weights.cwiseProduct(getSolver()->m_obj2poses[body] -  getSolver()->m_obj2prevPoses[body])).squaredNorm();
  }
  return out;
}


DynErrCost::DynErrCost(DynSolver* solver, double normalForceCoeff, double forceErrCoeff) :
  DynComponent(solver), m_normalForceCoeff(normalForceCoeff), m_forceErrCoeff(forceErrCoeff) {
}

ConvexObjectivePtr DynErrCost::convexify(GRBModel* model) {

  typedef map<btRigidBody*, ExprVector> CO2Force;
  CO2Force netForces;
  BOOST_FOREACH(btRigidBody* body, getSolver()->m_bodies) netForces[body] = ExprVector(DynSolver::POSE_DIM);
  ConvexObjectivePtr out(new ConvexObjective());

  getSolver()->m_world->performDiscreteCollisionDetection();
  btDispatcher* dispatcher = getSolver()->m_world->getDispatcher();
  int numManifolds = dispatcher->getNumManifolds();
  LOG_INFO_FMT("%i overlaps", numManifolds);
  for (int i = 0; i < numManifolds; ++i) {
    btPersistentManifold* contactManifold = dispatcher->getManifoldByIndexInternal(i);
    int numContacts = contactManifold->getNumContacts();
    btRigidBody* objA = static_cast<btRigidBody*> (contactManifold->getBody0());
    btRigidBody* objB = static_cast<btRigidBody*> (contactManifold->getBody1());
    for (int j = 0; j < numContacts; ++j) {
      btManifoldPoint& pt = contactManifold->getContactPoint(j);
      GRBVar fAB = model->addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "normal_force");
      out->m_objective += m_normalForceCoeff * fAB * fAB;
      out->m_vars.push_back(fAB);
      exprInc(netForces[objA], exprMult(toVector3d(pt.m_normalWorldOnB), fAB)); // xxx sign?
      exprInc(netForces[objB], exprMult(toVector3d(-pt.m_normalWorldOnB), fAB));

    }
  }

  BOOST_FOREACH(btRigidBody* body, getSolver()->m_bodies) {
    for (int i = 0; i < DynSolver::POSE_DIM; ++i) {
      ExprVector forceErr = exprSub(netForces[body], getSolver()->getVelVars(body));
      assert (forceErr.size() == 3);
      out->m_objective += m_forceErrCoeff * exprNorm2(forceErr);

    }
  }

  return out;
}

double DynErrCost::evaluate() {
  return 0;
}

DynOverlapCost::DynOverlapCost(DynSolver* solver, double overlapCoeff) :
  DynComponent(solver), m_overlapCoeff(overlapCoeff) {
}

ConvexObjectivePtr DynOverlapCost::convexify(GRBModel* model) {

  ConvexObjectivePtr out(new ConvexObjective());

  getSolver()->m_world->performDiscreteCollisionDetection();
  btDispatcher* dispatcher = getSolver()->m_world->getDispatcher();
  int numManifolds = dispatcher->getNumManifolds();
  for (int i = 0; i < numManifolds; ++i) {
    btPersistentManifold* contactManifold = dispatcher->getManifoldByIndexInternal(i);
    int numContacts = contactManifold->getNumContacts();
    btRigidBody* objA = static_cast<btRigidBody*> (contactManifold->getBody0());
    btRigidBody* objB = static_cast<btRigidBody*> (contactManifold->getBody1());
    for (int j = 0; j < numContacts; ++j) {
      btManifoldPoint& pt = contactManifold->getContactPoint(j);
      GRBVar overlap_cost = model->addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS, "overlap_cost"); // positive
      out->m_vars.push_back(overlap_cost);
      // depth = current depth + (anglevar - currentangle) * dpose/dangle * ddist/depose
      //normalWorldOnB points from B towards A
      // or the
      VectorXd dDist_dPoseA = calcDistJacobian(pt.m_positionWorldOnA, pt.m_normalWorldOnB,
                                               getSolver()->getPoseValues(objA));
      VectorXd dDist_dPoseB = calcDistJacobian(pt.m_positionWorldOnB, -pt.m_normalWorldOnB,
                                               getSolver()->getPoseValues(objB));
      GRBLinExpr dDistOfA = makeDerivExpr(dDist_dPoseA, getSolver()->getPoseVars(objA),
                                             getSolver()->getPoseValues(objA)); // XXX CHECK SIGN
      GRBLinExpr dDistOfB = makeDerivExpr(dDist_dPoseB, getSolver()->getPoseVars(objB),
                                             getSolver()->getPoseValues(objB));
//      cout << "dDist_dPoseA " << dDist_dPoseA.transpose() << endl;
//      cout << "dDist_dPoseB " << dDist_dPoseB.transpose() << endl;

      GRBLinExpr overlap = -(dDistOfA + dDistOfB + pt.getDistance()/METERS);
      out->m_exprs.push_back(overlap - overlap_cost);
      out->m_cntNames.push_back("hingefunc");
      out->m_objective += m_overlapCoeff * overlap_cost;
    }
  }

  return out;
}

double DynOverlapCost::evaluate() {
  double out = 0;

  getSolver()->m_world->performDiscreteCollisionDetection();
  btDispatcher* dispatcher = getSolver()->m_world->getDispatcher();
  int numManifolds = dispatcher->getNumManifolds();
  for (int i = 0; i < numManifolds; ++i) {
    btPersistentManifold* contactManifold = dispatcher->getManifoldByIndexInternal(i);
    int numContacts = contactManifold->getNumContacts();
    for (int j = 0; j < numContacts; ++j) {
      btManifoldPoint& pt = contactManifold->getContactPoint(j);
      out += m_overlapCoeff * pospart(-pt.getDistance()/METERS);
    }
  }
  return out;
}

DynTrustRegion::DynTrustRegion(DynSolver* solver) : DynComponent(solver) {
  m_maxDiffPerIter = VectorXd::Constant(DynSolver::POSE_DIM, .2);
}

ConvexConstraintPtr DynTrustRegion::convexify(GRBModel* model) {
  for (int j = 0; j < DynSolver::POSE_DIM; ++j) {
    BOOST_FOREACH(btRigidBody* o, getSolver()->m_bodies) {
      getSolver()->getPoseVars(o)[j].set(GRB_DoubleAttr_LB,
           getSolver()->getPoseValues(o)[j] - m_maxDiffPerIter[j]);
      getSolver()->getPoseVars(o)[j].set(GRB_DoubleAttr_UB,
           getSolver()->getPoseValues(o)[j] + m_maxDiffPerIter[j]);
    }
  }
  return ConvexConstraintPtr(new ConvexConstraint());
}

void DynTrustRegion::adjustTrustRegion(double ratio) {
  m_maxDiffPerIter *= ratio;
  m_shrinkage *= ratio;
}
