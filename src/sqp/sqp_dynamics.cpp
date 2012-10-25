#include "sqp_dynamics.h"
#include "sqp.h"
#include <btBulletDynamicsCommon.h>
#include "state_setter.h"
#include <boost/foreach.hpp>

template <typename T>
vector<T> sliceVec(const vector<T>& in, int start, int end) {
  return vector<T>(in.begin()+start, in.begin()+end);
}

ExprVector operator*(const GRBVar& v, const btVector3& x) {
  ExprVector out(3);
  for (int i=0; i < 3; ++i) {
    out[i] = v*x.m_floats[i];
  }
  return out;
}

ExprVector operator-(const ExprVector& v0, const ExprVector& v1) {
  assert (v0.size() == v1.size());
  ExprVector out(v0.size());
  for (int i=0; i < v0.size(); ++i) {
    out[i] = v0[i] - v1[i];
  }
  return out;
}
ExprVector operator+(const ExprVector& v0, const ExprVector& v1) {
  assert (v0.size() == v1.size());
  ExprVector out(v0.size());
  for (int i=0; i < v0.size(); ++i) {
    out[i] = v0[i] + v1[i];
  }
  return out;
}


GRBQuadExpr length2(ExprVector& v) {
  GRBQuadExpr out(0);
  for (int i=0; i < v.size(); ++i) out += v[i] * v[i];
  return out;
}

double DynamicsSolver::sumObjectives(vector<ConvexObjectivePtr>& objectives) {
	
	GRBQuadExpr objective(0);
	
	BOOST_FOREACH(ConvexObjectivePtr& part, objectives) {
		part->addToModel();
		objective += part->m_objective;
	}
	double out = fixPermanentVarsAndOptimize();

	BOOST_FOREACH(ConvexObjectivePtr& part, objectives) {
		part->removeFromModel();
	}
		
	return out;
}


void DynamicsSolver::updateValues() {
  BOOST_FOREACH(btRigidBody* o, m_bodies) {
    VarArray& poseVars = m_obj2poseVars[o];
    MatrixXd& poses = m_obj2poses[o];
    for (int i=0; i < poses.rows(); ++i) {
      for (int j=0; j < poses.cols(); ++j) {
        poses(i,j) = poseVars(i,j).get(GRB_DoubleAttr_X);
      }
    }
  }
}


void DynamicsSolver::storeValues() {
  m_obj2poses_backup = m_obj2poses;
}

void DynamicsSolver::rollbackValues() {
  m_obj2poses = m_obj2poses_backup;
}


double DynamicsSolver::fixPermanentVarsAndOptimize() {
  vector<GRBConstr> m_cnts;


  BOOST_FOREACH(btRigidBody* o, m_bodies) {
    VarArray& poseVars = m_obj2poseVars[o];
    MatrixXd& poses = m_obj2poses[o];
    for (int i=0; i < poses.rows(); ++i) {
      for (int j=0; j < poses.cols(); ++j) {
        m_cnts.push_back(m_model->addConstr(poseVars(i,j) == poses(i,j)));
      }
    }
  }

  m_model->optimize();
  double out = getApproxObjective();

  BOOST_FOREACH(GRBConstr& cnt, m_cnts) {
    m_model->remove(cnt);
  }

  return out;
}


VarVector DynamicsSolver::getPoseVars(btRigidBody* o, int t) {
  if (t==CW_TIME) t = m_t;
  return m_obj2poseVars[o].row(t);
}

VectorXd DynamicsSolver::getPoseValues(btRigidBody* o, int t) {
  if (t==CW_TIME) t = m_t;
  return m_obj2poses[o].row(t);
}

ExprVector DynamicsSolver::getVelVars(btRigidBody* o, int t) {
  if (t==CW_TIME) t = m_t;
  int thi, tlo;
  if (t==getNumTimesteps()-1) {
    thi=t;
    tlo=t-1;
  }
  else {
    thi=t+1;
    tlo=t;
  }
  ExprVector out(POSE_DIM);
  VarArray& poses = m_obj2poseVars[o];
  for (int i=0; i < POSE_DIM; ++i) {
    out[i] = poses(thi,i) - poses(tlo,i);
  }
  return out;
}

VectorXd DynamicsSolver::getVelValues(btRigidBody* o, int t) {
  if (t==CW_TIME) t = m_t;
  int thi, tlo;
  if (t==getNumTimesteps()-1) {
    thi=t;
    tlo=t-1;
  }
  else {
    thi=t+1;
    tlo=t;
  }
  MatrixXd& poses = m_obj2poses[o];
  return poses.row(thi) - poses.row(tlo);
}

void DynamicsSolver::setTimestep(int t) {
  VectorXd stateVec(m_bodies.size()*POSE_DIM);
  int iStart = 0;
  BOOST_FOREACH(btRigidBody* o, m_bodies) {
    stateVec.middleRows(iStart, POSE_DIM) = m_obj2poses[o].row(t);
    iStart += POSE_DIM;
  }
  m_ss->setState(stateVec);
  m_t = t;
}

int DynamicsSolver::getNumTimesteps() {
  return m_numTimesteps;
}


VectorXd DynamicsSolver::getPose(btRigidBody* o) {
  return toVector3d(o->getCenterOfMassPosition());
}

void DynamicsSolver::addObject(btRigidBody* o) {
  assert(POSE_DIM==3);
  addObject(o, toVector3d(o->getCenterOfMassPosition()));
}

void DynamicsSolver::addObject(btRigidBody* o, const VectorXd& initPose) {
  assert(!m_initialized);
  m_bodies.push_back(o);
  m_obj2poseVars[o] = VarArray(getNumTimesteps(), POSE_DIM);
  m_obj2poses[o] = MatrixXd::Zero(getNumTimesteps(), POSE_DIM);
  VarArray& poseVars = m_obj2poseVars[o];

  for (int i=0; i < getNumTimesteps(); ++i) {
    for (int j=0; j < POSE_DIM; ++j) {
      char namebuf[10];
      sprintf(namebuf, "j_%i_%i", i,j);
      poseVars(i,j) = m_model->addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, namebuf);
    }
  }
  m_model->update();
}


void DynamicsSolver::setStatic(btRigidBody* o) {
  assert(!m_initialized);
  assert(m_obj2poses.find(o) != m_obj2poses.end());
  constrainPoses(o, m_obj2poses[o]); // assuming poses were initialized with current position
}

void DynamicsSolver::constrainPoses(btRigidBody* o, const MatrixXd& poses) {
  assert(!m_initialized);
  VarArray& poseVars = m_obj2poseVars[o];
  for (int i=0; i < poses.rows(); ++i) {
    for (int j=0; j < poseVars.cols(); ++j) {
      m_model->addConstr(poseVars(i,j) == poses(i,j));
    }
  }
}

DynamicsSolver::DynamicsSolver(btCollisionWorld* world, int numTimesteps) :
    m_world(world),
    m_t(0),
    m_numTimesteps(numTimesteps),
    m_initialized(false)
{}

void DynamicsSolver::initialize() {
  m_initialized=true;
}

////////////////////

DynamicsSolver* DynamicsCost::getSolver() {
  return m_solver;
}

GRBLinExpr makeDerivExpr(const VectorXd& grad, const VarVector& vars, const VectorXd& curvals) {
  GRBLinExpr out;
  out.addTerms(grad.data(), vars.data(), curvals.size());
  out -= grad.dot(curvals);
  return out;
}

VectorXd calcDistJacobian(const btVector3& penpt, const btVector3& normal, const VectorXd& pose) {
  assert (DynamicsSolver::POSE_DIM == 3);
  return - toVector3d(normal);

}

ConvexObjectivePtr OverlapCost::convexify() {

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
        GRBVar overlap_cost = m_model->addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS, "overlap_cost"); // positive
        out->m_vars.push_back(overlap_cost);
        // depth = current depth + (anglevar - currentangle) * dpose/dangle * ddist/depose
        VectorXd dDist_dPoseA = calcDistJacobian(pt.m_positionWorldOnA, -pt.m_normalWorldOnB, getSolver()->getPoseValues(objA));
        VectorXd dDist_dPoseB = calcDistJacobian(pt.m_positionWorldOnB, pt.m_normalWorldOnB, getSolver()->getPoseValues(objB));
        GRBLinExpr dOverlapOfA = makeDerivExpr(dDist_dPoseA, getSolver()->getPoseVars(objA), getSolver()->getPoseValues(objA)); // XXX CHECK SIGN
        GRBLinExpr dOverlapOfB = makeDerivExpr(dDist_dPoseB, getSolver()->getPoseVars(objB), getSolver()->getPoseValues(objB));
        GRBLinExpr overlap = dOverlapOfA + dOverlapOfB + pt.getDistance();
        out->m_exprs.push_back(overlap_cost - overlap);
        out->m_cntNames.push_back("hingefunc");
        out->m_objective += overlap_cost;
        out->m_val += pt.getDistance();
      }
    }

  }

  return out;
}

double OverlapCost::evaluate() {

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
        out += pospart(pt.getDistance());
      }
    }

  }
  return out;
}


ConvexObjectivePtr NoFricDynCost::convexify() {
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
        GRBVar fAB = m_model->addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS, "normal_force");
        out->m_objective += fAB * fAB;
        out->m_vars.push_back(fAB);
        netForces[objA] = netForces[objA] + (ExprVector)(fAB * pt.m_normalWorldOnB); // xxx sign?
        netForces[objB] = netForces[objB] - (ExprVector)(fAB * pt.m_normalWorldOnB);
      }
    }

    BOOST_FOREACH(const CO2Force::value_type& co_force, netForces) {
      ExprVector forceErr = co_force.second - getSolver()->getVelVars(co_force.first);
      out->m_objective += length2(forceErr);
    }
  }

  return out;
}

double NoFricDynCost::evaluate() {
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
  assert (DynamicsSolver::POSE_DIM==3);
  ComboStateSetterPtr combo(new ComboStateSetter());
  BOOST_FOREACH(btRigidBody* o, bodies) {
    combo->addSetter(StateSetterPtr(new ObjectPositionSetter(o)));
  }
  return combo;
}
