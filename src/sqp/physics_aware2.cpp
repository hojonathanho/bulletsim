#include "sqp_algorithm.h"
#include "physics_aware2.h"
#include <BulletCollision/NarrowPhaseCollision/btGjkEpa2.h>
#include "kinematics_utils.h"
#include "simulation/bullet_io.h"
#include "state_setter.h"
#include "utils/logging.h"

using namespace Eigen;
using namespace std;

struct ScopedBodySave {
  btRigidBody* m_body;
  btTransform m_tf;
  ScopedBodySave(btRigidBody* body) :
    m_body(body), m_tf(body->getCenterOfMassTransform()) {
  }
  ~ScopedBodySave() {
    m_body->setCenterOfMassTransform(m_tf);
  }
};

#if 0
Vector3d depthPerRotation(const btVector3& center, const btVector3& pt, const btVector3& normal) {
  btVector3 disp = pt - center;
  return -Vector3d(disp.cross(btVector3(1, 0, 0).dot(normal)), disp.cross(btVector3(0, 1, 0).dot(
              normal)), disp.cross(btVector3(0, 0, 1).dot(normal)));
}
#endif

#if 0

JointCollInfo cartToJointObsCollInfo(const CartCollInfo& in, const Eigen::VectorXd& dofVals, RobotBasePtr robot,
    const std::vector<int>& dofInds, btRigidBody* body) {

  ScopedRobotSave srs(robot);
  ScopedBodySave sbs(body);

  robot->SetActiveDOFs(dofInds);
  robot->SetActiveDOFValues(toDoubleVec(dofVals));

  setTransformFromXYZRod(body, dofVals.middleRows(7,6));

  JointCollInfo out;
  out.dists.resize(in.size());
  out.jacs.resize(in.size());

  int nJoints = dofInds.size();

  for (int iColl = 0; iColl < in.size(); ++iColl) {
    const LinkCollision& lc = in[iColl];
    out.dists[iColl] = lc.dist;

    std::vector<double> jacvec(3*nJoints);
    robot->CalculateActiveJacobian(lc.linkInd, toRaveVector(lc.point), jacvec);
    out.jacs[iColl].middleRows(0,7) = - toVector3d(lc.normal).transpose() * Eigen::Map<MatrixXd>(jacvec.data(), 3, nJoints);
    out.jacs[iColl].middleRows(7,3) = toVector3d(lc.normal);
    out.jacs[iColl].middleRows(10,3) = depthPerRotation(body->getCenterOfMassPosition(), lc.point, lc.normal);

  }
  return out;
}

#endif

#if 0

class JacobianCalc {
public:
  virtual TrajJointCollInfo call(const TrajCartCollInfo& in) = 0;
};

class PushObjJac {
public:
  TrajJointCollInfo call(const TrajCartCollInfo& in);
  // first 7 indices will be joints, next three will be rodrigues vector for pose
};

TrajJointCollInfo PushObjJac::call(const TrajCartCollInfo& in, const MatrixXd& in) {
  TrajJointCollInfo out(in.size());
  for (int iStep = 0; iStep < in.size(); ++iStep) {
    out[iStep] = cartToJointObsCollInfo(in[iStep], traj.row(iStep), m_robot, dofInds);
  }
  return out;
}
#endif

#if 0
void PushCollision::removeVariablesAndConstraints() {
  BOOST_FOREACH(GRBConstr& cnt, m_cnts)
  m_problem ->m_model->remove(cnt);
  BOOST_FOREACH(GRBVar& var, m_vars)
  m_problem->m_model->remove(var);
  m_vars.clear();
  m_cnts.clear();
}
#endif

static inline double sq(double x) {
  return x * x;
}

PushCollision::PushCollision(RaveRobotObjectPtr robot, OpenRAVE::KinBody::LinkPtr link,
                             btRigidBody* body, std::vector<int> dofInds) :
  m_robot(robot->robot), m_body(body), m_linkBody(robot->associatedObj(link)->rigidBody.get()), m_dofInds(dofInds) {

  m_setter.reset(new ComboStateSetter());
  m_setter->addSetter(StateSetterPtr(new RobotJointSetter(robot,dofInds)));
  m_setter->addSetter(StateSetterPtr(new ObjectPoseSetter(body)));

  m_bodyShape = dynamic_cast<btConvexShape*> (body->getCollisionShape());
  assert(m_bodyShape);
  btCompoundShape* linkCompoundShape = dynamic_cast<btCompoundShape*> (m_linkBody->getCollisionShape());
  assert(linkCompoundShape);
  m_linkBodyShape = dynamic_cast<btConvexShape*> (linkCompoundShape->getChildShape(0));
  assert(m_linkBodyShape);
  assert(linkCompoundShape->getNumChildShapes()==1);
}

double PushCollision::calcPenCost(const Eigen::VectorXd& dofs) {
  ScopedStateSave ss(m_setter.get());
  m_setter->setState(dofs);

  btGjkEpaSolver2::sResults results;
  btVector3 comdiff = m_linkBody->getCenterOfMassTransform().getOrigin()
      - m_body->getCenterOfMassTransform().getOrigin();
  btGjkEpaSolver2::SignedDistance(m_linkBodyShape, m_linkBody->getCenterOfMassTransform(), m_bodyShape,
                                  m_body->getCenterOfMassTransform(), comdiff, results);

  if (results.status == btGjkEpaSolver2::sResults::Separated)
    return 0;
  else if (results.status == btGjkEpaSolver2::sResults::Penetrating)
    return 100*sq(results.distance);
  else
  ASSERT_FAIL();
  return 0;
}

double PushCollision::calcViolCost(const Eigen::VectorXd& dofs0, const Eigen::VectorXd& dofs1) {
  ScopedStateSave ss(m_setter.get());
  m_setter->setState(dofs0);

  btGjkEpaSolver2::sResults results0;
  btVector3 bodyPos0 = m_body->getCenterOfMassPosition();
  btVector3 comdiff0 = m_linkBody->getCenterOfMassTransform().getOrigin()
      - m_body->getCenterOfMassTransform().getOrigin();
  btGjkEpaSolver2::SignedDistance(m_linkBodyShape, m_linkBody->getCenterOfMassTransform(), m_bodyShape,
                                  m_body->getCenterOfMassTransform(), comdiff0, results0);

  m_setter->setState(dofs1);
  btGjkEpaSolver2::sResults results1;
  btVector3 bodyPos1 = m_body->getCenterOfMassPosition();
  btVector3 comdiff1 = m_linkBody->getCenterOfMassTransform().getOrigin()
      - m_body->getCenterOfMassTransform().getOrigin();
  btGjkEpaSolver2::SignedDistance(m_linkBodyShape, m_linkBody->getCenterOfMassTransform(), m_bodyShape,
                                  m_body->getCenterOfMassTransform(), comdiff1, results1);

  btVector3 disp = bodyPos1 - bodyPos0;

  if ((results0.status == btGjkEpaSolver2::sResults::Separated) || (results0.status
      == btGjkEpaSolver2::sResults::Penetrating)) {

    double cost0 = pow(disp.length(),3) * pow(pospart(results0.distance),1); //+ 0 * disp.angle(results0.normal * results0.distance));
    double cost1 = pow(disp.length(),3) * pow(pospart(results1.distance),1); //+ 0 * disp.angle(results1.normal * results1.distance));

    return (cost0 + cost1) * 1000;

  }
  //unreachable
  else {
    ASSERT_FAIL();
    return 0;
  }
}

double PushCollision::getCost() {
  ScopedRobotSave srs(m_robot);
  ScopedBodySave sbs(m_body);

  Eigen::MatrixXd& traj = m_problem->m_currentTraj;
  double cost = 0;

  for (int iStep = 0; iStep < traj.rows(); ++iStep)
    if (m_problem->m_optMask(iStep)) {

      double penCost, violCostBefore(0), violCostAfter(0);

      penCost = calcPenCost(traj.row(iStep)); // that's easy, just get the closest point
      if (iStep > 0)
        violCostBefore = calcViolCost(traj.row(iStep - 1), traj.row(iStep));
      if (iStep < traj.rows() - 1)
        violCostAfter = calcViolCost(traj.row(iStep), traj.row(iStep + 1));

      cost += penCost + violCostBefore;
      if (iStep == traj.rows() - 1) {
        cost += violCostAfter;
      }
    }

  return cost;

}

void PushCollision::getCostAndGradient(double& cost, Eigen::MatrixXd& gradient) {
  ScopedRobotSave srs(m_robot);
  ScopedBodySave sbs(m_body);

  Eigen::MatrixXd& traj = m_problem->m_currentTraj;

  gradient = MatrixXd::Zero(traj.rows(), traj.cols());
  cost = 0;

  double epsilon = 1e-4;

  for (int iStep = 0; iStep < traj.rows(); ++iStep)
    if (m_problem->m_optMask(iStep)) {

      double penCost(0), violCostBefore(0), violCostAfter(0), penCost1(0), violCostBefore1(0), violCostAfter1(0);

      penCost = calcPenCost(traj.row(iStep)); // that's easy, just get the closest point
      if (iStep > 0)
        violCostBefore = calcViolCost(traj.row(iStep - 1), traj.row(iStep));
      if (iStep < traj.rows() - 1)
        violCostAfter = calcViolCost(traj.row(iStep), traj.row(iStep + 1));

      cost += penCost + violCostBefore;
      if (iStep == traj.rows() - 1) {
        cost += violCostAfter;
      }

      LOG_INFO_FMT("step: %i. pen %.3e. violbefore: %.3e. violafter: %.3e", iStep, penCost, violCostBefore, violCostAfter);

      for (int iDof = 0; iDof < traj.cols(); ++iDof) {

        VectorXd pertDofs = traj.row(iStep);
        pertDofs(iDof) += epsilon;

        penCost1 = calcPenCost(pertDofs);
        if (iStep > 0)
          violCostBefore1 = calcViolCost(traj.row(iStep - 1), pertDofs);
        if (iStep < traj.rows() - 1)
          violCostAfter1 = calcViolCost(pertDofs, traj.row(iStep + 1));

        gradient(iStep, iDof)
            = ((penCost1 + violCostBefore1 + violCostAfter1) - (penCost + violCostBefore + violCostAfter)) / epsilon;

      }

    }
//#define TEST_COST_AND_GRAD
#ifdef TEST_COST_AND_GRAD
ASSERT_ALMOST_EQUAL2(cost, getCost(), 1e-8, 1e-8);
#endif
}

void PushCollision::updateModel(const Eigen::MatrixXd& traj, GRBQuadExpr& objective) {

  //  removeVariablesAndConstraints();
  // Actually run through trajectory and find collisions

  MatrixXd grad;
  getCostAndGradient(m_exactObjective, grad);
  m_obj = linearizationFromCostAndGrad( m_problem->m_trajVars, m_problem->m_currentTraj,m_exactObjective, grad);

  objective += m_obj;
}

