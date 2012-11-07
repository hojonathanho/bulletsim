#include "state_setter.h"
#include "utils/logging.h"
#include "utils/clock.h"
#include "kinematics_utils.h"
#include "utils_sqp.h"
#include "simulation/bullet_io.h"
#include "simulation/openravesupport.h"
using namespace Eigen;

RobotJointSetter::RobotJointSetter(RaveRobotObjectPtr robot, const std::vector<int>& dofInds, bool useAffine) :
  m_robot(robot->robot), m_dofInds(dofInds), m_useAffine(useAffine) {
  std::vector<OpenRAVE::KinBody::LinkPtr> links = getAffectedLinks(robot->robot, dofInds);
  vector<btRigidBody*> bodies;
  links.clear();
  BOOST_FOREACH(const OpenRAVE::KinBody::LinkPtr& link, robot->robot->GetLinks()){
    if (robot->associatedObj(link)){
      bodies.push_back(robot->associatedObj(link)->rigidBody.get());
      links.push_back(link);
    }
  }
  m_brs.reset(new BulletRaveSyncher(links, bodies));
}

void RobotJointSetter::setState(const Eigen::VectorXd& state) {
  if (m_useAffine) {
    m_robot->SetActiveDOFs(m_dofInds, DOF_X | DOF_Y | DOF_RotationAxis, OpenRAVE::RaveVector<double>(0,0,1));
  }
  else {
    m_robot->SetActiveDOFs(m_dofInds);
  }
  m_robot->SetActiveDOFValues(toDoubleVec(state));
  m_brs->updateBullet(true);
}

VectorXd RobotJointSetter::getState() {
  if (m_useAffine) {
    m_robot->SetActiveDOFs(m_dofInds, DOF_X | DOF_Y | DOF_RotationAxis, OpenRAVE::RaveVector<double>(0,0,1));
  }
  else {
    m_robot->SetActiveDOFs(m_dofInds);
  }
  vector<double> x;
  m_robot->GetActiveDOFValues(x);

  return toVectorXd(x);
}


int ComboStateSetter::getNumDof() {
  int sum = 0;
  BOOST_FOREACH(StateSetterPtr& ss, m_setters)
    sum += ss->getNumDof();
  return sum;
}

void ComboStateSetter::setState(const Eigen::VectorXd& state) {
  int iStart = 0;
  BOOST_FOREACH(StateSetterPtr& ss, m_setters) {
    int iEnd = iStart + ss->getNumDof();
    ss->setState(state.middleRows(iStart, iEnd - iStart));
    iStart = iEnd;
  }
}

VectorXd ComboStateSetter::getState() {
  int iStart=0;
  VectorXd out(getNumDof());
  BOOST_FOREACH(StateSetterPtr& ss, m_setters) {
    int iEnd = iStart + ss->getNumDof();
    out.middleRows(iStart, iEnd-iStart) = ss->getState();
    iStart = iEnd;
  }
  return out;
}

void ObjectPoseSetter::setState(const Eigen::VectorXd& state) {
  m_body->setCenterOfMassTransform(fromXYZROD(state));
  m_body->getMotionState()->setWorldTransform(fromXYZROD(state));
}

VectorXd ObjectPoseSetter::getState() {
  return toXYZROD(m_body->getCenterOfMassTransform());
}
