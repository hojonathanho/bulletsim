#pragma once
#include "simulation/environment.h"
#include "sqp_fwd.h"
#include <Eigen/Dense>
#include <openrave/openrave.h>
#include "simulation/openravesupport.h"


Eigen::VectorXd toXYZROD(const btTransform& tf);
btTransform fromXYZROD(const Eigen::VectorXd& xyzrod);
void setTransformFromXYZROD(btRigidBody* body, const Eigen::VectorXd& xyzrod);


void getArmKinInfo(const OpenRAVE::RobotBasePtr& robot, const OpenRAVE::RobotBase::ManipulatorPtr manip, std::vector<OpenRAVE::KinBody::LinkPtr>& armLinks, std::vector<OpenRAVE::KinBody::JointPtr>& armJoints, std::vector<int>& chainDepthOfBodies);

std::vector<OpenRAVE::KinBody::JointPtr> getArmJoints(OpenRAVE::RobotBase::ManipulatorPtr manip);
std::vector<OpenRAVE::KinBody::LinkPtr> getArmLinks(OpenRAVE::RobotBase::ManipulatorPtr manip);
std::vector<OpenRAVE::KinBody::LinkPtr> getAffectedLinks(OpenRAVE::RobotBasePtr robot, const std::vector<int>& dofInds);


class BulletRaveSyncher {
public:
  std::vector<OpenRAVE::KinBody::LinkPtr> m_links; // what links are part of the object we're planning with
  std::vector<btRigidBody*> m_bodies; // what rigid bodies are part of the object we're planning with
  BulletRaveSyncher(const std::vector<OpenRAVE::KinBody::LinkPtr>& links, const std::vector<btRigidBody*>& bodies) :
    m_links(links), m_bodies(bodies) {}
  void updateBullet(bool updateMotionState=false /*i.e., do you want to render it*/);
};


std::vector<btVector3> getGripperPositions(const Eigen::MatrixXd& traj, RaveRobotObject::Manipulator::Ptr rrom);

std::vector<btTransform> getGripperPoses(const Eigen::MatrixXd& traj, RaveRobotObject::Manipulator::Ptr rrom);

BulletRaveSyncherPtr syncherFromArm(RaveRobotObject::Manipulator::Ptr rrom);
BulletRaveSyncherPtr fullBodySyncher(RaveRobotObject* rro);

class ArmPrinter {
public:
  static std::string commaSep(std::vector<double> v);

  RaveRobotObject::Manipulator::Ptr m_left, m_right;
  ArmPrinter(RaveRobotObject::Manipulator::Ptr left, RaveRobotObject::Manipulator::Ptr right) :
  m_left(left), m_right(right) {}
  void printJoints();
  void printCarts();
  void printAll();
};


void removeBodiesFromBullet(vector<BulletObject::Ptr> objs, btDynamicsWorld* world);


// unfortunately there doesn't seem to be a way to find out which link is grabbing which
// so i resort to hackery
KinBody::LinkPtr getGrabberLink(KinBodyPtr body);
void setGrabberLink(KinBody::LinkPtr grabber, KinBodyPtr body);
