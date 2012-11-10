#pragma once
#include "sqp_fwd.h"
#include "simulation/simulation_fwd.h"
#include <Eigen/Dense>
#include <openrave/openrave.h>
#include <LinearMath/btTransform.h>
#include <btBulletDynamicsCommon.h>
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using std::vector;
using namespace OpenRAVE;

Eigen::VectorXd toXYZROD(const btTransform& tf);
btTransform fromXYZROD(const Eigen::VectorXd& xyzrod);
void setTransformFromXYZROD(btRigidBody* body, const Eigen::VectorXd& xyzrod);
Matrix3d rotJacWorld(const Vector3d& ptWorld, const Vector3d& centerWorld, const Vector3d& rod);

inline Vector3d toVector3d(const RaveVector<double>& p) {
  return Vector3d(p.x, p.y, p.z);
}
inline Vector4d toQuatVector4d(const RaveVector<double>& p) {
  return Vector4d(p.y, p.z, p.w, p.x);
}
OpenRAVE::Transform toRaveTransform(const btQuaternion& q, const btVector3& p);
OpenRAVE::Transform toRaveTransform(double x, double y, double a);
Eigen::MatrixXd makeTraj(const Eigen::VectorXd& startJoints, const Eigen::VectorXd& endJoints, int nSteps);

void getJointLimits(const RobotBasePtr& robot, const vector<int>& dofInds, VectorXd& lower, VectorXd& upper);

std::vector<KinBody::JointPtr> getArmJoints(OpenRAVE::RobotBase::ManipulatorPtr manip);
std::vector<KinBody::LinkPtr> getArmLinks(OpenRAVE::RobotBase::ManipulatorPtr manip);
std::vector<KinBody::LinkPtr> getAffectedLinks(OpenRAVE::RobotBasePtr robot, const std::vector<int>& dofInds);
void getAffectedLinks2(RaveRobotObject* rro, vector<KinBody::LinkPtr>& links, vector<int>& linkInds);
void getAffectedLinks2(RobotBasePtr robot, const vector<int>& dofInds,
                       vector<KinBody::LinkPtr>& links, vector<int>& linkInds);
int getRobotLinkIndex(RobotBasePtr robot, KinBody::LinkPtr link);

void setDofVals(RobotBasePtr robot,  const vector<int>& dofInds, const VectorXd& dofVals);
void setDofVals(RobotBasePtr robot,  const vector<int>& dofInds, const VectorXd& dofVals, const Vector3d& affVals);

VectorXd genRandomDofVals(RobotBasePtr, const vector<int>& dofInds);
Vector3d genRandomWaypoint(const Vector3d& start, const Vector3d& end);


Eigen::MatrixXd calcPointJacobian(const RobotBasePtr& robot, int linkInd, const btVector3& pt, bool useAffine);
void calcActiveLinkJac(const VectorXd& dofvals, KinBody::Link* link, RobotBasePtr robot, MatrixXd& posjac, MatrixXd& rotjac, bool useAffine);

class BulletRaveSyncher {
public:
  std::vector<OpenRAVE::KinBody::LinkPtr> m_links; // what links are part of the object we're planning with
  std::vector<btRigidBody*> m_bodies; // what rigid bodies are part of the object we're planning with
  BulletRaveSyncher(const std::vector<OpenRAVE::KinBody::LinkPtr>& links, const std::vector<btRigidBody*>& bodies) :
    m_links(links), m_bodies(bodies) {}
  void updateBullet(bool updateMotionState=false /*i.e., do you want to render it*/);
};


std::vector<btVector3> getGripperPositions(const Eigen::MatrixXd& traj, RobotManipulatorPtr rrom);

std::vector<btTransform> getGripperPoses(const Eigen::MatrixXd& traj, RobotManipulatorPtr rrom);

BulletRaveSyncherPtr syncherFromArm(RobotManipulatorPtr rrom);
BulletRaveSyncherPtr fullBodySyncher(RaveRobotObject* rro);

vector<KinBody::LinkPtr> fullBodyCollisionLinks(RaveRobotObject* rro);


class ArmPrinter {
public:
  static std::string commaSep(std::vector<double> v);

  RobotManipulatorPtr m_left, m_right;
  ArmPrinter(RobotManipulatorPtr left, RobotManipulatorPtr right) :
  m_left(left), m_right(right) {}
  void printJoints();
  void printCarts();
  void printAll();
};


void removeBodiesFromBullet(vector<BulletObjectPtr> objs, btDynamicsWorld* world);


