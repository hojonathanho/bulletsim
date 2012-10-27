#include "kinematics_utils.h"
#include "simulation/bullet_io.h"
#include <boost/foreach.hpp>
#include "simulation/openravesupport.h"
#include <opencv2/calib3d/calib3d.hpp>
#include "utils_sqp.h"
#include "utils/logging.h"
using namespace std;
using namespace Eigen;

Eigen::VectorXd toXYZROD(const btTransform& tf) {
  Eigen::VectorXd out(6);
  btVector3 pos = tf.getOrigin();
  out(0) = pos.x();
  out(1) = pos.y();
  out(2) = pos.z();
  const btMatrix3x3& rot = tf.getBasis();

  cv::Mat_<double> rot0(3, 3);
  rot0 << rot.getRow(0).x(), rot.getRow(0).y(), rot.getRow(0).z(),
      rot.getRow(1).x(), rot.getRow(1).y(), rot.getRow(1).z(),
      rot.getRow(2).x(), rot.getRow(2).y(), rot.getRow(2).z();

  cv::Mat_<double> rod(3, 1);
  cv::Rodrigues(rot0, rod);
  out(3) = rod(0, 0);
  out(4) = rod(1, 0);
  out(5) = rod(2, 0);
  return out;
}

btTransform fromXYZROD(const Eigen::VectorXd& xyzrod) {
  cv::Mat rot0(3, 3, CV_32F);
  cv::Vec3d rvec(xyzrod(3), xyzrod(4), xyzrod(5));
  cv::Rodrigues(rvec, rot0);
  cv::Mat_<float> rot;
  rot0.copyTo(rot);
  btMatrix3x3 basis(btMatrix3x3(rot(0, 0), rot(0, 1), rot(0, 2), rot(1, 0), rot(1, 1), rot(1, 2), rot(2, 0), rot(2, 1),
                                rot(2, 2)));
  btQuaternion q;
  basis.getRotation(q);

  btVector3 t(xyzrod(0), xyzrod(1), xyzrod(2));
  return btTransform(q, t);
}
void setTransformFromXYZROD(btRigidBody* body, const VectorXd& xyzrod) {
  body->setCenterOfMassTransform(fromXYZROD(xyzrod));
}


std::vector<KinBody::JointPtr> getArmJoints(OpenRAVE::RobotBase::ManipulatorPtr manip) {
  std::vector<KinBody::JointPtr> armJoints;
  BOOST_FOREACH(int ind, manip->GetArmIndices()) armJoints.push_back(manip->GetRobot()->GetJointFromDOFIndex(ind));
  return armJoints;
}

std::vector<KinBody::LinkPtr> getArmLinks(OpenRAVE::RobotBase::ManipulatorPtr manip) {
  RobotBasePtr robot = manip->GetRobot();
  int rootLinkInd = manip->GetBase()->GetIndex();
  vector<KinBody::JointPtr> armJoints = getArmJoints(manip);
  KinBody::JointPtr& firstJoint = armJoints[0];

  vector<KinBody::LinkPtr> armLinks;
  BOOST_FOREACH(KinBody::LinkPtr link, robot->GetLinks()) {
    // check chain between link and torso_lift_link
    // see if it contains firstJoint
    vector<KinBody::JointPtr> jointChain;
    robot->GetChain(rootLinkInd, link->GetIndex(), jointChain);
    if (link->GetGeometries().size() && count(jointChain.begin(), jointChain.end(), firstJoint)) armLinks.push_back(link);
  }
  return armLinks;
}

MatrixXd calcPointJacobian(const RobotBasePtr& robot, int linkInd, const btVector3& pt, bool useAffine) {
  int nJoints = robot->GetActiveDOF();
  int ndof = nJoints + 3 * useAffine;
  std::vector<double> jacvec(3 * nJoints);
  robot->CalculateActiveJacobian(linkInd, util::toRaveVector(pt), jacvec);
  OpenRAVE::Transform robotTF = robot->GetTransform();

  MatrixXd jac(3, ndof);
  Matrix3d affineJac = Matrix3d::Identity();
  affineJac(0,2) = -(pt.y() - robotTF.trans.y);
  affineJac(1,2) = pt.x() - robotTF.trans.x;
  affineJac(2,2) = 0;
  jac.leftCols(nJoints) = Eigen::Map<MatrixXd>(jacvec.data(), 3, nJoints);
  jac.rightCols(3) = affineJac;
  return jac;
}



void BulletRaveSyncher::updateBullet(bool updateMotionState) {
  for (int iBody = 0; iBody < m_bodies.size(); ++iBody) {
    btTransform tf = util::toBtTransform(m_links[iBody]->GetTransform(), GeneralConfig::scale);
    m_bodies[iBody]->setCenterOfMassTransform(tf);
    if (updateMotionState) dynamic_cast<BulletObject::MotionState*>(m_bodies[iBody]->getMotionState())->setKinematicPos(tf);
  }
}

string ArmPrinter::commaSep(std::vector<double> v) {
  stringstream ss;
  BOOST_FOREACH(double d, v)
ss  << d << ", ";
  return ss.str();
}

void ArmPrinter::printAll() {
  vector<double> dofvals;
  m_left->robot->robot->GetDOFValues(dofvals);

  BOOST_FOREACH(KinBody::JointPtr joint, m_left->robot->robot->GetJoints()) {
    cout << joint->GetName() << ": " << dofvals[joint->GetDOFIndex()] << endl;
  }
  cout << "all dof vals: " << dofvals << endl;
  cout << "transform: " << m_left->robot->robot->GetTransform() << endl;
}

void ArmPrinter::printJoints() {
  cout << "cart left: " << commaSep(m_left->getDOFValues()) << " right: " << commaSep(m_right->getDOFValues())
      << endl;
}
void ArmPrinter::printCarts() {
  cout << "cart left: " << m_left->getTransform() << "right: " << m_right->getTransform() << endl;
}

std::vector<OpenRAVE::KinBody::LinkPtr> getAffectedLinks(OpenRAVE::RobotBasePtr robot, const std::vector<int>& dofInds) {
  std::vector<OpenRAVE::KinBody::LinkPtr> links = robot->GetLinks();
  std::vector<OpenRAVE::KinBody::LinkPtr> out;
  for (int iLink = 0; iLink < links.size(); ++iLink) {
    for (int iDof = 0; iDof < dofInds.size(); ++iDof) {
      if (robot->DoesAffect(iDof, iLink)) {
        out.push_back(links[iLink]);
        break;
      }
    }
  }
  return out;
}

vector<btVector3> getGripperPositions(const MatrixXd& traj, RaveRobotObject::Manipulator::Ptr rrom) {
  vector<double> dofOrig = rrom->getDOFValues();
  vector<btVector3> out;
  for (int i = 0; i < traj.rows(); ++i) {
    rrom->setDOFValues(toDoubleVec(traj.row(i)));
    out.push_back(rrom->getTransform().getOrigin());
  }
  rrom->setDOFValues(dofOrig);
  return out;
}

vector<btTransform> getGripperPoses(const MatrixXd& traj, RaveRobotObject::Manipulator::Ptr rrom) {
  vector<double> dofOrig = rrom->getDOFValues();
  vector<btTransform> out;
  for (int i = 0; i < traj.rows(); ++i) {
    rrom->setDOFValues(toDoubleVec(traj.row(i)));
    out.push_back(rrom->getTransform());
  }
  rrom->setDOFValues(dofOrig);
  return out;
}

BulletRaveSyncherPtr syncherFromArm(RaveRobotObject::Manipulator::Ptr rrom) {
  vector<KinBody::LinkPtr> armLinks = getArmLinks(rrom->manip);
  vector<btRigidBody*> armBodies;
  BOOST_FOREACH(KinBody::LinkPtr& link, armLinks) {
    armBodies.push_back(rrom->robot->associatedObj(link)->rigidBody.get());
  }

  vector<KinBodyPtr> grabbedBodies;
  rrom->robot->robot->GetGrabbed(grabbedBodies);
  BOOST_FOREACH(KinBodyPtr body, grabbedBodies) {
    RaveObject* robj = rrom->robot->rave->rave2bulletsim[body];
    BOOST_FOREACH(KinBody::LinkPtr link, body->GetLinks()) {
      armLinks.push_back(link);
      armBodies.push_back(robj->associatedObj(link)->rigidBody.get());
    }
  }
  return BulletRaveSyncherPtr(new BulletRaveSyncher(armLinks, armBodies));
}

void removeBodiesFromBullet(vector<BulletObject::Ptr> objs, btDynamicsWorld* world) {
  BOOST_FOREACH(BulletObject::Ptr obj, objs) {
    world->removeRigidBody(obj->rigidBody.get());
  }
}



BulletRaveSyncherPtr fullBodySyncher(RaveRobotObject* rro) {
  RobotBasePtr robot = rro->robot;
  std::set<KinBody::LinkPtr> linkSet;
  for (int i=0; i < rro->numCreatedManips(); ++i) {
    KinBody::LinkPtr eeLink = rro->getManipByIndex(i)->manip->GetEndEffector();
    vector<KinBody::LinkPtr> chain;
    robot->GetChain(0,eeLink->GetIndex(), chain);
    BOOST_FOREACH(KinBody::LinkPtr link, chain) linkSet.insert(link);
  }

  stringstream ss;

  vector<btRigidBody*> bodies;
  vector<KinBody::LinkPtr> links;
  BOOST_FOREACH(KinBody::LinkPtr link, linkSet) {
    BulletObject::Ptr bobj = rro->associatedObj(link);
    if (bobj) {
      bodies.push_back(bobj->rigidBody.get());
      links.push_back(link);
      ss << link->GetName() << " ";
    }
  }
  LOG_DEBUG("synched links: " << ss);
  return BulletRaveSyncherPtr(new BulletRaveSyncher(links, bodies));
}

