#include "kinematics_utils.h"
#include "simulation/bullet_io.h"
#include <boost/foreach.hpp>
#include "simulation/openravesupport.h"
#include <opencv2/calib3d/calib3d.hpp>
#include "utils_sqp.h"
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

void ArmPrinter::printJoints() {
  cout << "left joints: " << commaSep(m_left->getDOFValues()) << " right: " << commaSep(m_right->getDOFValues())
      << endl;
}
void ArmPrinter::printCarts() {
  cout << "right joints: " << m_left->getTransform() << "right: " << m_right->getTransform() << endl;
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
