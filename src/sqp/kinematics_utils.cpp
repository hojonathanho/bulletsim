#include "kinematics_utils.h"
#include "simulation/bullet_io.h"
#include <boost/foreach.hpp>
#include "simulation/openravesupport.h"
#include <opencv2/calib3d/calib3d.hpp>
#include "utils_sqp.h"
#include "utils/logging.h"
#include "utils/conversions.h"
#include "utils/interpolation.h"
#include <Eigen/Geometry>
#include "functions.h"
using namespace std;
using namespace Eigen;

OpenRAVE::Transform toRaveTransform(const btQuaternion& q, const btVector3& p) {
  return OpenRAVE::Transform(util::toRaveQuaternion(q), util::toRaveVector(p));
}
OpenRAVE::Transform toRaveTransform(double x, double y, double a) {
  return toRaveTransform(btQuaternion(0,0,a), btVector3(x,y,0));
}

MatrixXd createInterpolatedTrajectory(RobotManipulatorPtr manip, const vector<OpenRAVE::Transform>& tfs) {
	RobotBasePtr robot = manip->robot->robot;
	vector<int> armInds = manip->manip->GetArmIndices();
	robot->SetActiveDOFs(armInds);
	LOG_WARN("createInterpolatedTrajectory: returning bullshit answer");
	MatrixXd out(tfs.size(), armInds.size());
	vector<double> lower,upper;
	robot->GetActiveDOFLimits(lower, upper);
	for (int i=0; i < armInds.size(); ++i) {
		out.col(i).setConstant((lower[i] + upper[i])/2);
	}
	vector<double> dofVals;
	robot->GetActiveDOFValues(dofVals);
	out.row(0) = toVectorXd(dofVals);
	return out;
}

Matrix3d rod2mat_cv(const Vector3d& rod0) {
  cv::Mat_<double> rot(3, 3);
  cv::Mat_<double> rod(3, 1);
  rod << rod0(0), rod0(1), rod0(2);
  cv::Rodrigues(rod, rot);
  return Map<Matrix3d>(reinterpret_cast<double*>(rot.data));
}

Vector3d calcPtWorld(const Vector3d& ptLocal, const Vector3d& centerWorld, const Vector3d& rod) {
  return centerWorld + rod2mat_cv(rod) * ptLocal;
}

Matrix3d rotJacLocal(const Vector3d& ptLocal, const Vector3d& rod) {
  struct F : public fVectorOfVector {
    VectorXd m_ptLocal;
    F(const VectorXd& ptLocal1) : m_ptLocal(ptLocal1) {}
    VectorXd operator()(const VectorXd& rod1) const {
      return calcPtWorld(m_ptLocal, Vector3d(0,0,0), rod1);
    }
  };
  return calcJacobian(F(ptLocal), rod);
}

Matrix3d rotJacWorld(const Vector3d& ptWorld, const Vector3d& centerWorld, const Vector3d& rod) {
  Affine3d A;
  A = Eigen::Translation3d(centerWorld) * rod2mat_cv(rod);
  Vector3d ptLocal = A.inverse() * ptWorld;
  return rotJacLocal(ptLocal, rod);
}

Eigen::VectorXd toXYZROD_wrong(const btTransform& tf) {
  Eigen::Affine3d aff = toEigenTransform(tf).cast<double>();
  Eigen::VectorXd out(6);
  out.topRows(3) = aff.translation();
  AngleAxisd aa;
  aa = aff.rotation();
  out.bottomRows(3) = aa.angle() * aa.axis();
  return out;
}


btTransform fromXYZROD_wrong(const Eigen::VectorXd& xyzrod) {
  Eigen::Affine3f T;
  Vector3f rod = xyzrod.bottomRows(3).cast<float>();
  T = Translation3f(xyzrod.topRows(3).cast<float>()) * AngleAxisf(rod.norm(), rod.normalized());
  return toBulletTransform(T);
}


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

Eigen::MatrixXd makeTraj(const Eigen::VectorXd& startJoints, const Eigen::VectorXd& endJoints,
    int nSteps) {
  assert(startJoints.size() == endJoints.size());
  Eigen::MatrixXd startEndJoints(2, startJoints.size());
  startEndJoints.row(0) = startJoints;
  startEndJoints.row(1) = endJoints;
  return interp2d(VectorXd::LinSpaced(nSteps, 0, 1), VectorXd::LinSpaced(2, 0, 1), startEndJoints);

}

void getJointLimits(const RobotBasePtr& robot, const vector<int>& dofInds, VectorXd& lower, VectorXd& upper) {
  lower.resize(dofInds.size());
  upper.resize(dofInds.size());

  vector<double> ul, ll;
  for (int i = 0; i < dofInds.size(); ++i) {
    robot->GetJointFromDOFIndex(dofInds[i])->GetLimits(ll, ul);
    lower(i) = ll[0];
    upper(i) = ul[0];
  }
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

void setDofVals(RobotBasePtr robot,  const vector<int>& dofInds, const VectorXd& dofVals) {
  if (dofInds.size() > 0) robot->SetActiveDOFs(dofInds); //otherwise just use the active dofs
  robot->SetActiveDOFValues(toDoubleVec(dofVals),false);
}

void setDofVals(RobotBasePtr robot,  const vector<int>& dofInds, const VectorXd& dofVals, const Vector3d& affVals) {
  robot->SetTransform(toRaveTransform(affVals(0), affVals(1), affVals(2)));
  setDofVals(robot, dofInds, dofVals);
}

VectorXd genRandomDofVals(RobotBasePtr robot, const vector<int>& dofInds) {
	VectorXd lower, upper;
	getJointLimits(robot, dofInds, lower, upper);
	VectorXd out;
	for (int i=0; i < dofInds.size(); ++i) {
	  out(i) = lower(i) + (upper(i) - lower(i)) * randf();
	}
	return out;
}

Vector3d genRandomWaypoint(const Vector3d& start, const Vector3d& end) {
	double centerx = (end(0) + start(0))/2;
	double centery = (end(1) + start(1))/2;
	double rmax = (end.topRows(2) - start.topRows(2)).norm()/2;
	double r = randf() * rmax;
	double theta = randf() * 2 * SIMD_PI;
	double x = centerx + r * cos(theta);
	double y = centery + r * sin(theta);
	double ang = randf() * 2 * SIMD_PI;
	return Vector3d(x,y,ang);
		
}

MatrixXd calcPointJacobian(const RobotBasePtr& robot, int linkInd, const btVector3& pt, bool useAffine) {
  int njoints = robot->GetActiveDOF();
  int ndof = njoints + 3 * useAffine;
  std::vector<double> jacvec(3 * njoints);
  robot->CalculateActiveJacobian(linkInd, util::toRaveVector(pt), jacvec);
  MatrixXd jac(3, ndof);
  jac.leftCols(njoints) = Eigen::Map<MatrixXd>(jacvec.data(), 3, njoints);

  if (useAffine) {
    OpenRAVE::Transform robotTF = robot->GetTransform();

    Matrix3d affineJac = Matrix3d::Identity();
    affineJac(0,2) = -(pt.y() - robotTF.trans.y);
    affineJac(1,2) = pt.x() - robotTF.trans.x;
    affineJac(2,2) = 0;
    jac.rightCols(3) = affineJac;
  }

  return jac;
}

void calcActiveLinkJac(const VectorXd& dofvals, KinBody::Link* link, RobotBasePtr robot, MatrixXd& posjac, MatrixXd& rotjac, bool useAffine) {
  struct F : public fVectorOfVector {
    KinBody::Link* m_link;
    RobotBasePtr m_robot;
    bool m_useAffine;
    F(KinBody::Link* link, RobotBasePtr robot, bool useAffine) : m_link(link), m_robot(robot), m_useAffine(useAffine) {}
    VectorXd operator()(const VectorXd& vals) const {
      if (m_useAffine) setDofVals(m_robot, vector<int>(), vals.topRows(vals.size()-3), vals.bottomRows(3));
      else setDofVals(m_robot, vector<int>(), vals);
      VectorXd out(7);
      OpenRAVE::Transform tf = m_link->GetTransform();
      out.topRows(3) = toVector3d(tf.trans);
      out.bottomRows(4) = toQuatVector4d(tf.rot);
      return out;
    }
  };

  ScopedRobotSave srs(robot);
  setDofVals(robot, vector<int>(), dofvals);
  assert (link != NULL);
  MatrixXd fulljac = calcJacobian(F(link, robot, useAffine), dofvals, 1e-5);
  posjac = fulljac.topRows(3);
  rotjac = fulljac.bottomRows(4);

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

bool doesAffect(const RobotBasePtr& robot, const vector<int>& dofInds, int linkInd) {
  BOOST_FOREACH(int dofInd, dofInds) {
    if (robot->DoesAffect(dofInd, linkInd)) return true;
  }
  return false;
}

int getRobotLinkIndex(RobotBasePtr robot, KinBody::LinkPtr link) {
  if (link->GetParent() == robot) return link->GetIndex();
  else {
    return robot->IsGrabbing(link->GetParent())->GetIndex();
  }
}

void getAffectedLinks2(RaveRobotObject* rro, vector<KinBody::LinkPtr>& links, vector<int>& linkInds) {
  links.clear();
  linkInds.clear();
  RobotBasePtr robot = rro->robot;
  links = fullBodyCollisionLinks(rro);
  BOOST_FOREACH(KinBody::LinkPtr& link, links) linkInds.push_back(link->GetIndex());

  vector<KinBodyPtr> grabbed;
  robot->GetGrabbed(grabbed);
  BOOST_FOREACH(const KinBodyPtr& body, grabbed) {
    KinBody::LinkPtr grabberLink = robot->IsGrabbing(body);
    assert(grabberLink);
    BOOST_FOREACH(const KinBody::LinkPtr& link, body->GetLinks()) {
      if (link->GetGeometries().size() > 0) {
        links.push_back(link);
        linkInds.push_back(grabberLink->GetIndex());
      }
    }
  }

}


void getAffectedLinks2(RobotBasePtr robot, const vector<int>& dofInds, vector<KinBody::LinkPtr>& links,
                       vector<int>& linkInds) {
  const vector<KinBody::LinkPtr>& robotLinks = robot->GetLinks();
  links.clear();
  linkInds.clear();
  BOOST_FOREACH(const KinBody::LinkPtr& link, robotLinks) {
    if (link->GetGeometries().size()>0 && doesAffect(robot, dofInds, link->GetIndex())) {
      links.push_back(link);
      linkInds.push_back(link->GetIndex());
    }
  }

  vector<KinBodyPtr> grabbed;
  robot->GetGrabbed(grabbed);
  BOOST_FOREACH(const KinBodyPtr& body, grabbed) {
    KinBody::LinkPtr grabberLink = robot->IsGrabbing(body);
    assert(grabberLink);
    if (doesAffect(robot, dofInds, grabberLink->GetIndex())) {
      BOOST_FOREACH(const KinBody::LinkPtr& link, body->GetLinks()) {
        if (link->GetGeometries().size()>0) {
          links.push_back(link);
          linkInds.push_back(grabberLink->GetIndex());
        }
      }
    }
  }

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

vector<KinBody::LinkPtr> fullBodyCollisionLinks(RaveRobotObject* rro) {
  std::set<KinBody::LinkPtr> linkSet;
  for (int i=0; i < rro->numCreatedManips(); ++i) {
    KinBody::LinkPtr eeLink = rro->getManipByIndex(i)->manip->GetEndEffector();
    vector<KinBody::LinkPtr> chain;
    rro->robot->GetChain(0,eeLink->GetIndex(), chain);
    BOOST_FOREACH(KinBody::LinkPtr link, chain) {
      if (link->GetGeometries().size() > 0) {
        linkSet.insert(link);
      }
    }
  }
  vector<KinBody::LinkPtr> out;
  BOOST_FOREACH(KinBody::LinkPtr link, linkSet) {
    out.push_back(link);
  }
  return out;
}


BulletRaveSyncherPtr fullBodySyncher(RaveRobotObject* rro) {
  RobotBasePtr robot = rro->robot;

  vector<KinBody::LinkPtr> links = fullBodyCollisionLinks(rro);

  stringstream ss;

  vector<btRigidBody*> bodies;
  BOOST_FOREACH(KinBody::LinkPtr link, links) {
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



