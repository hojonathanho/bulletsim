#include "traj_costs.h"
#include "simulation/util.h"
#include <boost/foreach.hpp>
#include "utils_sqp.h"
#include "utils/config.h"
#include "utils/logging.h"
#include "utils/clock.h"
#include "simulation/config_bullet.h"
#include "config_sqp.h"
#include "simulation/openravesupport.h"
using namespace std;
using namespace OpenRAVE;
using namespace util;
using namespace Eigen;

void getArmKinInfo(const RobotBasePtr& robot, const RobotBase::ManipulatorPtr manip, std::vector<KinBody::LinkPtr>& armLinks, 
    std::vector<KinBody::JointPtr>& armJoints, std::vector<int>& chainDepthOfBodies) {
  int rootLinkInd = robot->GetLink("torso_lift_link")->GetIndex();
  BOOST_FOREACH(int ind, manip->GetArmIndices()) armJoints.push_back(robot->GetJointFromDOFIndex(ind));
  KinBody::JointPtr& firstJoint = armJoints[0];

  BOOST_FOREACH(KinBody::LinkPtr link, robot->GetLinks()) {
    vector<KinBody::JointPtr> jointChain;
    robot->GetChain(rootLinkInd, link->GetIndex(), jointChain);
    if (link->GetGeometries().size() && count(jointChain.begin(), jointChain.end(), firstJoint)) armLinks.push_back(link);
  }

  int nLinks = armLinks.size();

  chainDepthOfBodies = vector<int>(nLinks,0);
  for (int iLink = 0; iLink < armLinks.size(); ++iLink) {
    vector<KinBody::JointPtr> jointChain;
    robot->GetChain(rootLinkInd, armLinks[iLink]->GetIndex(), jointChain);
    BOOST_FOREACH(KinBody::JointPtr& joint0, armJoints) {
      BOOST_FOREACH(KinBody::JointPtr& joint1, jointChain) {
        if (joint0 == joint1) chainDepthOfBodies[iLink]++;
      }
    }
  }
}

std::vector<KinBody::JointPtr> getArmJoints(OpenRAVE::RobotBase::ManipulatorPtr manip) {
  std::vector<KinBody::JointPtr> armJoints;
  BOOST_FOREACH(int ind, manip->GetArmIndices()) armJoints.push_back(manip->GetRobot()->GetJointFromDOFIndex(ind));
  return armJoints;
}

std::vector<KinBody::LinkPtr> getArmLinks(OpenRAVE::RobotBase::ManipulatorPtr manip) {
  RobotBasePtr robot = manip->GetRobot();
  int rootLinkInd = robot->GetLink("torso_lift_link")->GetIndex();
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




#if 0
MatrixXd discreteDeriv(MatrixXd& in) {
  MatrixXd out(in.rows(), in.cols());
  out.row(0) = in.row(1) - in.row(0);
  int nRow = in.rows();
  out.row(nRow-1) = in.row(nRow-1) - in.row(nRow-2);
  out.block(1,nRow-2) = in.block(2, nRow-1) - in.block(0,nRow-3);
  return out;
}

MatrixXd discreteSecondDeriv(MatrixXd& in) {
  return discreteDeriv(discreteDeriv(in));
}
#endif

TrajCartCollInfo collectTrajCollisions(const Eigen::MatrixXd& traj, RobotBasePtr robot, BulletRaveSyncher& brs, btCollisionWorld* world, const std::vector<int>& dofInds) {
  ScopedRobotSave srs(robot);
  TrajCartCollInfo out(traj.rows());
  robot->SetActiveDOFs(dofInds);
  vector<int> linkInds;
  BOOST_FOREACH(KinBody::LinkPtr link, brs.m_links) linkInds.push_back(link->GetIndex());
  for (int iStep=0; iStep<traj.rows(); ++iStep) {
    robot->SetActiveDOFValues(toDoubleVec(traj.row(iStep)));
    brs.updateBullet();
    for (int iBody=0; iBody < brs.m_bodies.size(); ++iBody) {
      CollisionCollector collisionCollector;
      btRigidBody* body = brs.m_bodies[iBody];
      world->contactTest(body, collisionCollector);
      int nColl = collisionCollector.m_collisions.size();
      for (int iColl = 0; iColl < nColl; ++iColl) {
        Collision& collision = collisionCollector.m_collisions[iColl];
        btVector3 point = (collision.m_obj0 == body) ? collision.m_world0 : collision.m_world1;
        btVector3 normal = (collision.m_obj0 == body) ? collision.m_normal : -collision.m_normal;
        double dist = collision.m_distance;
        out[iStep].push_back(LinkCollision(dist, linkInds[iBody], point, normal));
      }
    }
  }
  return out;
}

//inline Eigen::Vector3d toVector3d(const btVector3& in) {return Eigen::Vector3d(in.x(), in.y(), in.z());}

JointCollInfo cartToJointCollInfo(const CartCollInfo& in, const Eigen::VectorXd& dofVals, RobotBasePtr robot,
    const std::vector<int>& dofInds) {

  ScopedRobotSave srs(robot);

  robot->SetActiveDOFs(dofInds);
  robot->SetActiveDOFValues(toDoubleVec(dofVals));
  vector<KinBody::JointPtr> joints;
  BOOST_FOREACH(int dofInd, dofInds) joints.push_back(robot->GetJointFromDOFIndex(dofInd));

  JointCollInfo out;
  out.dists.resize(in.size());
  out.jacs.resize(in.size());

  for (int iColl = 0; iColl < in.size(); ++iColl) {
    const LinkCollision& lc = in[iColl];
//    out.jacs[iColl] = VectorXd::Zero(joints.size());
    out.dists[iColl] = lc.dist;
#if 0
    for (int iJoint = 0; iJoint < chainDepthOfBodies[lc.linkInd]; ++iJoint) {
      const KinBody::JointPtr& joint = joints[iJoint];
      out.jacs[iColl](iJoint) += (lc.point - toBtVector(joint->GetAnchor())) .cross(toBtVector(joint->GetAxis())) .dot(lc.normal);
      out.dists[iColl] = lc.dist;
    }
#endif

#if 0
    for (int iJoint = 0; iJoint < joints.size(); ++iJoint) {
      if (robot->DoesAffect(dofInds[iJoint], lc.linkInd)) {
        const KinBody::JointPtr& joint = joints[iJoint];
        out.jacs[iColl](iJoint) = (lc.point - toBtVector(joint->GetAnchor())) .cross(toBtVector(joint->GetAxis())) .dot(lc.normal);
      }
    }
#endif

#if 1
    int nJoints = joints.size();
    std::vector<double> jacvec(3*nJoints);
    robot->CalculateActiveJacobian(lc.linkInd, toRaveVector(lc.point), jacvec);
    out.jacs[iColl] = - toVector3d(lc.normal).transpose() * Eigen::Map<MatrixXd>(jacvec.data(), 3, nJoints);

//    cout << grad.transpose() << endl;
//    cout << out.jacs[iColl].transpose() << endl;
//    cout << (grad - out.jacs[iColl]).norm() << endl;
//    assert ((grad - out.jacs[iColl]).norm() < 1e-6);
#endif

  }
  return out;
}

TrajJointCollInfo trajCartToJointCollInfo(const TrajCartCollInfo& in, const Eigen::MatrixXd& traj, RobotBasePtr robot,
    const std::vector<int>& dofInds) {
  TrajJointCollInfo out(in.size());
  for (int iStep=0; iStep < in.size(); ++iStep) {
    out[iStep] = cartToJointCollInfo(in[iStep], traj.row(iStep), robot, dofInds);
  }
  return out;
}



void calcCollisionInfo(btRigidBody* body, btCollisionWorld* world, std::vector<btVector3>& points, std::vector<btVector3>& normals, std::vector<double>& dists) {
  CollisionCollector collisionCollector;
  world->contactTest(body, collisionCollector);
  int nColl = collisionCollector.m_collisions.size();
  points.resize(nColl);
  normals.resize(nColl);
  dists.resize(nColl);
  for (int iColl = 0; iColl < nColl; ++iColl) {
    Collision& collision = collisionCollector.m_collisions[iColl];
    points[iColl] = (collision.m_obj0 == body) ? collision.m_world0 : collision.m_world1;
    normals[iColl] = (collision.m_obj0 == body) ? collision.m_normal : -collision.m_normal;
    dists[iColl] = collision.m_distance;
  }
}

float calcCollisionCost(btRigidBody* body, btCollisionWorld* world) {
  std::vector<btVector3> points, normals;
  std::vector<double> depths;
  calcCollisionInfo(body, world, points, normals, depths);
  float cost = 0;
  for (int i = 0; i < depths.size(); ++i) {
    cost += depths[i] * depths[i];
  }
  return cost;
}

void calcCollisionCostAndJointGrad(const std::vector<btRigidBody*>& bodies, btCollisionWorld* world, const RobotBasePtr& robot, const vector<KinBody::JointPtr>& jointsInChain, const vector<int>& chainDepthOfBodies, double& cost, Eigen::VectorXd& grad, int& nColl) {
  int nJoints = jointsInChain.size();

  cost = 0;
  grad = Eigen::VectorXd::Zero(nJoints);
  nColl = 0;

  for (int iBody = 0; iBody < bodies.size(); ++iBody) {
    std::vector<double> depths;
    std::vector<btVector3> normals, points;
    calcCollisionInfo(bodies[iBody], world, points, normals, depths);
    nColl += depths.size();
    for (int iColl = 0; iColl < depths.size(); ++iColl) {
      for (int iJoint = 0; iJoint < chainDepthOfBodies[iBody]; ++iJoint) {
        const KinBody::JointPtr& joint = jointsInChain[iJoint];
        grad(iJoint) += (points[iColl] - toBtVector(joint->GetAnchor())) .cross(toBtVector(jointsInChain[iJoint]->GetAxis())) .dot(normals[iColl]);
        // note: there's also an openrave function in KinBody to calculate jacobian that is more general
        cost += fabs(depths[iColl]);
      }
    }
  }
}

void calcMultiBodyJointSpaceCollisionInfo(const std::vector<btRigidBody*>& bodies, btCollisionWorld* world, 
  const RobotBasePtr& robot, const vector<KinBody::JointPtr>& jointsInChain, 
  const vector<int>& chainDepthOfBodies, std::vector<Eigen::VectorXd>& collJacs, std::vector<double>& collDists) {
  int nJoints = jointsInChain.size();

  for (int iBody = 0; iBody < bodies.size(); ++iBody) {
    std::vector<double> dists;
    std::vector<btVector3> normals, points;
    calcCollisionInfo(bodies[iBody], world, points, normals, dists);
    for (int iColl = 0; iColl < dists.size(); ++iColl) {
      VectorXd collJac = VectorXd::Zero(nJoints);
      for (int iJoint = 0; iJoint < chainDepthOfBodies[iBody]; ++iJoint) {
        const KinBody::JointPtr& joint = jointsInChain[iJoint];
        collJac(iJoint) = (points[iColl] - toBtVector(joint->GetAnchor())) .cross(toBtVector(jointsInChain[iJoint]->GetAxis())) .dot(normals[iColl]);
        // note: there's also an openrave function in KinBody to calculate jacobian that is more general
      }
      collJacs.push_back(collJac);
      collDists.push_back(dists[iColl]);
    }
    if (SQPConfig::topCollOnly && points.size() > 0) continue;
  }
}

#if 0
void calcMultiBodyJointSpaceCollisionInfo(const std::vector<btRigidBody*>& bodies, btCollisionWorld* world, 
  const RobotBasePtr& robot, const vector<KinBody::JointPtr>& jointsInChain, 
  std::vector<Eigen::VectorXd>& collJacs, std::vector<double>& collDists) {
  int nJoints = jointsInChain.size();

  for (int iBody = 0; iBody < bodies.size(); ++iBody) {
    std::vector<double> dists;
    std::vector<btVector3> normals, points;
    calcCollisionInfo(bodies[iBody], world, points, normals, dists);
    for (int iColl = 0; iColl < dists.size(); ++iColl) {
      VectorXd collJac(nJoints);
      for (int iJoint = 0; iJoint < chainDepthOfBodies[iBody]; ++iJoint) {
        const KinBody::JointPtr& joint = jointsInChain[iJoint];
        collJac(iJoint) = (points[iColl] - toBtVector(joint->GetAnchor())) .cross(toBtVector(jointsInChain[iJoint]->GetAxis())) .dot(normals[iColl]);
        // note: there's also an openrave function in KinBody to calculate jacobian that is more general
      }
      collJacs.push_back(collJac);
      collDists.push_back(dists[iColl]);
    }
  }
}

TrajJointCollInfo GenericCCE::collectCollisionInfo(const Eigen::MatrixXd& traj) {
  ScopedRobotSave srs(m_robot);
  TrajJointCollInfo trajCI(traj.rows());
  for (int iStep=0; iStep<traj.rows(); ++iStep) {
    TimeStepCollisionInfo& timeCI = trajCI[iStep];
    robot->setDOFValues(traj.row(iStep));
    
  }
}
#endif


//std::vector< std::vector<Collision> >
//TrajCollInfo getTrajCollisions(){}

float ArmCCE::calcCost(const Eigen::MatrixXd& traj) {
  float val = 0;
  int nSteps = traj.rows();

  vector<int> armInds;
  BOOST_FOREACH(KinBody::JointPtr joint, m_joints) armInds.push_back(joint->GetDOFIndex());
  m_robot->SetActiveDOFs(armInds);
  vector<double> curVals;
  m_robot->GetActiveDOFValues(curVals);

  for (int iStep=0; iStep < nSteps; ++iStep) {
    m_robot->SetActiveDOFValues(toDoubleVec(traj.row(iStep)));
    m_syncher.updateBullet();
    for (int iBody=0; iBody < m_bodies.size(); ++iBody) {
      val += calcCollisionCost(m_bodies[iBody], m_world);
    }
  }

  m_robot->SetActiveDOFValues(curVals);
  m_syncher.updateBullet();
  return val;
}

void ArmCCE::calcCostAndGrad(const Eigen::MatrixXd& traj, double& val, Eigen::MatrixXd& grad) {
  int nSteps = traj.rows();

  vector<int> armInds;
  BOOST_FOREACH(KinBody::JointPtr joint, m_joints) armInds.push_back(joint->GetDOFIndex());
  m_robot->SetActiveDOFs(armInds);
  vector<double> curVals;
  m_robot->GetActiveDOFValues(curVals);

  grad = MatrixXd::Zero(nSteps, m_nJoints);
  int totalNumColl=0;

  for (int iStep=0; iStep < nSteps; ++iStep) {
    m_robot->SetActiveDOFValues(toDoubleVec(traj.row(iStep)));
    m_syncher.updateBullet();
    VectorXd gradHere;
    double costHere;
    int numCollHere;
    calcCollisionCostAndJointGrad(m_bodies, m_world, m_robot, m_joints, m_chainDepthOfBodies, costHere, gradHere, numCollHere);
    grad.row(iStep) += gradHere;
    val += costHere;
    totalNumColl += numCollHere;
  }

  m_robot->SetActiveDOFValues(curVals);
  m_syncher.updateBullet();

  LOG_INFO_FMT("total number of collisions: %i", totalNumColl);
}

TrajJointCollInfo ArmCCE::collectCollisionInfo(const Eigen::MatrixXd& traj) {
  int nSteps = traj.rows();

  vector<int> armInds;
  BOOST_FOREACH(KinBody::JointPtr joint, m_joints) armInds.push_back(joint->GetDOFIndex());
  m_robot->SetActiveDOFs(armInds);
  vector<double> curVals;
  m_robot->GetActiveDOFValues(curVals);

  vector<JointCollInfo> trajCollisionInfo(nSteps);
  for (int iStep=0; iStep < nSteps; ++iStep) {
    m_robot->SetActiveDOFValues(toDoubleVec(traj.row(iStep)));
    m_syncher.updateBullet();
    calcMultiBodyJointSpaceCollisionInfo(m_bodies, m_world, m_robot, m_joints, m_chainDepthOfBodies, trajCollisionInfo[iStep].jacs, trajCollisionInfo[iStep].dists);
  }

  m_robot->SetActiveDOFValues(curVals);
  m_syncher.updateBullet();

  return trajCollisionInfo;
}

void BulletRaveSyncher::updateBullet() {
  for (int iBody = 0; iBody < m_bodies.size(); ++iBody) {
    m_bodies[iBody]->setCenterOfMassTransform(util::toBtTransform(m_links[iBody]->GetTransform(), GeneralConfig::scale));
  }
}


void countCollisions(const TrajJointCollInfo& trajCollInfo, double safeDistMinusPadding, int& nNear, int& nUnsafe, int& nColl) {
  nNear=0;
  nUnsafe=0;
  nColl=0;
  double eps = 1e-6;
  for (int iStep = 0; iStep < trajCollInfo.size(); ++iStep) {
    const vector<double>& dists = trajCollInfo[iStep].dists;
    for (int iColl=0; iColl < trajCollInfo[iStep].jacs.size(); ++iColl) {
      if (dists[iColl] < -BulletConfig::linkPadding) ++nColl;
      else if (dists[iColl] < safeDistMinusPadding-eps) ++nUnsafe;
      else if (dists[iColl] < -eps) ++nNear;
    }
  }
}
