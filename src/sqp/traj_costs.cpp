#include "traj_costs.h"
#include "simulation/util.h"
#include <boost/foreach.hpp>
#include "utils_sqp.h"
#include "utils/config.h"
#include "utils/logging.h"
#include "utils/clock.h"
#include "simulation/config_bullet.h"
using namespace std;
using namespace OpenRAVE;
using namespace util;
using namespace Eigen;

void getArmKinInfo(const RobotBasePtr& robot, const RobotBase::ManipulatorPtr manip, std::vector<KinBody::LinkPtr>& armLinks, std::vector<KinBody::JointPtr>& armJoints, std::vector<int>& chainDepthOfBodies) {
  int rootLinkInd               = robot->GetLink("torso_lift_link")->GetIndex();
  BOOST_FOREACH(int ind, manip->GetArmIndices()) armJoints.push_back(robot->GetJointFromDOFIndex(ind));
  KinBody::JointPtr& firstJoint = armJoints[0];

  BOOST_FOREACH(KinBody::LinkPtr link, robot->GetLinks()) {
    vector<KinBody::JointPtr> jointChain;
    robot->GetChain(rootLinkInd, link->GetIndex(), jointChain);
    if (link->GetGeometries().size() && count(jointChain.begin(), jointChain.end(), firstJoint)) armLinks.push_back(link);
  }

  int nLinks                    = armLinks.size();

  chainDepthOfBodies            = vector<int>(nLinks,0);
  for (int iLink                = 0; iLink < armLinks.size(); ++iLink) {
    int linkInd                 = armLinks[iLink]->GetIndex();
    vector<KinBody::JointPtr> jointChain;
    robot->GetChain(rootLinkInd, armLinks[iLink]->GetIndex(), jointChain);
    BOOST_FOREACH(KinBody::JointPtr& joint0, armJoints) {
      BOOST_FOREACH(KinBody::JointPtr& joint1, jointChain) {
        if (joint0 == joint1) chainDepthOfBodies[iLink]++;
      }
    }
  }
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

void calcMultiBodyJointSpaceCollisionInfo(const std::vector<btRigidBody*>& bodies, btCollisionWorld* world, const RobotBasePtr& robot, const vector<KinBody::JointPtr>& jointsInChain, const vector<int>& chainDepthOfBodies, std::vector<Eigen::VectorXd>& collJacs, std::vector<
    double>& collDists) {
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
  BOOST_FOREACH(KinBody::JointPtr joint, m_joints)
armInds  .push_back(joint->GetDOFIndex());
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

typedef std::pair<std::vector<Eigen::VectorXd>, std::vector<double> > TimestepCollisionInfo;
typedef std::vector<TimestepCollisionInfo> TrajCollisionInfo;

TrajCollisionInfo ArmCCE::collectCollisionInfo(const Eigen::MatrixXd& traj) {
  int nSteps = traj.rows();

  vector<int> armInds;
  BOOST_FOREACH(KinBody::JointPtr joint, m_joints) armInds.push_back(joint->GetDOFIndex());
  m_robot->SetActiveDOFs(armInds);
  vector<double> curVals;
  m_robot->GetActiveDOFValues(curVals);

  vector<TimestepCollisionInfo> trajCollisionInfo(nSteps);
  for (int iStep=0; iStep < nSteps; ++iStep) {
    m_robot->SetActiveDOFValues(toDoubleVec(traj.row(iStep)));
    m_syncher.updateBullet();
    calcMultiBodyJointSpaceCollisionInfo(m_bodies, m_world, m_robot, m_joints, m_chainDepthOfBodies, trajCollisionInfo[iStep].first, trajCollisionInfo[iStep].second);
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


void countCollisions(const TrajCollisionInfo& trajCollInfo, double safeDistMinusPadding, int& nNear, int& nUnsafe, int& nColl) {
  nNear=0;
  nUnsafe=0;
  nColl=0;
  for (int iStep = 0; iStep < trajCollInfo.size(); ++iStep) {
    const vector<double>& dists = trajCollInfo[iStep].second;
    for (int iColl=0; iColl < trajCollInfo[iStep].first.size(); ++iColl) {
      if (dists[iColl] < -BulletConfig::linkPadding) ++nColl;
      else if (dists[iColl] < safeDistMinusPadding) ++nUnsafe;
      else if (dists[iColl] < 0) ++nNear;
    }
  }
}
