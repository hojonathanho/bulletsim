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
#include "kinematics_utils.h"
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

TrajCartCollInfo collectTrajCollisions(const Eigen::MatrixXd& traj, RobotBasePtr robot, BulletRaveSyncher& brs, btCollisionWorld* world, const std::vector<int>& dofInds, bool useAffine) {
  ScopedRobotSave srs(robot);
  TrajCartCollInfo out(traj.rows());
  if (useAffine) {
    robot->SetActiveDOFs(dofInds, DOF_X | DOF_Y | DOF_RotationAxis, OpenRAVE::RaveVector<double>(0,0,1));
  }
  else {
    robot->SetActiveDOFs(dofInds);
  }
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
        btVector3 point = ((collision.m_obj0 == body) ? collision.m_world0 : collision.m_world1)/METERS;
        btVector3 normal = (collision.m_obj0 == body) ? collision.m_normal : -collision.m_normal;
        double dist = collision.m_distance/METERS+BulletConfig::linkPadding;
        if (point.getZ() > .05*METERS) out[iStep].push_back(LinkCollision(dist, linkInds[iBody], point, normal));
      }
    }
    LOG_DEBUG_FMT("%i collisions at time %i", out[iStep].size(), iStep);
  }
  return out;
}


TrajCartCollInfo continuousTrajCollisions(const Eigen::MatrixXd& traj,
    OpenRAVE::RobotBasePtr robot, BulletRaveSyncher& brs, btCollisionWorld* world,
    const std::vector<int>& dofInds, float dSafeCont) {
  ScopedRobotSave srs(robot);
  vector<int> linkInds;
  BOOST_FOREACH(KinBody::LinkPtr link, brs.m_links) linkInds.push_back(link->GetIndex());


  robot->SetActiveDOFs(dofInds);
  robot->SetActiveDOFValues(toDoubleVec(traj.row(0)));

  vector<btTransform> oldTransforms;
  BOOST_FOREACH(OpenRAVE::KinBody::LinkPtr link, brs.m_links) oldTransforms.push_back(toBtTransform(link->GetTransform(), METERS));

  TrajCartCollInfo out(traj.rows()-1);


  for (int iStep=1; iStep < traj.rows(); ++iStep) {
    robot->SetActiveDOFValues(toDoubleVec(traj.row(iStep)));
    vector<btTransform> newTransforms;
    BOOST_FOREACH(OpenRAVE::KinBody::LinkPtr link, brs.m_links) newTransforms.push_back(toBtTransform(link->GetTransform(),METERS));
    for (int iBody=0; iBody < brs.m_bodies.size(); ++iBody) {
      btConvexShape* cShape = dynamic_cast<btConvexShape*>(brs.m_bodies[iBody]->getCollisionShape());
      if (cShape == NULL) printf("bad link: %s\n", brs.m_links[iBody]->GetName().c_str());
      assert(cShape != NULL);
      btCollisionWorld::ClosestConvexResultCallback ccc(btVector3(NAN, NAN, NAN), btVector3(NAN, NAN, NAN));
      // XXX in general there is some arbitrary child shape transform
      // but for our pr2 model loaded by openravesupport.cpp, there's only one child shape, and transform is the idenetiy
      world->convexSweepTest(cShape, oldTransforms[iBody], newTransforms[iBody], ccc, (BulletConfig::linkPadding - dSafeCont)*METERS);
      if (ccc.hasHit() && ccc.m_hitPointWorld.getZ() > .05*METERS) {
        out[iStep-1].push_back(LinkCollision(.05, linkInds[iBody], ccc.m_hitPointWorld, ccc.m_hitNormalWorld));
        out[iStep-1][0].frac = ccc.m_closestHitFraction;
      }
    }

    oldTransforms = newTransforms;
  }
  return out;
}

JointCollInfo cartToJointCollInfo(const CartCollInfo& in, const Eigen::VectorXd& dofVals, RobotBasePtr robot,
    const std::vector<int>& dofInds, bool useAffine) {

  ScopedRobotSave srs(robot);

  if (useAffine) {
    robot->SetActiveDOFs(dofInds, DOF_X | DOF_Y | DOF_RotationAxis, OpenRAVE::RaveVector<double>(0,0,1));
  }
  else {
    robot->SetActiveDOFs(dofInds);
  }
  robot->SetActiveDOFValues(toDoubleVec(dofVals));

  JointCollInfo out;
  out.dists.resize(in.size());
  out.jacs.resize(in.size());

  int nJoints = dofInds.size() + useAffine*3;

  for (int iColl = 0; iColl < in.size(); ++iColl) {
    const LinkCollision& lc = in[iColl];
    out.dists[iColl] = lc.dist;

    std::vector<double> jacvec(3*nJoints);
    robot->CalculateActiveJacobian(lc.linkInd, toRaveVector(lc.point), jacvec);
    out.jacs[iColl] = - toVector3d(lc.normal).transpose() * Eigen::Map<MatrixXd>(jacvec.data(), 3, nJoints);
    if (useAffine) out.jacs[iColl](nJoints - 1) *= -1;

  }
  return out;
}



TrajJointCollInfo trajCartToJointCollInfo(const TrajCartCollInfo& in, const Eigen::MatrixXd& traj, RobotBasePtr robot,
    const std::vector<int>& dofInds, bool useAffine) {
  TrajJointCollInfo out(in.size());
  for (int iStep=0; iStep < in.size(); ++iStep) {
    out[iStep] = cartToJointCollInfo(in[iStep], traj.row(iStep), robot, dofInds, useAffine);
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
    dists[iColl] = (collision.m_distance + BulletConfig::linkPadding) / METERS;
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




void countCollisions(const TrajJointCollInfo& trajCollInfo, double safeDist, int& nNear, int& nUnsafe, int& nColl) {
  nNear=0;
  nUnsafe=0;
  nColl=0;
  for (int iStep = 0; iStep < trajCollInfo.size(); ++iStep) {
    const vector<double>& dists = trajCollInfo[iStep].dists;
    for (int iColl=0; iColl < trajCollInfo[iStep].jacs.size(); ++iColl) {
      if (dists[iColl] < 0) ++nColl;
      else if (dists[iColl] < safeDist) ++nUnsafe;
      else ++nNear;
    }
  }
}
