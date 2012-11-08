#include "collisions.h"
#include "simulation/util.h"
#include <boost/foreach.hpp>
#include "utils_sqp.h"
#include "utils/config.h"
#include "utils/logging.h"
#include "simulation/config_bullet.h"
#include "config_sqp.h"
#include "simulation/bullet_io.h"
#include "simulation/openravesupport.h"
#include "kinematics_utils.h"
using namespace std;
using namespace OpenRAVE;
using namespace util;

using namespace Eigen;

#define USE_CONTACTTEST

struct CollisionCollector : public btCollisionWorld::ContactResultCallback {
  std::vector<Collision> m_collisions;
  btScalar addSingleResult(btManifoldPoint& pt, const btCollisionObject *colObj0, int, int,
                           const btCollisionObject *colObj1, int, int) {
    m_collisions.push_back(Collision(colObj0, colObj1, pt.m_positionWorldOnA, pt.m_positionWorldOnB,
                                     pt.m_normalWorldOnB, pt.m_distance1));
    return 0;
  }
};

#ifndef USE_CONTACTTEST

TrajCartCollInfo collectTrajCollisions(const Eigen::MatrixXd& traj, RaveRobotObject* rro,
                                       const std::vector<int>& dofInds, bool useAffine) {
  // BEGIN COPIED                                       
  RobotBasePtr& robot = rro->robot;
  ScopedRobotSave srs(robot);
  btCollisionWorld* world = rro->getEnvironment()->bullet->dynamicsWorld;

  vector<KinBody::LinkPtr> links;
  vector<int> linkInds;
  vector<btRigidBody*> bodies;
  if (useAffine) getAffectedLinks2(rro, links, linkInds);
  else getAffectedLinks2(rro->robot, dofInds, links, linkInds);
  BOOST_FOREACH(KinBody::LinkPtr& link, links) {
    bodies.push_back(rro->rave->rave2bulletsim[link->GetParent()]->associatedObj(link)->rigidBody.get());
  }
  BulletRaveSyncher brs(links, bodies);
  TrajCartCollInfo out(traj.rows());

  if (useAffine) {
    robot->SetActiveDOFs(dofInds, DOF_X | DOF_Y | DOF_RotationAxis, OpenRAVE::RaveVector<double>(0, 0, 1));
  } else {
    robot->SetActiveDOFs(dofInds);
  }
  // END COPIED

  typedef map<btRigidBody*,int> RB2LinkInd;
  RB2LinkInd rb2linkInd;
  typedef pair<btRigidBody*, btRigidBody*> RBPair;
  set< RBPair > ignore;
  for (int i=0; i < bodies.size(); ++i) {
    rb2linkInd[bodies[i]] = linkInds[i];
  }

  ///// actually run through the trajectories and find collisions /////////////
  for (int iStep = 0; iStep < traj.rows(); ++iStep) {
    robot->SetActiveDOFValues(toDoubleVec(traj.row(iStep)));
    brs.updateBullet();
    
    world->performDiscreteCollisionDetection();
    btDispatcher* dispatcher = world->getDispatcher();
    int numManifolds = dispatcher->getNumManifolds();
    for (int i = 0; i < numManifolds; ++i) {
      btPersistentManifold* contactManifold = dispatcher->getManifoldByIndexInternal(i);
      int numContacts = contactManifold->getNumContacts();
      btRigidBody* objA = static_cast<btRigidBody*> (contactManifold->getBody0());
      btRigidBody* objB = static_cast<btRigidBody*> (contactManifold->getBody1());

//      if (iStep == 0) {
//        ignore.insert(RBPair(objA, objB));
//        continue;
//      }
//      if (ignore.find(RBPair(objA, objB)) != ignore.end()) continue;
      

      RB2LinkInd::iterator itA = rb2linkInd.find(objA);
      if (itA != rb2linkInd.end()) {
        int linkInd = itA->second;
        for (int j = 0; j < numContacts; ++j) {
          btManifoldPoint& pt = contactManifold->getContactPoint(j);
          btVector3 point = pt.m_positionWorldOnA / METERS;
          btVector3 normal = pt.m_normalWorldOnB;
          double dist = pt.m_distance1 / METERS + SQPConfig::padMult * BulletConfig::linkPadding;
          if (point.getZ() > .1) {
            out[iStep].push_back(LinkCollision(dist, linkInd, point, normal,1./numContacts));
          }
        }
      }
      RB2LinkInd::iterator itB = rb2linkInd.find(objB);
      if (itB != rb2linkInd.end()) {
        int linkInd = itB->second;
        for (int j = 0; j < numContacts; ++j) {
          btManifoldPoint& pt = contactManifold->getContactPoint(j);
          btVector3 point = pt.m_positionWorldOnB / METERS;
          btVector3 normal = -pt.m_normalWorldOnB;
          double dist = pt.m_distance1 / METERS + SQPConfig::padMult * BulletConfig::linkPadding;
          if (point.getZ() > .1) {
            out[iStep].push_back(LinkCollision(dist, linkInd, point, normal, 1. / numContacts));
          }
        }
      }
    }
  }
  return out;
}

#else

TrajCartCollInfo collectTrajCollisions(const Eigen::MatrixXd& traj, RaveRobotObject* rro,
                                       const std::vector<int>& dofInds, bool useAffine) {
  RobotBasePtr& robot = rro->robot;
  ScopedRobotSave srs(robot);
  btCollisionWorld* world = rro->getEnvironment()->bullet->dynamicsWorld;

  vector<KinBody::LinkPtr> links;
  vector<int> linkInds;
  vector<btRigidBody*> bodies;
  if (useAffine) getAffectedLinks2(rro, links, linkInds);
  else getAffectedLinks2(rro->robot, dofInds, links, linkInds);
  BOOST_FOREACH(KinBody::LinkPtr& link, links) {
    bodies.push_back(rro->rave->rave2bulletsim[link->GetParent()]->associatedObj(link)->rigidBody.get());
  }
  BulletRaveSyncher brs(links, bodies);
  TrajCartCollInfo out(traj.rows());

  if (useAffine) {
    robot->SetActiveDOFs(dofInds, DOF_X | DOF_Y | DOF_RotationAxis, OpenRAVE::RaveVector<double>(0, 0, 1));
  } else {
    robot->SetActiveDOFs(dofInds);
  }

  ///// actually run through the trajectories and find collisions /////////////
  for (int iStep = 0; iStep < traj.rows(); ++iStep) {
    robot->SetActiveDOFValues(toDoubleVec(traj.row(iStep)));
    brs.updateBullet();
    for (int iBody = 0; iBody < brs.m_bodies.size(); ++iBody) {
      CollisionCollector collisionCollector;
      btRigidBody* body = brs.m_bodies[iBody];
      world->contactTest(body, collisionCollector);
      int nColl = collisionCollector.m_collisions.size();
      for (int iColl = 0; iColl < nColl; ++iColl) {
        Collision& collision = collisionCollector.m_collisions[iColl];
        btVector3 point = ((collision.m_obj0 == body) ? collision.m_world0 : collision.m_world1) / METERS;
        btVector3 normal = (collision.m_obj0 == body) ? collision.m_normal : -collision.m_normal;
        double dist = collision.m_distance / METERS + SQPConfig::padMult * BulletConfig::linkPadding;
        if (point.getZ() > .1) {
          out[iStep].push_back(LinkCollision(dist, linkInds[iBody], point, normal,1./nColl));
        }
      }
    }
  }
  return out;
}
#endif


btConvexShape* getConvexCollisionShape(btCollisionShape* shape) {
  if (shape->getShapeType() == COMPOUND_SHAPE_PROXYTYPE) {
    btCompoundShape* compound = static_cast<btCompoundShape*> (shape);
    assert(compound->getNumChildShapes()==1);
    btConvexShape* out = static_cast<btConvexShape*> (compound->getChildShape(0));
    assert(out != NULL);
    return out;
  } else {
    btConvexShape* out = static_cast<btConvexShape*> (shape);
    assert(out != NULL);
    return out;
  }
}

TrajCartCollInfo continuousTrajCollisions(const Eigen::MatrixXd& traj, RaveRobotObject* rro,
                  const std::vector<int>& dofInds, bool useAffine, float dSafeCont) {

  // COPIED VERBATIM FROM collectTrajCollisions
  RobotBasePtr& robot = rro->robot;
  ScopedRobotSave srs(robot);
  btCollisionWorld* world = rro->getEnvironment()->bullet->dynamicsWorld;

  vector<KinBody::LinkPtr> links;
  vector<int> linkInds;
  vector<btRigidBody*> bodies;
  if (useAffine) getAffectedLinks2(rro, links, linkInds);
  else getAffectedLinks2(rro->robot, dofInds, links, linkInds);
  BOOST_FOREACH(KinBody::LinkPtr& link, links) {
    bodies.push_back(rro->rave->rave2bulletsim[link->GetParent()]->associatedObj(link)->rigidBody.get());
  }
  BulletRaveSyncher brs(links, bodies);
  TrajCartCollInfo out(traj.rows()-1); // except this line!!

  if (useAffine) {
    robot->SetActiveDOFs(dofInds, DOF_X | DOF_Y | DOF_RotationAxis, OpenRAVE::RaveVector<double>(0, 0, 1));
  } else {
    robot->SetActiveDOFs(dofInds);
  }
  //////////////////////////////////////////////

  robot->SetActiveDOFValues(toDoubleVec(traj.row(0)));

  vector<btTransform> oldTransforms;
  BOOST_FOREACH(OpenRAVE::KinBody::LinkPtr link, brs.m_links) {
    oldTransforms.push_back(toBtTransform(link->GetTransform(), METERS));
  }

  for (int iStep = 1; iStep < traj.rows(); ++iStep) {
    robot->SetActiveDOFValues(toDoubleVec(traj.row(iStep)));
    vector<btTransform> newTransforms;
    BOOST_FOREACH(OpenRAVE::KinBody::LinkPtr link, brs.m_links) {
      newTransforms.push_back(toBtTransform(link->GetTransform(), METERS));
    }
    for (int iBody = 0; iBody < brs.m_bodies.size(); ++iBody) {

      btConvexShape* cShape = getConvexCollisionShape(brs.m_bodies[iBody]->getCollisionShape());
      if (cShape == NULL)
        printf("bad link: %s\n", brs.m_links[iBody]->GetName().c_str());
      assert(cShape != NULL);
      btCollisionWorld::ClosestConvexResultCallback ccc(btVector3(NAN, NAN, NAN), btVector3(NAN, NAN, NAN));
      // XXX in general there is some arbitrary child shape transform
      // but for our pr2 model loaded by openravesupport.cpp, there's only one child shape, and transform is the idenetiy
      world->convexSweepTest(cShape, oldTransforms[iBody], newTransforms[iBody], ccc, (SQPConfig::padMult
          * BulletConfig::linkPadding - dSafeCont) * METERS);
      if (ccc.hasHit() && ccc.m_hitPointWorld.getZ() > .05 * METERS) {
        out[iStep - 1].push_back(LinkCollision(.05, linkInds[iBody], ccc.m_hitPointWorld, ccc.m_hitNormalWorld, ccc.m_closestHitFraction));
      }
    }

    oldTransforms = newTransforms;
  }
  return out;
}

JointCollInfo cartToJointCollInfo(const CartCollInfo& in, RobotBasePtr robot, bool useAffine) {
  // assumes that you've set active dofs
  JointCollInfo out;
  out.dists.resize(in.size());
  out.jacs.resize(in.size());
  out.weights.resize(in.size());

  for (int iColl = 0; iColl < in.size(); ++iColl) {
    const LinkCollision& lc = in[iColl];
    out.dists[iColl] = lc.dist;
    MatrixXd jac = calcPointJacobian(robot, lc.linkInd, lc.point, useAffine);
    out.jacs[iColl] = -toVector3d(lc.normal).transpose() * jac;
    out.weights[iColl] = lc.frac;

  }
  return out;
}

TrajJointCollInfo trajCartToJointCollInfo(const TrajCartCollInfo& in, const Eigen::MatrixXd& traj, RobotBasePtr robot,
                                          const std::vector<int>& dofInds, bool useAffine) {
  ScopedRobotSave srs(robot);
  TrajJointCollInfo out(in.size());
  robot->SetActiveDOFs(dofInds);
  for (int iStep = 0; iStep < in.size(); ++iStep) {
    robot->SetActiveDOFValues(toDoubleVec(traj.row(iStep)));
    out[iStep] = cartToJointCollInfo(in[iStep], robot, useAffine);
  }
  return out;
}

void countCollisions(const TrajJointCollInfo& trajCollInfo, double safeDist, int& nNear, int& nUnsafe, int& nColl) {
  nNear = 0;
  nUnsafe = 0;
  nColl = 0;
  for (int iStep = 0; iStep < trajCollInfo.size(); ++iStep) {
    const vector<double>& dists = trajCollInfo[iStep].dists;
    for (int iColl = 0; iColl < trajCollInfo[iStep].jacs.size(); ++iColl) {
      if (dists[iColl] < 0) {
        ++nColl;
      } else if (dists[iColl] < safeDist)
        ++nUnsafe;
      else
        ++nNear;
    }
  }
}
