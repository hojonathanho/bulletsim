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

struct CollisionCollector : public btCollisionWorld::ContactResultCallback {
  std::vector<Collision> m_collisions;
  btScalar addSingleResult(btManifoldPoint& pt, const btCollisionObject *colObj0, int, int,
                           const btCollisionObject *colObj1, int, int) {
    m_collisions.push_back(Collision(colObj0, colObj1, pt.m_positionWorldOnA, pt.m_positionWorldOnB,
                                     pt.m_normalWorldOnB, pt.m_distance1));
    return 0;
  }
};

TrajCartCollInfo collectTrajCollisions(const Eigen::MatrixXd& traj, RaveRobotObject* rro,
                                       const std::vector<int>& dofInds, bool useAffine) {
  RobotBasePtr& robot = rro->robot;
  ScopedRobotSave srs(robot);
  btCollisionWorld* world = rro->getEnvironment()->bullet->dynamicsWorld;

  vector<KinBody::LinkPtr> links;
  vector<int> linkInds;
  vector<btRigidBody*> bodies;
  getAffectedLinks2(rro->robot, dofInds, links, linkInds);
  cout << "AFFECT LINKS NAMES" << endl;
  BOOST_FOREACH(KinBody::LinkPtr& link, links) {
    bodies.push_back(rro->associatedObj(link)->rigidBody.get());
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
        if (point.getZ() > .05 * METERS)
          out[iStep].push_back(LinkCollision(dist, linkInds[iBody], point, normal));
      }
    }
  }
  return out;
}

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

TrajCartCollInfo continuousTrajCollisions(const Eigen::MatrixXd& traj, OpenRAVE::RobotBasePtr robot,
                                          BulletRaveSyncher& brs, btCollisionWorld* world,
                                          const std::vector<int>& dofInds, float dSafeCont) {
  ScopedRobotSave srs(robot);
  vector<int> linkInds;
  BOOST_FOREACH(KinBody::LinkPtr link, brs.m_links) {
    linkInds.push_back(link->GetIndex());
  }

  robot->SetActiveDOFs(dofInds);
  robot->SetActiveDOFValues(toDoubleVec(traj.row(0)));

  vector<btTransform> oldTransforms;
  BOOST_FOREACH(OpenRAVE::KinBody::LinkPtr link, brs.m_links) {
    oldTransforms.push_back(toBtTransform(link->GetTransform(), METERS));
  }

  TrajCartCollInfo out(traj.rows() - 1);

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
        out[iStep - 1].push_back(LinkCollision(.05, linkInds[iBody], ccc.m_hitPointWorld, ccc.m_hitNormalWorld));
        out[iStep - 1][0].frac = ccc.m_closestHitFraction;
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

  for (int iColl = 0; iColl < in.size(); ++iColl) {
    const LinkCollision& lc = in[iColl];
    out.dists[iColl] = lc.dist;
    MatrixXd jac = calcPointJacobian(robot, lc.linkInd, lc.point, useAffine);
    out.jacs[iColl] = -toVector3d(lc.normal).transpose() * jac;

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
