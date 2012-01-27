#include "make_bodies.h"
#include "config.h"
#include "bullet_io.h"
#include <boost/shared_ptr.hpp>
#include <BulletSoftBody/btSoftBodyHelpers.h>

using boost::shared_ptr;


BulletObject::Ptr makeTable(const vector<btVector3>& corners, float thickness) {
  btVector3 origin = (corners[0] + corners[2])/2;
  origin[2] -= thickness/2;
  btVector3 halfExtents = (corners[2] - corners[0]).absolute()/2;
  halfExtents[2] = thickness/2;

  cout << "origin: " << origin << endl;
  cout << "halfExtents: " << halfExtents << endl;

  shared_ptr<btDefaultMotionState> ms(new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),origin)));  
  return BulletObject::Ptr(new BoxObject(0,halfExtents,ms));

}

BulletSoftObject::Ptr makeTowel(const vector<btVector3>& points, btSoftBodyWorldInfo& worldInfo) {
  btVector3 offset(0,0,.01*METERS);
  btSoftBody* psb=btSoftBodyHelpers::CreatePatch(worldInfo,
						 points[0]+offset,
						 points[1]+offset,
						 points[3]+offset,
						 points[2]+offset,
						 45, 31,
						 0/*1+2+4+8*/, true);
  cout << "points[0] " << points[0].x() << " " << points[0].y() << " " << points[0].z() << endl;
  psb->getCollisionShape()->setMargin(.01*METERS);
  btSoftBody::Material* pm=psb->appendMaterial();
  pm->m_kLST		=	5*0.1;
  pm->m_kAST = 5*0.1;
  //	pm->m_flags		-=	btSoftBody::fMaterial::DebugDraw;
  psb->generateBendingConstraints(2, pm);
  psb->setTotalMass(1);

  return BulletSoftObject::Ptr(new BulletSoftObject(psb));
}

BulletSoftObject::Ptr makeSelfCollidingTowel(const vector<btVector3>& points, btSoftBodyWorldInfo& worldInfo) {
  btVector3 offset(0,0,.01*METERS);
    btSoftBody *psb = btSoftBodyHelpers::CreatePatch(
        worldInfo,
        points[0]+offset,
        points[1]+offset,
        points[3]+offset,
        points[2]+offset,
        45, 31,
        0, true);

    psb->m_cfg.piterations = 2;
    psb->m_cfg.collisions = btSoftBody::fCollision::CL_SS
        | btSoftBody::fCollision::CL_RS
        | btSoftBody::fCollision::CL_SELF;
    psb->m_cfg.kDF = 1.0;
    psb->getCollisionShape()->setMargin(0.05);
    btSoftBody::Material *pm = psb->appendMaterial();
    pm->m_kLST = 0.1;
    pm->m_kAST = 0.1;
    psb->generateBendingConstraints(2, pm);
    psb->randomizeConstraints();
    psb->setTotalMass(1, true);
    psb->generateClusters(0);

    return BulletSoftObject::Ptr(new BulletSoftObject(psb));
}
