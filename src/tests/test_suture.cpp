#include "simulation/rope.h"
#include "simulation/softbodies.h"
#include "simulation/basescene.h"
#include "simulation/util.h"
#include "simulation/config_bullet.h"
#include "simulation/bullet_io.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>

using boost::shared_ptr;
using namespace util;

int main(int argc, char *argv[]) {
  GeneralConfig::scale = 20.;

  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.addGroup(BulletConfig());
  parser.read(argc, argv);

  BaseScene scene;

  // create cloth
  btScalar s = 1;
  btScalar z = 1;
  btSoftBody* psb = btSoftBodyHelpers::CreatePatch(scene.env->bullet->softBodyWorldInfo,
						   METERS*btVector3(-s,-s,z),
						   METERS*btVector3(+s,-s,z),
						   METERS*btVector3(-s,+s,z),
						   METERS*btVector3(+s,+s,z),
						   31, 31,
						   0, true);
  
  psb->m_cfg.piterations = 2;
  psb->m_cfg.collisions = btSoftBody::fCollision::CL_SS
    | btSoftBody::fCollision::CL_RS
    | btSoftBody::fCollision::CL_SELF;
  psb->m_cfg.kDF = 1.0;
  psb->getCollisionShape()->setMargin(0.01);
  btSoftBody::Material *pm = psb->appendMaterial();
  pm->m_kLST = 0.1;
  psb->generateBendingConstraints(2, pm);
  psb->randomizeConstraints();
  psb->setTotalMass(10, true);
  psb->generateClusters(0);
  /*
  psb->getCollisionShape()->setMargin(0.4);
  btSoftBody::Material* pm=psb->appendMaterial();
  pm->m_kLST = 0.4;
  //pm->m_flags -= btSoftBody::fMaterial::DebugDraw;
  psb->generateBendingConstraints(2, pm);
  psb->setTotalMass(150);
  */
  BulletSoftObject::Ptr cloth(new BulletSoftObject(psb));

  scene.env->add(cloth);

  // create rope
  btScalar rope_radius = .001;
  btScalar rope_mass = .01;
  btScalar segment_len = .01;
  int nLinks = 200;
  vector<btVector3> ctrlPts;
  for (int i = 0; i < nLinks; i++) {
    ctrlPts.push_back(METERS*btVector3(-1+.5+segment_len*i,0,1));
  }
  shared_ptr<CapsuleRope> ropePtr(new CapsuleRope(ctrlPts, METERS*rope_radius, rope_mass));
  vector<BulletObject::Ptr> children =  ropePtr->getChildren();
  for (int j=0; j<children.size(); j++) {
    children[j]->setColor(1,0,0,1);
  }
  scene.env->add(ropePtr);
  
  //BoxObject::Ptr box(new BoxObject(5, btVector3(0.1, 0.1, 0.1), btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 1.1))));
  //scene.env->add(box);

  scene.startViewer();
  while (!scene.viewer.done()) {
    scene.step(.01);
  }

  return 0;
}
