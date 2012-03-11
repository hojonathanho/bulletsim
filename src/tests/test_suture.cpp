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
  btScalar s = .5;
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
  psb->m_cfg.kDP = 0.5;
  psb->getCollisionShape()->setMargin(.01);
  btSoftBody::Material *pm = psb->appendMaterial();
  pm->m_kLST = 0.1;
  psb->generateBendingConstraints(2, pm);
  psb->randomizeConstraints();
  psb->setTotalMass(10, true);
  psb->generateClusters(10000);
  
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

  /*
  // create cloth anchors
  btVector3 node0 = psb->m_nodes[0].m_x;
  btVector3 node1 = psb->m_nodes[30].m_x;
  btVector3 node2 = psb->m_nodes[930].m_x;
  btVector3 node3 = psb->m_nodes[960].m_x;
  SphereObject::Ptr anchor0(new SphereObject(0, 0.005*METERS, btTransform(btQuaternion(0,0,0,1), btVector3(node0.x(),node0.y(),METERS)), true));
  anchor0->setColor(1, 0, 1, 1);
  anchor0->rigidBody->setCollisionFlags(anchor0->rigidBody->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);
  scene.env->add(anchor0);
  psb->appendAnchor(0, anchor0->rigidBody.get());
  SphereObject::Ptr anchor1(new SphereObject(0, 0.005*METERS, btTransform(btQuaternion(0,0,0,1), btVector3(node1.x(),node1.y(),METERS)), true));
  anchor1->setColor(1, 0, 1, 1);
  anchor1->rigidBody->setCollisionFlags(anchor1->rigidBody->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);
  scene.env->add(anchor1);
  psb->appendAnchor(30, anchor1->rigidBody.get());
  SphereObject::Ptr anchor2(new SphereObject(0, 0.005*METERS, btTransform(btQuaternion(0,0,0,1), btVector3(node2.x(),node2.y(),METERS)), true));
  anchor2->setColor(1, 0, 1, 1);
  anchor2->rigidBody->setCollisionFlags(anchor2->rigidBody->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);
  scene.env->add(anchor2);
  psb->appendAnchor(930, anchor2->rigidBody.get());
  SphereObject::Ptr anchor3(new SphereObject(0, 0.005*METERS, btTransform(btQuaternion(0,0,0,1), btVector3(node3.x(),node3.y(),METERS)), true));
  anchor3->setColor(1, 0, 1, 1);
  anchor3->rigidBody->setCollisionFlags(anchor3->rigidBody->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);
  scene.env->add(anchor3);
  psb->appendAnchor(960, anchor3->rigidBody.get());
  */

  /*
  // create softbody rope
  btSoftBody* rsb = btSoftBodyHelpers::CreateRope(scene.env->bullet->softBodyWorldInfo,
						  METERS*btVector3(-s,.1,z+.1),
						  METERS*btVector3(+s,.1,z+.1),
						  20, 0);
  BulletSoftObject::Ptr rope(new BulletSoftObject(rsb));
  rsb->m_cfg.piterations = 2;
  rsb->m_cfg.collisions = btSoftBody::fCollision::CL_SS
    | btSoftBody::fCollision::CL_RS
    | btSoftBody::fCollision::CL_SELF;
  rsb->m_cfg.kDF = 1.0;
  rsb->getCollisionShape()->setMargin(.03);
  //rsb->setTotalMass(1, true);
  rsb->generateClusters(0);
  scene.env->add(rope);  
  */

  // create rope
  btScalar rope_radius = .001;
  btScalar rope_mass = .01;
  btScalar segment_len = .01;
  int nLinks = 50;
  vector<btVector3> ctrlPts;
  for (int i = 0; i < nLinks; i++) {
    ctrlPts.push_back(METERS*btVector3(segment_len*i,0,1));
  }
  shared_ptr<CapsuleRope> ropePtr(new CapsuleRope(ctrlPts, METERS*rope_radius, rope_mass));
  vector<BulletObject::Ptr> children =  ropePtr->getChildren();
  for (int j=0; j<children.size(); j++) {
    children[j]->setColor(1,0,0,1);
  }
  scene.env->add(ropePtr);

  // create rope needle
  SphereObject::Ptr needle(new SphereObject(0, 0.005*METERS, btTransform(btQuaternion(0,0,0,1), btVector3(ctrlPts[0].x(),ctrlPts[0].y(),METERS*1)), true));
  //needle->rigidBody->setCollisionFlags(needle->rigidBody->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);
  scene.env->add(needle);

  btScalar len = 0;
  boost::shared_ptr<btPoint2PointConstraint> jointPtr(new btPoint2PointConstraint(*children[0]->rigidBody,*needle->rigidBody,btVector3(len/2,0,0),btVector3(-len/2,0,0)));
  //boost::shared_ptr<btGeneric6DofSpringConstraint> springPtr = createBendConstraint(len,children[0]->rigidBody,needle->rigidBody,angDamping,angStiffness,angLimit);
  scene.env->addConstraint(BulletConstraint::Ptr(new BulletConstraint(jointPtr, true)));

  Action::Ptr moveRope = needle->createMoveAction
    (btTransform(btQuaternion(0, 0, 0, 1), ctrlPts[0]+METERS*btVector3(0, 0, 1)),
     btTransform(btQuaternion(0, 0, 0, 1), ctrlPts[0]-METERS*btVector3(0, 0, 1)),
     20);

  //BoxObject::Ptr box(new BoxObect(5, METERS*btVector3(0.1, 0.1, 0.1), btTransform(btQuaternion(0, 0, 0, 1), METERS*btVector3(0, 0, 1.1))));
  //scene.env->add(box);

  scene.startViewer();
  const float dt = 0.01;
  scene.runAction(moveRope, dt);
  cout << "action done" << endl;
  while (!scene.viewer.done()) {
    scene.step(dt);
  }
  

  return 0;
}
