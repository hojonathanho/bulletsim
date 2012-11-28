#include "simulation/simplescene.h"
#include "rigid_body_dynamics.h"
#include "utils/interpolation.h"
#include "config_sqp.h"
#include "simulation/config_bullet.h"
#include "sqp_fwd.h"
#include <osgDB/ReadFile>

using namespace std;
//
//btTransform transformInterp(const btTransform& tf0, const btTransform& tf1, float frac) {
//  btTransform out;
//  out.setOrigin(tf0.getOrigin()*(1-frac) + tf1.getOrigin()*frac);
//  out.setRotation(tf0.getRotation().slerp(tf1.getRotation(), frac));
//}
#define BOX0
#define BOX1
#define FLOOR
#ifdef BOX0
RigidBodyPtr heldobj;
//boost::shared_ptr<PoseConstraint> ctrlCnt;
void moveObj(float dx, float dy) {
  heldobj->m_externalImpulse += Vector3d(dx, dy,0);
}
#endif
extern void setupBulletForSQP(btCollisionWorld* world);
int main(int argc, char* argv[]) {

  // create stack of boxes
  // make another box that moves--put equality constraints on the trajectory
  // add in costs
  SQPConfig::maxIter = 1;



  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.addGroup(SQPConfig());
  parser.addGroup(BulletConfig());
  parser.read(argc, argv);

  GeneralConfig::scale=1;
  BulletConfig::dt=1;
  initializeGRB();

  btVector3 halfExtents(.5, .5, .5);
  float mass = 1;
#ifdef BOX0
  BulletObject::Ptr box0(new BoxObject(mass, halfExtents, btTransform(btQuaternion::getIdentity(), btVector3(0, 0, .5))));
#endif
#ifdef BOX1
  BulletObject::Ptr box1(new BoxObject(mass, halfExtents, btTransform(btQuaternion::getIdentity(), btVector3(1, 0, .5))));
#endif
#ifdef FLOOR
  BulletObject::Ptr floor(new BoxObject(mass, btVector3(5, 5, .5), btTransform(btQuaternion::getIdentity(),
                                                                               btVector3(0, 0, -.5))));
#endif
  BulletObject::Ptr sphere(new SphereObject(mass, .5,btTransform(btQuaternion::getIdentity(), btVector3(2,0,.5))));
  osg::ref_ptr<osg::Image> image = osgDB::readImageFile("/home/joschu/Dropbox/Proj/ipi/smiley1.jpg");
  sphere->setTexture(image);
  Scene scene;
  setupBulletForSQP(scene.env->bullet->dynamicsWorld);
  scene.startViewer();
  util::setGlobalEnv(scene.env);
  btAlignedObjectArray<btCollisionObject*> objs = scene.env->bullet->dynamicsWorld->getCollisionObjectArray();
  for (int i = 0; i < objs.size(); ++i) {
    scene.env->bullet->dynamicsWorld->removeRigidBody(dynamic_cast<btRigidBody*> (objs[i]));
  }

  scene.env->remove(scene.ground);
#ifdef BOX0
  scene.env->add(box0);
#endif
#ifdef BOX1
   scene.env->add(box1);
#endif
#ifdef FLOOR
  scene.env->add(floor);
  floor->setColor(1,1,1,1);
#endif
#ifdef BOX0
  box0->setColor(1,0,0,.5);
#endif
#ifdef BOX1
  box1->setColor(0,1,0,.5);
#endif
  scene.env->add(sphere);

#ifdef BOX0
  scene.addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Left, boost::bind(moveObj, 0, -.01));
  scene.addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Right, boost::bind(moveObj, 0, .01));
  scene.addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Up, boost::bind(moveObj, -.01, 0));
  scene.addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Down, boost::bind(moveObj, .01, 0));
#endif


  DynSolver solver = DynSolver(scene.env->bullet->dynamicsWorld);
  TrustRegionPtr tr(new DynTrustRegion(&solver));
  solver.setTrustRegion(tr);
//  solver.addCost(CostPtr(new OverlapPenalty(&solver)));
  solver.addConstraint(ConstraintPtr(new OverlapConstraint(&solver)));
//  solver.addCost(CostPtr(new ForcePenalty(&solver)));
  solver.addConstraint(ConstraintPtr(new FrictionConstraint(&solver)));
//  solver.addCost(CostPtr(new TangentMotion(&solver)));
  solver.addCost(CostPtr(new ComplementarityCost(&solver)));
  solver.addCost(CostPtr(new FricCost(&solver)));
#ifdef BOX0
  solver.addObject(RigidBodyPtr(new RigidBody(box0->rigidBody.get(), "box0")));
  heldobj = solver.m_bodies.back();
#endif
#ifdef BOX1
  solver.addObject(RigidBodyPtr(new RigidBody(box1->rigidBody.get(), "box1")));
#endif
#ifdef FLOOR
  solver.addObject(RigidBodyPtr(new RigidBody(floor->rigidBody.get(), "floor")));

  solver.addConstraint(ConstraintPtr(new PoseConstraint(solver.m_bodies.back())));
#endif
  solver.addObject(RigidBodyPtr(new RigidBody(sphere->rigidBody.get(), "sphere")));
#ifdef BOX0
//  ctrlCnt.reset(new PoseConstraint(solver.m_bodies[0]));
//  solver.addConstraint(ctrlCnt);
#endif
  //  solver.addCost()
#define CATCH_GRB
#ifdef CATCH_GRB
  try {
#endif
    while (true) {
      solver.step();
      scene.step(0);
//      scene.idle(true);
    }
#ifdef CATCH_GRB
  }
  catch (GRBException e) {
    cout << e.getMessage() << endl;
    throw;
  }
#endif

}
