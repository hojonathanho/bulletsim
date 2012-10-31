
#include "sqp_dynamics.h"
#include "simulation/simplescene.h"
#include "utils/interpolation.h"
#include "config_sqp.h"

using namespace std;
//
//btTransform transformInterp(const btTransform& tf0, const btTransform& tf1, float frac) {
//	btTransform out;
//	out.setOrigin(tf0.getOrigin()*(1-frac) + tf1.getOrigin()*frac);
//	out.setRotation(tf0.getRotation().slerp(tf1.getRotation(), frac));
//}


btRigidBody* heldobj;
void drive(float dx, float dy) {
  btVector3 pos = heldobj->getCenterOfMassPosition();
  pos.setX(pos.x() + dx);
  pos.setY(pos.y() + dy);
  heldobj->setCenterOfMassTransform(btTransform(btQuaternion::getIdentity(), pos));
  heldobj->getMotionState()->setWorldTransform(btTransform(btQuaternion::getIdentity(), pos));
}

int main(int argc, char* argv[]) {

	// create stack of boxes
	// make another box that moves--put equality constraints on the trajectory
	// add in costs
	
  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.addGroup(SQPConfig());
  parser.read(argc, argv);

  SQPConfig::maxIter = 5;

  initializeGRB();

	btVector3 halfExtents(.5,.5,.5);
	float mass=1;
	BulletObject::Ptr box0(new BoxObject(mass,halfExtents, 
		btTransform(btQuaternion::getIdentity(),btVector3(0,0,.5))));
  BulletObject::Ptr box1(new BoxObject(mass,halfExtents,
    btTransform(btQuaternion::getIdentity(),btVector3(1,0,.5))));
	
	BulletObject::Ptr floor(new BoxObject(mass, btVector3(5,5,.5), 
		btTransform(btQuaternion::getIdentity(), btVector3(0,0,-.4))));


  Scene scene;
  scene.startViewer();

  btAlignedObjectArray<btCollisionObject*> objs = scene.env->bullet->dynamicsWorld->getCollisionObjectArray();
  for (int i=0; i < objs.size(); ++i) {
    scene.env->bullet->dynamicsWorld->removeRigidBody(dynamic_cast<btRigidBody*>(objs[i]));
  }
	
	scene.env->add(box0);
	scene.env->add(box1);
	scene.env->add(floor);

	DynSolver solver = DynSolver(scene.env->bullet->dynamicsWorld);
	DynErrCostPtr dynCost(new DynErrCost(&solver, 1, 100));
	DynOverlapCostPtr overlapCost(new DynOverlapCost(&solver, 1000));
//	solver.addCost(dynCost);
	VelCostPtr velCost(new VelCost(&solver, .1));
	TrustRegionPtr tr(new DynTrustRegion(&solver));
  solver.addCost(overlapCost);
  solver.addCost(velCost);
  solver.setTrustRegion(tr);
  solver.addObject(box0->rigidBody.get(),"box0");
  solver.addObject(box1->rigidBody.get(),"box1");
  solver.addObject(floor->rigidBody.get(),"floor");
  solver.constrainPose(floor->rigidBody.get());
  solver.initialize();


  heldobj = box1->rigidBody.get();
  scene.addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Left, boost::bind(drive, .05, 0));
  scene.addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Right, boost::bind(drive, -.05, 0));
  scene.addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Up, boost::bind(drive, 0, .05));
  scene.addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Down, boost::bind(drive, 0, -.05));

  while (true) {
    if (SQPConfig::pauseEachIter) scene.idle(true);
    Optimizer::OptStatus status = solver.optimize();
    assert(status == Optimizer::CONVERGED);
    scene.step(0);
    solver.getPosesFromWorld();
    solver.constrainPose(heldobj);
    tr->resetTrustRegion();
  }
	

}
