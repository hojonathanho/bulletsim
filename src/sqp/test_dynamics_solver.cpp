
#include "sqp_dynamics.h"
#include "simulation/simplescene.h"
#include "utils/interpolation.h"

using namespace std;
//
//btTransform transformInterp(const btTransform& tf0, const btTransform& tf1, float frac) {
//	btTransform out;
//	out.setOrigin(tf0.getOrigin()*(1-frac) + tf1.getOrigin()*frac);
//	out.setRotation(tf0.getRotation().slerp(tf1.getRotation(), frac));
//}

Eigen::MatrixXd makeTraj(const Eigen::VectorXd& startJoints, const Eigen::VectorXd& endJoints,
    int nSteps) {
  assert(startJoints.size() == endJoints.size());
  Eigen::MatrixXd startEndJoints(2, startJoints.size());
  startEndJoints.row(0) = startJoints;
  startEndJoints.row(1) = endJoints;
  return interp2d(VectorXd::LinSpaced(nSteps, 0, 1), VectorXd::LinSpaced(2, 0, 1), startEndJoints);

}

int main(int argc, char* argv[]) {

	// create stack of boxes
	// make another box that moves--put equality constraints on the trajectory
	// add in costs
	
	btVector3 halfExtents(.5,.5,.5);
	float mass=1;
	BulletObject::Ptr box0(new BoxObject(mass,halfExtents, 
		btTransform(btQuaternion::getIdentity(),btVector3(0,0,.5))));
	BulletObject::Ptr box1(new BoxObject(mass,halfExtents, 
		btTransform(btQuaternion::getIdentity(),btVector3(0,1,.5))));
	BulletObject::Ptr box2(new BoxObject(mass,halfExtents, 
		btTransform(btQuaternion::getIdentity(),btVector3(0,0,1.5))));
	BulletObject::Ptr box3(new BoxObject(mass,halfExtents, 
		btTransform(btQuaternion::getIdentity(),btVector3(0,1,1.5))));
	
	BulletObject::Ptr boxQ(new BoxObject(mass,halfExtents, 
		btTransform(btQuaternion::getIdentity(),btVector3(-2,.5,.5))));
	
	VectorXd start(3); start << -2,.5,.5;
	VectorXd end(3); end << 2,.5,.5;
	
	BulletObject::Ptr floor(new BoxObject(mass, btVector3(5,5,.5), 
		btTransform(btQuaternion::getIdentity(), btVector3(0,0,-.5))));
	
	int T=50;

	Scene scene;

	DynamicsSolver solver(scene.env->bullet->dynamicsWorld, T);
	NoFricDynCostPtr dynCost(new NoFricDynCost());
	OverlapCostPtr overlapCost(new OverlapCost());
	solver.addCost(dynCost);
  solver.addCost(overlapCost);
  solver.addObject(box0->rigidBody.get());
  solver.addObject(box1->rigidBody.get());
  solver.addObject(box2->rigidBody.get());
  solver.addObject(box3->rigidBody.get());
  solver.addObject(boxQ->rigidBody.get());
  solver.addObject(floor->rigidBody.get());
  solver.constrainPoses(boxQ->rigidBody.get(), makeTraj(start, end, T)); // boxQ has a fixed trajectory
  solver.setStatic(floor->rigidBody.get()); // floor doesn't move
  solver.initialize();
  solver.optimize();
	
	
}
