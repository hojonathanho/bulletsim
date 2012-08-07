#include "simulation/simplescene.h"
#include "simulation/plotting.h"
#include "simulation/basicobjects.h"
#include <boost/foreach.hpp>
#include "utils/conversions.h"
#include "simulation/util.h"
#include "simulation/hand.h"
#include "tracking/surface_sampling.h"
using namespace std;


int main(int argc, char* argv[]) {
	Parser parser;
	parser.addGroup(GeneralConfig());
	parser.addGroup(BulletConfig());
	parser.read(argc, argv);
	Scene scene;

	Hand::Ptr hand = makeHand(19, btTransform(btQuaternion::getIdentity(), btVector3(0,0,0.1*METERS)));
	scene.env->add(hand);

	BulletObject::Ptr box(new BoxObject(1,btVector3(1,1,1), btTransform(btQuaternion::getIdentity(), btVector3(3,3,3))));
//	scene.env->add(box);

	std::set<btRigidBody*> objs;
	BOOST_FOREACH(BulletObject::Ptr child, hand->getChildren()) {
		objs.insert(child->rigidBody.get());
	}

//	objs.insert(box->rigidBody.get());

	vector<btVector3> surfacePoints;
	vector<btRigidBody*> ownerBodies;
	getSurfacePoints(objs, scene.env->bullet->dynamicsWorld, surfacePoints, ownerBodies);


	vector<btVector3> dsSurfacePoints;
	vector<btRigidBody*> dsOwnerBodies;
	downsamplePointsOnEachBody(surfacePoints, ownerBodies, .015*METERS, dsSurfacePoints, dsOwnerBodies);

	printf("%i points found\n", surfacePoints.size());
	printf("%i points after downsampling\n", dsSurfacePoints.size());

	PlotPoints::Ptr dsPoints(new PlotPoints(10));
	dsPoints->setDefaultColor(0,1,0,1);
	dsPoints->setPoints(dsSurfacePoints);
	scene.env->add(dsPoints);

	PlotPoints::Ptr origPoints(new PlotPoints(5));
	origPoints->setDefaultColor(1,1,0,1);
	origPoints->setPoints(surfacePoints);
	scene.env->add(origPoints);

	hand->setColor(0,1,1,.6);
	scene.startViewer();
	scene.idle(true);
	scene.startLoop();

}
