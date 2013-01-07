/*
 * simulated_tracker.cpp
 *
 *  Created on: Aug 6, 2012
 *      Author: alex
 */

/*
 * simulated_tracker.cpp
 *
 *  Created on: Aug 6, 2012
 *      Author: alex
 */

#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "simulation/util.h"
#include "simulation/config_bullet.h"
#include "simulation/config_viewer.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <boost/thread.hpp>
#include <stdio.h>
#include <iostream>
#include "utils/conversions.h"

#include "clouds/utils_pcl.h"
#include "tracking/utils_tracking.h"
#include "utils/logging.h"
#include "utils/utils_vector.h"
#include "tracking/visibility.h"
#include "physics_tracker.h"
#include "feature_extractor.h"
#include "simulation/simplescene.h"
#include "tracking/config_tracking.h"
#include "tracking/tracked_object.h"
#include "simulation/hand.h"
#include "tracked_compound.h"

using namespace Eigen;
using namespace std;

void shift(GenericCompoundObject::Ptr gco, btVector3 increment) {
	BOOST_FOREACH(BulletObject::Ptr obj, gco->children) {
		btRigidBody* rb = obj->rigidBody.get();
		rb->setCenterOfMassTransform(btTransform(
				btQuaternion::getIdentity(), increment) *
				rb->getCenterOfMassTransform());
	}
}

int main(int argc, char *argv[]) {
  Eigen::internal::setNbThreads(2);

  GeneralConfig::scale = 100;
  BulletConfig::maxSubSteps = 0;
  BulletConfig::gravity = btVector3(0,0,-0.1);

  Parser parser;
  parser.addGroup(TrackingConfig());
  parser.addGroup(GeneralConfig());
  parser.addGroup(BulletConfig());
  parser.addGroup(ViewerConfig());
  parser.read(argc, argv);

  // set up scene
  Scene scene;
  scene.startViewer();
  util::setGlobalEnv(scene.env);

	btVector3 initHandPos(0,0,0.3*METERS);
	btTransform initHandTrans = btTransform(btQuaternion(btVector3(1,0,0), M_PI/2.0), initHandPos);
	HumanHandObject::Ptr hand(new HumanHandObject(scene.rave, initHandTrans));
	scene.env->add(hand);

	HumanHandObject::Ptr observed_hand(new HumanHandObject(scene.rave, initHandTrans));
	BulletInstance::Ptr bullet2(new BulletInstance());
	BOOST_FOREACH(BulletObject::Ptr obj, observed_hand->children) bullet2->dynamicsWorld->addRigidBody(obj->rigidBody.get());
	TrackedObject::Ptr observed_tracked(new TrackedCompound(observed_hand, bullet2->dynamicsWorld));

	TrackedObject::Ptr trackedObj(new TrackedCompound(hand));
	EverythingIsVisible::Ptr visInterface(new EverythingIsVisible());

	TrackedObjectFeatureExtractor::Ptr objectFeatures(new TrackedObjectFeatureExtractor(trackedObj));
	TrackedObjectFeatureExtractor::Ptr observedFeatures(new TrackedObjectFeatureExtractor(observed_tracked));
	PhysicsTracker::Ptr alg(new PhysicsTracker(objectFeatures, observedFeatures, visInterface));
	PhysicsTrackerVisualizer::Ptr trakingVisualizer(new PhysicsTrackerVisualizer(&scene, alg));

	bool applyEvidence = true;
  scene.addVoidKeyCallback('a',boost::bind(toggle, &applyEvidence), "apply evidence");
  bool exit_loop = false;
  scene.addVoidKeyCallback('q',boost::bind(toggle, &exit_loop), "exit");

  scene.addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Page_Up,boost::bind(shift, observed_hand, btVector3(0,0,0.01*METERS)), "move synthetic data up");
	scene.addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Page_Down,boost::bind(shift, observed_hand, btVector3(0,0,-0.01*METERS)), "move synthetic data down");
	scene.addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Left,boost::bind(shift, observed_hand, btVector3(0,-0.01*METERS,0)), "move synthetic data left");
	scene.addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Right,boost::bind(shift, observed_hand, btVector3(0,0.01*METERS,0)), "move synthetic data right");
	scene.addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Up,boost::bind(shift, observed_hand, btVector3(-0.01*METERS,0,0)), "move synthetic data farther");
	scene.addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Down,boost::bind(shift, observed_hand, btVector3(0.01*METERS,0,0)), "move synthetic data closer");

	while (!exit_loop ) {
		//Update the inputs of the featureExtractors and visibilities (if they have any inputs)

		//Do iteration
		alg->updateFeatures();
		alg->expectationStep();
		alg->maximizationStep(applyEvidence);

		trakingVisualizer->update();
//		scene.step(0.03);
		//scene.step(.03,2,.015);

		scene.draw();
	}

	return 0;
}



#if 0
#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "simulation/util.h"
#include "simulation/config_bullet.h"
#include "simulation/config_viewer.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <boost/thread.hpp>
#include <stdio.h>
#include <iostream>
#include "utils/conversions.h"

#include "clouds/utils_pcl.h"
#include "tracking/utils_tracking.h"
#include "utils/logging.h"
#include "utils/utils_vector.h"
#include "tracking/visibility.h"
#include "tracking/simple_physics_tracker.h"
#include "simulation/simplescene.h"
#include "tracking/config_tracking.h"
#include "tracking/tracked_object.h"

#include "simulation/hand.h"
#include "tracking/tracked_compound.h"

using namespace Eigen;
using namespace std;

void shift(GenericCompoundObject::Ptr gco, btVector3 increment) {
	BOOST_FOREACH(BulletObject::Ptr obj, gco->children) {
		btRigidBody* rb = obj->rigidBody.get();
		rb->setCenterOfMassTransform(btTransform(
				btQuaternion::getIdentity(), increment) *
				rb->getCenterOfMassTransform());
	}
}

int main(int argc, char *argv[]) {
	GeneralConfig::scale = 100.;
	BulletConfig::maxSubSteps = 0;

	Parser parser;
	parser.addGroup(GeneralConfig());
	parser.addGroup(BulletConfig());
	parser.addGroup(SceneConfig());
	parser.addGroup(TrackingConfig());
	parser.read(argc, argv);

	Scene scene;

	btTransform startTransform;
	btCollisionShape* capsuleShape;
	btCollisionShape* planeShape;
	btRigidBody* body;

	Hand::Ptr hand = makeHand(19, btTransform(btQuaternion::getIdentity(), btVector3(0, 0, 0.1 * METERS)));
	scene.env->add(hand);

	Hand::Ptr observed_hand = makeHand(19, btTransform(btQuaternion::getIdentity(), btVector3(0, 0, 0.1 * METERS)));

	BulletInstance::Ptr bullet2(new BulletInstance());
	BOOST_FOREACH(BulletObject::Ptr obj, observed_hand->children) bullet2->dynamicsWorld->addRigidBody(obj->rigidBody.get());
	TrackedObject::Ptr observed_tracked(new TrackedCompound(observed_hand,bullet2->dynamicsWorld));

	TrackedObject::Ptr trackedObj(new TrackedCompound(hand));
	EverythingIsVisible::Ptr visInterface(new EverythingIsVisible());
	SimplePhysicsTracker alg(trackedObj, visInterface, scene.env);

	scene.addVoidKeyCallback('c', boost::bind(toggle, &alg.m_enableCorrPlot));
	scene.addVoidKeyCallback('C', boost::bind(toggle, &alg.m_enableCorrPlot));
	scene.addVoidKeyCallback('e', boost::bind(toggle, &alg.m_enableEstPlot));
	scene.addVoidKeyCallback('E',
			boost::bind(toggle, &alg.m_enableEstTransPlot));
	scene.addVoidKeyCallback('o', boost::bind(toggle, &alg.m_enableObsPlot));
	scene.addVoidKeyCallback('O',
			boost::bind(toggle, &alg.m_enableObsTransPlot));
	scene.addVoidKeyCallback('i', boost::bind(toggle,
			&alg.m_enableObsInlierPlot));
	scene.addVoidKeyCallback('I', boost::bind(toggle,
			&alg.m_enableObsInlierPlot));
	scene.addVoidKeyCallback('b', boost::bind(toggle, &alg.m_enableDebugPlot));
	scene.addVoidKeyCallback('B', boost::bind(toggle, &alg.m_enableDebugPlot));
	scene.addVoidKeyCallback('a', boost::bind(toggle, &alg.m_applyEvidence));

	scene.addVoidKeyCallback('[', boost::bind(add, &alg.m_count, -1));
	scene.addVoidKeyCallback(']', boost::bind(add, &alg.m_count, 1));

	scene.addVoidKeyCallback('u',boost::bind(shift, observed_hand, btVector3(0,0,0.01*METERS)));
	scene.addVoidKeyCallback('j',boost::bind(shift, observed_hand, btVector3(0,0,-0.01*METERS)));
	scene.addVoidKeyCallback('h',boost::bind(shift, observed_hand, btVector3(0,-0.01*METERS,0)));
	scene.addVoidKeyCallback('k',boost::bind(shift, observed_hand, btVector3(0,0.01*METERS,0)));
	bool quit = false;
	scene.addVoidKeyCallback('q', boost::bind(toggle, &quit));

	scene.startViewer();

	boost::posix_time::ptime sim_time =
			boost::posix_time::microsec_clock::local_time();
	while (true) {
		if (quit)
			break;

		MatrixXf xyz = toEigenMatrix(observed_tracked->getPoints());
		MatrixXf rgb = observed_tracked->getColors();
		assert(xyz.rows() == rgb.rows());
		ColorCloudPtr cloud(new ColorCloud());
		for (int i = 0; i < xyz.rows(); i++) {
			ColorPoint pt(rgb(i, 2) * 255.0, rgb(i, 1) * 255.0, rgb(i, 0) * 255.0);
			pt.x = xyz(i, 0);
			pt.y = xyz(i, 1);
			pt.z = xyz(i, 2);
			cloud->push_back(pt);
		}
		printf("cloud size: %i\n", observed_tracked->m_nNodes);
		alg.updateInput(cloud);
		alg.doIteration();

		scene.step(0.03);
		cout << (boost::posix_time::microsec_clock::local_time() - sim_time).total_milliseconds() << endl;
		sim_time = boost::posix_time::microsec_clock::local_time();
	}

	return 0;
}
#endif
