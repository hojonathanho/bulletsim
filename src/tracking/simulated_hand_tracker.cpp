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
	observed_tracked->init();

	TrackedObject::Ptr trackedObj(new TrackedCompound(hand, scene.env->bullet->dynamicsWorld));
	trackedObj->init();
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
	scene.addVoidKeyCallback('=', boost::bind(&EnvironmentObject::adjustTransparency, trackedObj->getSim(), 0.1f));
	scene.addVoidKeyCallback('-',boost::bind(&EnvironmentObject::adjustTransparency,trackedObj->getSim(), -0.1f));

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
