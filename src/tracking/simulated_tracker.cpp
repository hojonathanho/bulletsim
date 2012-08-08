/*
 * simulated_tracker.cpp
 *
 *  Created on: Aug 6, 2012
 *      Author: alex
 */


#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
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

using namespace Eigen;
using namespace std;

BulletSoftObject::Ptr initCloth(btSoftBodyWorldInfo& world_info, btScalar sx, btScalar sy, int rx, int ry, btScalar z) {
	int n_tex_coords = (rx - 1)*(ry -1)*12;
	float * tex_coords = new float[ n_tex_coords ];
	int fixed=0;//4+8;
	btSoftBody*		psb=btSoftBodyHelpers::CreatePatchUV(world_info,btVector3(-sx,-sy,z),
		btVector3(+sx,-sy,z),
		btVector3(-sx,+sy,z),
		btVector3(+sx,+sy,z),rx,ry,fixed,true,tex_coords);

//	psb->getCollisionShape()->setMargin(0.5);
	btSoftBody::Material* pm=psb->appendMaterial();
	pm->m_kLST		=	0.001;
	psb->generateBendingConstraints(2,pm);
	psb->setTotalMass(0.1);

	psb->generateClusters(512);
	psb->getCollisionShape()->setMargin(0.002*METERS);

	psb->m_cfg.collisions	=	0;
	psb->m_cfg.collisions += btSoftBody::fCollision::SDF_RS; ///SDF based rigid vs soft
	//psb->m_cfg.collisions += btSoftBody::fCollision::CL_RS; ///Cluster vs convex rigid vs soft
	//psb->m_cfg.collisions += btSoftBody::fCollision::VF_SS;	///Vertex vs face soft vs soft handling
	psb->m_cfg.collisions += btSoftBody::fCollision::CL_SS; ///Cluster vs cluster soft vs soft handling
	psb->m_cfg.collisions	+= btSoftBody::fCollision::CL_SELF; ///Cluster soft body self collision

	psb->m_cfg.kDF = 1;

	psb->m_cfg.piterations = 50;
	psb->m_cfg.citerations = 50;
	psb->m_cfg.diterations = 50;
//	psb->m_cfg.viterations = 10;

	// this function was not in the original bullet physics
	// this allows to swap the texture coordinates to match the swapped faces
	psb->randomizeConstraints(tex_coords);

	BulletSoftObject::Ptr bso(new BulletSoftObject(psb));
	bso->tritexcoords = new osg::Vec2Array;
	for (int i = 0; i < n_tex_coords; i+=2) {
		bso->tritexcoords->push_back(osg::Vec2f(tex_coords[i], tex_coords[i+1]));
	}

	return bso;
}

void shift(BulletSoftObject::Ptr pso, btVector3 increment) {
	btSoftBody::tNodeArray &nodes = pso->softBody->m_nodes;
	for (int i=0; i<nodes.size(); i++) {
		nodes[i].m_x += increment;
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

  	const btScalar	sx=0.32*METERS;
  	const btScalar	sy=0.46*METERS;
  	const btScalar	rx=32;
  	const btScalar	ry=46;
		const btScalar	h=0.25*METERS;
		const int		r=30;

		btTransform startTransform; btCollisionShape* capsuleShape;	btCollisionShape* planeShape;	btRigidBody* body;

		BoxObject::Ptr plane(new BoxObject(0, btVector3(sy*2,sy*2,0.01*METERS), btTransform(btQuaternion(btVector3(1,0,0), -10.0 * M_PI/180.0), btVector3(0,0,h-0.2*METERS))));
		plane->collisionShape->setMargin(0.001*METERS);
		plane->rigidBody->setFriction(1.0);
		scene.env->add(plane);

		CapsuleObject::Ptr capsule(new CapsuleObject(0, 0.02*METERS,sx*2, btTransform(btQuaternion(btVector3(0,0,1), 0.0), btVector3(0,-sy*0.5,h-0.04*METERS))));
		capsule->collisionShape->setMargin(0.005*METERS);
		capsule->rigidBody->setFriction(0.0);
		scene.env->add(capsule);

		BoxObject::Ptr box(new BoxObject(0, btVector3(TrackingConfig::pointOutlierDist*METERS/2.0,TrackingConfig::pointOutlierDist*METERS/2.0,TrackingConfig::pointOutlierDist*METERS/2.0), btTransform(btQuaternion(btVector3(1,0,0), 0.0), btVector3(0,0,h-0.1*METERS))));
		box->collisionShape->setMargin(0.001*METERS);
		box->rigidBody->setFriction(1.0);
		box->setColor(1,0,0,1);
		scene.env->add(box);

		BulletSoftObject::Ptr cloth = initCloth(scene.env->bullet->softBodyWorldInfo, sx, sy, rx, ry, h);
		cv::Mat flag_tex = cv::imread("/home/alex/Desktop/flag.jpg");
		cloth->setTexture(flag_tex.clone());
		scene.env->add(cloth);

		BulletSoftObject::Ptr observed_cloth = initCloth(scene.env->bullet->softBodyWorldInfo, sx, sy, rx, ry, h);
		observed_cloth->setTexture(flag_tex.clone());
		TrackedObject::Ptr observed_tracked(new TrackedTowel(observed_cloth, rx, ry));
		observed_tracked->init();

		TrackedObject::Ptr trackedObj(new TrackedTowel(cloth, rx, ry));
		trackedObj->init();
		EverythingIsVisible::Ptr visInterface(new EverythingIsVisible());

		TrackedObjectFeatureExtractor::Ptr objectFeatures(new TrackedObjectFeatureExtractor(trackedObj));
		TrackedObjectFeatureExtractor::Ptr observedFeatures(new TrackedObjectFeatureExtractor(observed_tracked));
		PhysicsTracker::Ptr alg(new PhysicsTracker(objectFeatures, observedFeatures, visInterface));
		PhysicsTrackerVisualizer::Ptr trakingVisualizer(new PhysicsTrackerVisualizer(&scene, alg));

		bool applyEvidence = true;
		scene.addVoidKeyCallback('a',boost::bind(toggle, &applyEvidence));
		scene.addVoidKeyCallback('=',boost::bind(&EnvironmentObject::adjustTransparency, trackedObj->getSim(), 0.1f));
		scene.addVoidKeyCallback('-',boost::bind(&EnvironmentObject::adjustTransparency, trackedObj->getSim(), -0.1f));
		scene.addVoidKeyCallback('q',boost::bind(exit, 0));
		scene.addVoidKeyCallback('u',boost::bind(shift, observed_cloth, btVector3(0,0,0.01*METERS)));
		scene.addVoidKeyCallback('j',boost::bind(shift, observed_cloth, btVector3(0,0,-0.01*METERS)));
		scene.addVoidKeyCallback('h',boost::bind(shift, observed_cloth, btVector3(0,-0.01*METERS,0)));
		scene.addVoidKeyCallback('k',boost::bind(shift, observed_cloth, btVector3(0,0.01*METERS,0)));

		btSoftBody::tNodeArray& target_nodes = observed_cloth->softBody->m_nodes;

		scene.startViewer();

    boost::posix_time::ptime sim_time = boost::posix_time::microsec_clock::local_time();
    while (true) {
    	//Update the inputs of the featureExtractors and visibilities (if they have any inputs)

    	//Do iteration
    	alg->updateFeatures();
			alg->expectationStep();
			alg->maximizationStep(applyEvidence);

			trakingVisualizer->update();
    	scene.step(0.03);
    	cout << (boost::posix_time::microsec_clock::local_time() - sim_time).total_milliseconds() << endl;
    	sim_time = boost::posix_time::microsec_clock::local_time();
    }

    return 0;
}
