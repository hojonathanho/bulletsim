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

using namespace Eigen;
using namespace std;

//HACK
osg::Vec2Array* generateTexCoordinates(BulletSoftObject::Ptr cloth, cv::Mat image, btScalar sx, btScalar sy) {
	btSoftBody::tFaceArray faces;
	for (int i = 0; i < cloth->softBody->m_faces.size(); ++i) {
		faces.push_back(cloth->softBody->m_faces[i]);
	}
	osg::Vec2Array* tritexcoords = new osg::Vec2Array;
	for (int j=0; j<faces.size(); j++) {
		for (int c=0; c<3; c++) {
			float u = (faces[j].m_n[c]->m_x.y()+sy)/(2*sy);
			float v = (faces[j].m_n[c]->m_x.x()+sx)/(2*sx);
			tritexcoords->push_back(osg::Vec2f(u,1.0-v));
		}
	}
	return tritexcoords;
}

void shift(BulletSoftObject::Ptr pso, btVector3 increment) {
	btSoftBody::tNodeArray &nodes = pso->softBody->m_nodes;
	for (int i=0; i<nodes.size(); i++) {
		nodes[i].m_x += increment;
	}
}

//HACK
cv::Mat stitchImages(cv::Mat src) {
	cv::Mat dst(2*src.rows, 2*src.cols, src.type());
	cv::Mat dst_roi = dst(cv::Rect(0, 0, src.cols, src.rows));
	src.copyTo(dst_roi);
	dst_roi = dst(cv::Rect(src.cols, 0, src.cols, src.rows));
	src.copyTo(dst_roi);
	dst_roi = dst(cv::Rect(0, src.rows, src.cols, src.rows));
	src.copyTo(dst_roi);
	dst_roi = dst(cv::Rect(src.cols, src.rows, src.cols, src.rows));
	src.copyTo(dst_roi);
	return dst;
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

	const btScalar	sx=0.32*METERS;
	const btScalar	sy=0.46*METERS;
	const btScalar	rx=26;
	const btScalar	ry=23;
	const btScalar	h=0.25*METERS;
	const btScalar	mass=0.1;

	cv::Mat flag_tex = cv::imread(string(getenv("BULLETSIM_SOURCE_DIR")) + "/data/rgbs/flag_tex.jpg");
  vector<btVector3> cloth_corners;
	cloth_corners.push_back(btVector3(+sx,+sy,h));
	cloth_corners.push_back(btVector3(+sx,-sy,h));
  cloth_corners.push_back(btVector3(-sx,-sy,h));
	cloth_corners.push_back(btVector3(-sx,+sy,h));
	BulletSoftObject::Ptr cloth = makeCloth(cloth_corners, rx, ry, mass);
	if (flag_tex.empty()) cloth->setColor(1,0,0,1);
	else cloth->setTexture(flag_tex.clone(), generateTexCoordinates(cloth,flag_tex,sx,sy));
	scene.env->add(cloth);

	cv::Mat flag_observed_tex = stitchImages(flag_tex);
	vector<btVector3> observed_cloth_corners;
	observed_cloth_corners.push_back(btVector3(+2*sx,+2*sy,h));
	observed_cloth_corners.push_back(btVector3(+2*sx,-2*sy,h));
	observed_cloth_corners.push_back(btVector3(-2*sx,-2*sy,h));
	observed_cloth_corners.push_back(btVector3(-2*sx,+2*sy,h));
	BulletSoftObject::Ptr observed_cloth = makeCloth(observed_cloth_corners, 2*rx, 2*ry, 4*mass);
	if (flag_tex.empty()) observed_cloth->setColor(1,0,0,1);
	else observed_cloth->setTexture(flag_observed_tex.clone(), generateTexCoordinates(observed_cloth,flag_observed_tex,2*sx,2*sy));
	TrackedObject::Ptr observed_tracked(new TrackedCloth(observed_cloth));
	observed_tracked->init();

	cv::imwrite("/home/alex/Desktop/flag4.jpg", flag_observed_tex);

	TrackedObject::Ptr trackedObj(new TrackedCloth(cloth));
	trackedObj->init();
	EverythingIsVisible::Ptr visInterface(new EverythingIsVisible());

	TrackedObjectFeatureExtractor::Ptr objectFeatures(new TrackedObjectFeatureExtractor(trackedObj));
	TrackedObjectFeatureExtractor::Ptr observedFeatures(new TrackedObjectFeatureExtractor(observed_tracked));
	PhysicsTracker::Ptr alg(new PhysicsTracker(objectFeatures, observedFeatures, visInterface));
	PhysicsTrackerVisualizer::Ptr trakingVisualizer(new PhysicsTrackerVisualizer(&scene, alg));

	bool applyEvidence = true;
  scene.addVoidKeyCallback('a',boost::bind(toggle, &applyEvidence), "apply evidence");
  scene.addVoidKeyCallback('=',boost::bind(&EnvironmentObject::adjustTransparency, trackedObj->getSim(), 0.1f), "increase opacity");
  scene.addVoidKeyCallback('-',boost::bind(&EnvironmentObject::adjustTransparency, trackedObj->getSim(), -0.1f), "decrease opacity");
  bool exit_loop = false;
  scene.addVoidKeyCallback('q',boost::bind(toggle, &exit_loop), "exit");

  scene.addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Page_Up,boost::bind(shift, observed_cloth, btVector3(0,0,0.01*METERS)), "move synthetic data up");
	scene.addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Page_Down,boost::bind(shift, observed_cloth, btVector3(0,0,-0.01*METERS)), "move synthetic data down");
	scene.addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Left,boost::bind(shift, observed_cloth, btVector3(0,-0.01*METERS,0)), "move synthetic data left");
	scene.addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Right,boost::bind(shift, observed_cloth, btVector3(0,0.01*METERS,0)), "move synthetic data right");
	scene.addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Up,boost::bind(shift, observed_cloth, btVector3(-0.01*METERS,0,0)), "move synthetic data farther");
	scene.addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Down,boost::bind(shift, observed_cloth, btVector3(0.01*METERS,0,0)), "move synthetic data closer");

	//boost::posix_time::ptime sim_time = boost::posix_time::microsec_clock::local_time();
	while (!exit_loop ) {
		//Update the inputs of the featureExtractors and visibilities (if they have any inputs)

		//Do iteration
		alg->updateFeatures();
		alg->expectationStep();
		alg->maximizationStep(applyEvidence);

		trakingVisualizer->update();
		scene.step(0.03);
		//scene.step(.03,2,.015);

		//cout << (boost::posix_time::microsec_clock::local_time() - sim_time).total_milliseconds() << endl;
		//sim_time = boost::posix_time::microsec_clock::local_time();
	}

	return 0;
}
