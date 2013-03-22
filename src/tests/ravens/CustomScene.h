/** Author: Ankush Gupta
    25th October 2012. */

#ifndef _CUSTOM_SCENE_RAVENS_
#define _CUSTOM_SCENE_RAVENS_

#include "RavensRigidBodyAction.h"

#include "simulation/simplescene.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>

#include <omp.h>
#include <iostream>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <openrave/kinbody.h>
#include "robots/ravens.h"
#include "robots/pr2.h"

#include "utils/config.h"
#include "simulation/config_viewer.h"
#include "simulation/softBodyHelpers.h"

#include "simulation/rope.h"
#include "robots/grabbing.h"

#include "RavenPlanners.h"
#include "BoxCloth.h"
#include "jointRecorder.h"
#include "jointPlayback.h"

#include "ravens_config.h"
#include "lfd/utils_python.h"

//class CustomScene;

class CustomScene : public Scene {
public:

	/* Class to represent the suturing needle + thread. */
	class SuturingNeedle {

		float rope_radius;
		float segment_len;
		int nLinks;
		Grab* needle_rope_grab;

	public:
		typedef boost::shared_ptr<SuturingNeedle> Ptr;
		CustomScene &scene;

		RaveObject::Ptr s_needle;
		boost::shared_ptr<CapsuleRope> ropePtr;
		const float 			s_needle_radius, s_needle_mass;
		const float 			s_end_angle;

		// Manipulator currently grasping the needle.
		RaveRobotObject::Manipulator::Ptr s_gripperManip;
		// Which gripper is grasping the needle?
		char s_grasping_gripper;
		btTransform s_grasp_tfm;

		SuturingNeedle (CustomScene * _scene, float _rope_radius=.0009, float _segment_len=0.003, int _nLinks=90);

		bool pointCloseToNeedle (btVector3 pt);

		btTransform getNeedleTipTransform ();
		btTransform getNeedleHandleTransform ();
		btTransform getNeedleCenterTransform ();

		void getNeedlePoints (vector<btVector3> & needlePoints, float scale=1.0);
		void getRopePoints (bool nodes, vector<btVector3> & ropePoints, float scale=1.0);

		void setGraspingTransformCallback ();
		void setConnectedRopeTransformCallback();
	};

	/* Class to represent the suturing needle + thread. */
	class SuturingPeg {

		Grab* peg_rope_grab;

		btVector3 offset;

	public:
		typedef boost::shared_ptr<SuturingPeg> Ptr;
		CustomScene &scene;

		CapsuleObject::Ptr p_peg;
		boost::shared_ptr<CapsuleRope> ropePtr;
		const float 			p_radius, p_len;

		const float rope_radius;
		const float segment_len;
		const int nLinks;

		// Manipulator currently grasping the needle.
		RaveRobotObject::Manipulator::Ptr p_gripperManip;
		KinBody::LinkPtr p_finger1, p_finger2;

		// Which gripper is grasping the needle?
		bool p_grasping_finger1;
		btTransform p_grasp_tfm;
		btMatrix3x3 corrRot;

		SuturingPeg (CustomScene * _scene, RaveRobotObject::Manipulator::Ptr _p_gripperManip,
					float _p_rad=0.0006, float _p_len=0.006,
					float _rope_radius=.0006, float _segment_len=0.0033, int _nLinks=90);

		void toggleFinger () {p_grasping_finger1 = !p_grasping_finger1;}

		btTransform getPegCenterTransform ();

		void getRopePoints (bool nodes, vector<btVector3> & ropePoints, float scale=1.0);

		void setFingerTransformCallback ();
		void setConnectedRopeTransformCallback();
	};


	/** Class for cloth made using box objects and spring constraints. *\/
	class BoxSutureCloth {

	public:
		typedef boost::shared_ptr<BoxSutureCloth> Ptr;


		// node indices of the nodes on the cut
		std::vector<int> cut_nodes1, cut_nodes2;

		/** @params:
		 *  N : the number of box-objects along the x-dimension.
		 *  M : the number of box-objects along the y-dimension.
		 *  S : the length of side of each box object.
		 *  H : the height of the box object : should be small to represent a plate.
		 *  CENTER : location of the center of the cloth.
		 *\/
		BoxSutureCloth(CustomScene &scene, int n, int m, btScalar s, btVector3 center);

	};*/

	//SoftBodyGripperAction::Ptr leftAction, rightAction;

	RavensRigidBodyGripperAction::Ptr lAction, rAction;  //>>>>>>>>>>>>> testing

	BulletInstance::Ptr bullet2;
	OSGInstance::Ptr osg2;
	Fork::Ptr fork;
	RaveRobotObject::Ptr origRobot, tmpRobot;
	Ravens ravens;
	IKInterpolationPlanner::Ptr ikPlannerL, ikPlannerR;



	/*
	// the cloth to be sutured
	SutureCloth::Ptr sCloth;
	*/
	// Two sides of box cloth cloth
	BoxCloth::Ptr cloth1, cloth2;
	unsigned int bcn, bcm;
	float bcs, bch;

	// Suturing needle
	SuturingNeedle::Ptr sNeedle;

	// Suturing peg
	SuturingPeg::Ptr sPeg;

	// the table in the scene
	BoxObject::Ptr table;
	// supports
	BoxObject::Ptr support1, support2;

	OpenRAVE::ViewerBasePtr rave_viewer;
	bool isRaveViewer;

	/** Points which are to be plotted in the scene : correspond to
	    nodes in the soft-body (cloth). */
	PlotPoints::Ptr plot_points;
	std::vector<btVector3> plotpoints;
	std::vector<btVector4> plotcolors;
	// Points for plotting center and tip of needle
	PlotPoints::Ptr plot_needle;

	/** Axes corresponding to the location where the
     *  left grippers are. **/
	PlotAxes::Ptr plot_axes1;
	PlotAxes::Ptr plot_axes2;

	jointRecorder::Ptr jRecorder;
	jointPlayback::Ptr jPlayback;

	CustomScene() : ravens(*this), isRaveViewer(false) {
		ikPlannerL.reset(new IKInterpolationPlanner(ravens,rave,'l'));
		ikPlannerR.reset(new IKInterpolationPlanner(ravens,rave,'r'));

		jRecorder.reset (new jointRecorder (*this));
		jPlayback.reset (new jointPlayback (*this, RavenConfig::enableLfd));

		bcn = RavenConfig::bcN; bcm = RavenConfig::bcM;
		bcs = RavenConfig::bcS; bch = RavenConfig::bcH;
	}


	void callGripperAction(char lr='l') {
		osg::ref_ptr<osgGA::GUIEventAdapter> e = new osgGA::GUIEventAdapter();
		e->setKey(lr == 'l' ? 'a' : 's');
		e->setEventType(osgGA::GUIEventAdapter::KEYDOWN);
		viewer.getEventQueue()->addEvent(e);
	}

	void togglePegFinger () {sPeg->toggleFinger();}

	/** Move the end-effector. */
	void moveEndEffector(char dir, bool world=false, char lr='l', float step=0.005);


	/** Gets points along edge. */
	void getBoxPoints(vector<btVector3> & boxPoints, float scale=1.0);
	/** Gets points of holes. */
	void getBoxHoles(vector<btVector3> & boxHoles, float scale=1.0);
	/** Stores the points of the rope into current recording file.
	 *  Stores either nodes or control points.
	 *  */
	void recordPoints ();

	// Resets positions of the things in scene
	void reset ();

	void run();

	/** Plots needle tip, flag for removing plots. */
	void plotNeedle(bool remove = false);
	/** Plots a few things related to grasping. */
	void plotGrasp(bool remove = false);
	/** Plot point cloud. */
	void plotAllPoints(bool remove = false);
	/** Plot tfm of hole. */
	void plotHoleTfm ();
	/** Plot tfm of peg.*/
	void plotPeg ();

	/** small tests to test the planners and the controller. */
	void testTrajectory();
	void testTrajectory2();
	void testTrajectory3();
	void testNeedle2();
	void testNeedle();
	void testGrab();
	void testGrasping();
};


/** Rotates a transform TFM by AND, around the point along
 * x-axis of the transform at a distance RAD away.*/
btTransform rotateByAngle (btTransform &tfm, const float ang, const float rad);


#endif
