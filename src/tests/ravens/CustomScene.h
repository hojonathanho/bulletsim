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
#include "SceneRecorder.hpp"
#include "ScenePlayer.hpp"
#include "jointPlayback.h"

#include "ravens_config.h"
#include "lfd/utils_python.h"

//class CustomScene;

enum TestSuite {
	SUTURING, ROPE_MANIP
};

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


	/** Class to represent the suturing thread. */
	class SuturingRope {
	public:
		typedef boost::shared_ptr<SuturingRope> Ptr;

		// transforms applied to links to set their initial position.
		// the rope is laid out along the x-axis. Then it is transformed by this transform.
		const btTransform initTfm;

		// pointer to the capsule-rope object
		boost::shared_ptr<CapsuleRope> capsulePtr;

		// rope links' parameters
		const float rope_radius;
		const float segment_len;
		const int numLinks;


		SuturingRope(CustomScene * scene, btTransform initLinkTfm,
				float _rope_radius=.0006, float _segment_len=0.0011, int _nLinks=90); // segment len was 0.0033

		// return the points sampled along the rope (generates a point-cloud for the rope).
		void getRopePoints (bool nodes, vector<btVector3> & ropePoints, float scale=1.0);

		// reset the link-transforms to their initial transforms.
		void resetLinkTransforms();
	};


	/* Class to represent the suturing peg [needle] + thread. */
	class SuturingPeg {

		Grab* peg_rope_grab;

		btVector3 offset;

	public:
		typedef boost::shared_ptr<SuturingPeg> Ptr;
		CustomScene &scene;

		CapsuleObject::Ptr p_peg;
		const float  p_radius, p_len;

		//pointer to the rope:
		SuturingRope::Ptr ropePtr;

		// Manipulator currently grasping the needle.
		RaveRobotObject::Manipulator::Ptr p_gripperManip;
		KinBody::LinkPtr p_finger1, p_finger2;

		// Which gripper is grasping the needle?
		bool p_grasping_finger1;
		btTransform p_grasp_tfm;
		btMatrix3x3 corrRot;

		SuturingPeg (CustomScene * _scene, RaveRobotObject::Manipulator::Ptr _p_gripperManip,
				float _p_rad=0.001, float _p_len=0.006,
				float _rope_radius=.0006, float _segment_len=0.0011, int _nLinks=200);

		void toggleFinger () {p_grasping_finger1 = !p_grasping_finger1;}

		btTransform getPegCenterTransform ();

		void getRopePoints (bool nodes, vector<btVector3> & ropePoints, float scale=1.0);

		void setFingerTransformCallback ();
		void setConnectedRopeTransformCallback();
	};


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
	// Two sides of box cloth
	BoxCloth::Ptr cloth1, cloth2;
	unsigned int bcn, bcm;
	float bcs, bch;

	// Suturing needle
	SuturingNeedle::Ptr sNeedle;

	// Suturing peg
	SuturingPeg::Ptr  sPeg;

	// suturing rope
	SuturingRope::Ptr sRope;

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

	//jointRecorder::Ptr jRecorder;
	//jointPlayback::Ptr jPlayback;
	ScenePlayer::Ptr scenePlayer;
	SceneRecorder::Ptr sceneRecorder;



	// type of scene to set-up
	TestSuite tsuite;

	CustomScene(TestSuite _tsuite=SUTURING) : ravens(*this), isRaveViewer(false), tsuite(_tsuite) {
		ikPlannerL.reset(new IKInterpolationPlanner(ravens,rave,'l'));
		ikPlannerR.reset(new IKInterpolationPlanner(ravens,rave,'r'));

		//jRecorder.reset (new jointRecorder (*this));
		scenePlayer.reset (new ScenePlayer (*this, 100., RavenConfig::enableLfd));

		// new scene recorder:
		sceneRecorder.reset(new SceneRecorder(*this));

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

	void toggleGrippers(string rl);
	void closeGrippers(string rl);
	void openGrippers(string rl);

	/** Move the end-effector. */
	void moveEndEffector(char dir, bool world=false, char lr='l', float step=0.005);


	/** Gets points along edge. */
	void getBoxPoints(vector<btVector3> & boxPoints, float scale=1.0);
	/** Gets points of holes. */
	void getBoxHoles(vector<btVector3> & boxHoles, float scale=1.0);

	/** returns a formatted string of current scene points.*/
	std::string getPointsMessage();

	/** Returns a string representing the current values of the DOFs of the robot. */
	std::string getJointsMessage();

	/** Stores the points of the rope into current recording file.
	 *  Stores either nodes or control points.
	 *  */
	void recordPoints ();

	/** Just adds the msg to the scene recording file. */
	void recordMessage(std::string msg);

	/** Actually saves the scene points into a file with name:
	 *  scene_XXXXX.txt, where XXXXX is a random number. */
	void saveScenePoints();

	// Resets positions of the things in scene
	void reset ();

	// sets up objects in the scene for suturing setting.
	void setup_suturing();

	// sets up objects in the scene for rope manipulation.
	void setup_rope_manip();

	void run();

	/** Plots needle tip, flag for removing plots. */
	void plotNeedle(bool remove = false);
	/** Plots a few things related to grasping. */
	void plotGrasp(bool remove = false);
	/** Plot point cloud. */
	void plotAllPoints(bool remove = false);
	void plotAllPoints2(vector<btVector3>& old, vector<btVector3>& newpts);
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


/** Rotates a transform TFM by ANG, around the point along
 * x-axis of the transform at a distance RAD away.*/
btTransform rotateByAngle (btTransform &tfm, const float ang, const float rad);


#endif
