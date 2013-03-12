/** Author: Ankush Gupta
    25th October 2012. */

#ifndef _CUSTOM_SCENE_RAVENS_
#define _CUSTOM_SCENE_RAVENS_

#include "SoftBodyGripperAction.h"
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


//class CustomScene;
#define NEEDLE_SCALE_FACTOR 0.8

class CustomScene : public Scene {
public:


	/** A class to represent the cloth in the scene.
	 * Holds the cloth object and other supporting structures. */
	/*
	class SutureCloth {
	public:
		typedef boost::shared_ptr<SutureCloth> Ptr;

		// the cloth
		BulletSoftObject::Ptr cloth;

		// node indices of the nodes on the cut
		std::vector<int> cut_nodes1, cut_nodes2;

		SutureCloth(CustomScene &scene, btScalar s1, btScalar s2, btScalar z, btVector3 center);


		/** Returns the line of maximum variance of the cut-points
		 * Performs a PCA on the points.
		 * SIDE_NUM  \in {1, 2} : if 1 : cut-points on the left.
		 *                        if 2 : cut-points on the right.
		 * The return value is a point on the line found and the direction-cosine of the line. *\/
		pair<pair<btVector3, btVector3> , pair<int, int> > fitLine(int side_num);


		/** See the DOC for fitLine.
		 *  In addition to fitting a line to the cut-points it aligns
		 *  the direction of the cut with the x-axis of the robot's (PR2) transform. *\/
		pair<pair<btVector3, btVector3> , pair<int, int> >fitLineAligned(int side_num, RaveRobotObject::Ptr robot);

		/** Returns a transform for grasping.
		 *  @param SIDE_NUM  \in {1, 2} : if 1 : transform for left-cut
		 *                                if 2 : transform for right-cut *\/
		btTransform getCutGraspTransform(int side_num, RaveRobotObject::Ptr robot, float frac=0.5);
	};*/


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

		// Is the needle currently piercing?
		bool s_piercing;
		// max distance between needle tip and point to cut at
		float s_pierce_threshold;

		// Manipulator currently grasping the needle.
		RaveRobotObject::Manipulator::Ptr s_gripperManip;
		// Is the needle being grasped?
		bool s_grasped;
		// Correction matrix of the needle
		btMatrix3x3 s_corrRot;

		SuturingNeedle (CustomScene * scene, float _rope_radius=.001, float _segment_len=0.005, int _nLinks=65);

		/** Toggle's needle piercing state. */
		void togglePiercing () {s_piercing = !s_piercing;}

		btTransform getNeedleTipTransform ();
		btTransform getNeedleHandleTransform ();

		void setGraspingTransformCallback ();
		void setConnectedRopeTransformCallback();
	};


	/** Class for cloth made using box objects and spring constraints. */
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
		 */
		BoxSutureCloth(CustomScene &scene, int n, int m, btScalar s, btVector3 center);

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
	// Two sides of box cloth cloth
	BoxCloth::Ptr cloth1, cloth2;

	// Suturing needle
	SuturingNeedle::Ptr sNeedle;

	// the table in the scene
	BoxObject::Ptr table;

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
		jPlayback.reset (new jointPlayback (*this, RavenConfig::lfdProcessing));
	}


	void callGripperAction(char lr='l') {
		osg::ref_ptr<osgGA::GUIEventAdapter> e = new osgGA::GUIEventAdapter();
		e->setKey(lr == 'l' ? 'a' : 's');
		e->setEventType(osgGA::GUIEventAdapter::KEYDOWN);
		viewer.getEventQueue()->addEvent(e);
	}

	/** Returns the coordinates of the last point directly below (-z) SOURCE_PT
	      on the cloth represented by PSB. */
	btVector3 getDownPoint(btVector3 &source_pt, boost::shared_ptr<btSoftBody> psb,
			btScalar radius=3.0);

	/** Raycasts from SOURCE to all the nodes of PSB
	      and returns a vector of the same size as the nodes of PSB
	      depicting whether that node is visible or not. */
	std::vector<btVector3> checkNodeVisibility(btVector3 camera_origin,
			boost::shared_ptr<btSoftBody> psb);

	/** Finds the distance from a node corresponding to
	      INPUT_IND on the cloth to the closest node attached to the gripper
	double getDistfromNodeToClosestAttachedNodeInGripper
	(GripperKinematicObject::Ptr gripper, int input_ind, int &closest_ind);*/

	/** Draws the axes at LEFT_GRIPPER1 and LEFT_GRIPPER2. */
	void drawAxes();

	/*Creates a square cloth with side length 2s.
	     The four coordinates of the cloth are:
	     {(s,s) (-s,s,) (-s,-s) (s,-s)}
	     Then, the center of the cloth (initially at (0,0,0)
	     is translated to CENTER.*/
	BulletSoftObject::Ptr createCloth(btScalar s1, btScalar s2, btScalar z, btVector3 center,
									  std::vector<int> &cut_nodes1, std::vector<int> &cut_nodes2,
									  bool getCutIndices=true,
				                      bool shouldCut = true,
				                      unsigned int resx = 60, unsigned int resy = 20);

	/** Returns ||(v1.x, v1.y) - (v2.x, v2.y)||. */
	btScalar inline getXYDistance(btVector3 &v1, btVector3 &v2);

	/** Move the end-effector. */
	void moveEndEffector(char dir, bool world=false, char lr='l', float step=0.005);

	/** Plots needle tip, flag for removing plots. */
	void plotNeedle(bool remove = false);
	/** Plots a few things related to grasping. */
	void plotGrasp(bool remove = false);

	vector<btVector3> getRopePoints (bool nodes);
	/** Stores the points of the rope into current recording file.
	 *  Stores either nodes or control points.
	 *  */
	void recordRopePoints (bool nodes=true);

	/** small tests to test the planners and the controller. */
	void testTrajectory();
	void testTrajectory2();
	void testTrajectory3();
	void testCircular();

	/** Small test to see if the robot can grasp the cloth.*/
	void testGrasping();
	void run();
};


/** Rotates a transform TFM by AND, around the point along
 * x-axis of the transform at a distance RAD away.*/
btTransform rotateByAngle (btTransform &tfm, const float ang, const float rad);


#endif
