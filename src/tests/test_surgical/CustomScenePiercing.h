#ifndef _CUSTOM_SCENE_
#define _CUSTOM_SCENE_

#include "PR2SoftBodyGripperAction.h"

#include "simulation/simplescene.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>

#include <omp.h>
#include <iostream>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <openrave/kinbody.h>
#include "robots/pr2.h"
#include "utils/config.h"
#include "simulation/config_viewer.h"
#include "simulation/softBodyHelpers.h"

void getContactPointsWith(btSoftBody *psb, btCollisionObject *pco, btSoftBody::tRContactArray &rcontacts);
struct ImplicitSphere;

class CustomScene : public Scene {
public:
	PR2SoftBodyGripperAction::Ptr leftAction, rightAction;
	BulletInstance::Ptr bullet2;
	OSGInstance::Ptr osg2;
	Fork::Ptr fork;
	RaveRobotObject::Ptr origRobot, tmpRobot;
	PR2Manager pr2m;

	// the cloth to be sutured
	BulletSoftObject::Ptr cloth;

	// Structure to represent the hole
	struct Hole {
		CustomScene * h_scene;
		vector <btSoftBody::Node *> h_nodes;
		btVector3 h_center, h_prev_center;
		bool h_currently_piercing, h_started_piercing, h_pierced;
		Hole (CustomScene *scene): h_scene(scene), h_pierced (false), h_center(0,0,0),
				h_currently_piercing(false), h_started_piercing(false){};
		void togglePiercing () {
			h_currently_piercing = h_pierced ? false : !h_currently_piercing;
			h_started_piercing = h_currently_piercing ? h_started_piercing : false;
			std::cout<<"Hole piercing: "<<h_currently_piercing<<std::endl;
		}
		void calculateCenter ();
		void holeCutCallback ();
	};

	// the suturing needle
	//CapsuleObject::Ptr needle;
	RaveObject::Ptr sneedle;
	float 			sneedle_radius;
	// is the needle allowed to pierce?
	bool piercing;
	// max distance between needle tip and point to cut at
	float pierce_threshold;

	// vector representing holes
	vector <Hole *> holes;

	// the table in the scene
	BoxObject::Ptr table;

	OpenRAVE::ViewerBasePtr rave_viewer;
	bool isRaveViewer;

	/** Points which are to be plotted in the scene : correspond to
	    nodes in the soft-body (cloth). */
	PlotPoints::Ptr plot_points;

	// Plotting needle
	PlotPoints::Ptr plot_needle;
	// Plotting holes
	PlotPoints::Ptr plot_holes;

	/** Axes corresponding to the location where the
     *  left grippers are. **/
	PlotAxes::Ptr cut_axes;

	CustomScene() : pr2m(*this), isRaveViewer(false), piercing(false), pierce_threshold(0.03), sneedle_radius(0.08) { }

	void createFork();
	void swapFork();

	// Toggle's needle piercing state
	void togglePiercing () {piercing = !piercing;}

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

	// Get's needle tip. Which tip depends on fwd
	btVector3 getNeedleTip ();

	// Plots needle tip, flag for removing plots
	void plotNeedle(bool remove = false);
	// Plot holes, flag for removing plots
	void plotHoles(bool remove = false);

	/* Creates a square cloth with side length 2s.
	     The four coordinates of the cloth are:
	     {(s,s) (-s,s,) (-s,-s) (s,-s)}
	     Then, the center of the cloth (initially at (0,0,0)
	     is translated to CENTER.*/
	BulletSoftObject::Ptr createCloth(btScalar s, btScalar z, btVector3 center,
				                          bool shouldCut = true,
				                          unsigned int resx = 50, unsigned int resy =50);

	/** Returns ||(v1.x, v1.y) - (v2.x, v2.y)||. */
	btScalar inline getXYDistance(btVector3 &v1, btVector3 &v2);

	//Cuts cloth at mentioned needle tip
	void cutCloth ();
	//PreStepCallbacks
	// to pierce cloth if needed
	void piercingCallBack ();
	// to compute centers of holes
	void computeHoleCentersCallBack();
	// to compute nodes near a point and add it to hole
	void findNearbyNodes (Hole * hole, btVector3 holePt);

	void run();
};
#endif
