/** Author: Ankush Gupta
    25th October 2012. */

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

#include "RavePlanners.h"



//class CustomScene;

class CustomScene : public Scene {
public:

	/** A class to represent the cloth in the scene.
	 * Holds the cloth object and other supporting structures. */
	class SutureCloth {
	public:
		typedef boost::shared_ptr<SutureCloth> Ptr;

		// the cloth
		BulletSoftObject::Ptr cloth;

		// node indices of the nodes on the cut
		std::vector<int> cut_nodes1, cut_nodes2;

		SutureCloth(CustomScene &scene, btScalar side_length, btScalar z, btVector3 center);


		/** Returns the line of maximum variance of the cut-points
		 * Performs a PCA on the points.
		 * SIDE_NUM  \in {1, 2} : if 1 : cut-points on the left.
		 *                        if 2 : cut-points on the right.
		 * The return value is a point on the line found and the direction-cosine of the line. */
		std::pair<btVector3, btVector3> fitLine(int side_num);

	};

	PR2SoftBodyGripperAction::Ptr leftAction, rightAction;
	BulletInstance::Ptr bullet2;
	OSGInstance::Ptr osg2;
	Fork::Ptr fork;
	RaveRobotObject::Ptr origRobot, tmpRobot;
	PR2Manager pr2m;


	// the cloth to be sutured
	SutureCloth::Ptr sCloth;

	// the table in the scene
	BoxObject::Ptr table;

	OpenRAVE::ViewerBasePtr rave_viewer;
	bool isRaveViewer;

	/** Points which are to be plotted in the scene : correspond to
	    nodes in the soft-body (cloth). */
	PlotPoints::Ptr plot_points;
	std::vector<btVector3> plotpoints;
	std::vector<btVector4> plotcolors;


	/** Axes corresponding to the location where the
     *  left grippers are. **/
	PlotAxes::Ptr plot_axes;
	PlotAxes::Ptr plot_axes1;

	CustomScene() : pr2m(*this), isRaveViewer(false) { }

	void createFork();
	void swapFork();

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
	BulletSoftObject::Ptr createCloth(btScalar s, btScalar z, btVector3 center,
									  std::vector<int> &cut_nodes1, std::vector<int> &cut_nodes2,
									  bool getCutIndices=true,
				                      bool shouldCut = true,
				                      unsigned int resx = 50, unsigned int resy =50);

	/** Returns ||(v1.x, v1.y) - (v2.x, v2.y)||. */
	btScalar inline getXYDistance(btVector3 &v1, btVector3 &v2);

	/** small tests to test the planners and the controller. */
	void testTrajectory();
	void testTrajectory2();


	void run();
};



#endif
