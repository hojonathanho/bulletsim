#pragma once

#include <boost/python.hpp>
#include <boost/shared_ptr.hpp>
#include "simulation/util.h"
#include "utils_python.h"
#include <vector>
#include <iostream>
#include "robots/ravens.h"
#include "simulation/plotting.h"


namespace py = boost::python;

/** Wraps around lfd rpm module. */
class RegistrationBijectModule {
	py::object f_g_reg_modules;
	py::object registration_module;
	py::object tps_rpm_func;

public:

	typedef boost::shared_ptr<RegistrationBijectModule> Ptr;

	/** tps-rpm algorithm mostly as described by chui and rangaran
	 *  @params:
	 *    src_pts           : pts in the original demonstration (meters)
	 *    target_pts        : pts int the new situation         (meters)
	 *    n_iter            : number of iterations
	 *    reg_init/reg_final: regularization on curvature; affineness vs. non-affineness
	 *    rad_init/rad_final: radius for correspondence calculation (meters) */
	RegistrationBijectModule(std::vector <std::vector<btVector3> > src_clouds,
			std::vector <std::vector<btVector3> > target_clouds,
			int n_iter=50,
			float bend_init=0.1, float bend_final=0.00001,
			float rad_init=0.5, float rad_final=0.0001,
			float rot_reg=1, float corr_reg=0.5, float outliersd=3);

	// returns the cost of warping : <f._cost, g._cost>
	std::pair<double, double> getWarpingCosts();

	/** Transform a btVector using tps.
	 *  Performs the mapping: pt in demonstration |--> pt in new setting. */
	btVector3 transform_point(btVector3 pt);
	std::vector<btVector3>  transform_points(const std::vector<btVector3> &pts);

	/** Transform a 4x4 btTransform using tps.
	 *  Performs the mapping: pt in demonstration |--> pt in new setting. */
	btTransform transform_frame(btTransform &frame);
	std::vector<btTransform> transform_frames(const std::vector<btTransform> &frames);

	/** return a vector of vector of points representing warped grid b/w mins and maxes.
	 *  a vector of points defines points on a single line.
	 *    - it returns three sets of lines, along the x,y,z axis. */
	void warped_grid3d(btVector3 mins, btVector3 maxs,
			int ncoarse, int nfine,
			std::vector<std::vector<btVector3> > &xlines,
			std::vector<std::vector<btVector3> > &ylines,
			std::vector<std::vector<btVector3> > &zlines);
};


/** Simple class which helps apply lfd by transforming the joints angles. */
class RavensLFDBij {
private:

	PlotLines::Ptr plot_lines_left, plot_lines_right;
	PlotLinesSet::Ptr pxlines, pylines, pzlines;

	/** Extract the joints indexed by INDS from IN_JOINT_VALS and store them into OUT_JOINT_VALS.*/
	void extractJoints (const vector<int> &inds, const vector<dReal> &in_joint_vals,
			vector<dReal> &out_joint_vals);

	Ravens  &ravens;
	RegistrationBijectModule::Ptr lfdrpm;

	vector<int> larm_indices;
	vector<int> rarm_indices;

	/** Find joint-angles in the warped space. **/
	vector< vector<double> > doTrajOpt(RaveRobotObject::Manipulator::Ptr manip,
			std::string link1_name, std::string link2_name,
			const vector<btTransform> & finger1_transforms,
			const vector<btTransform> & finger2_transforms,
			const vector< vector<dReal> > &old_joints);

	/** Plotting util functions. */
	void plotTransforms (const vector< btTransform > &transforms);
	void plotPoints      (const vector< btTransform > &transforms);
	void plotPath        (const vector< btTransform > &transforms, PlotLines::Ptr plot_lines, btVector3 color = btVector3(1,1,1));

public:
	/** Ravens   : the robot to transform the joints for.
	 *  SRC_PTS_ : the reference point locations.
	 *  TARGET_PTS_: the new point locations. */
	RavensLFDBij (Ravens &ravens_, const vector<vector<btVector3> > &src_clouds,
			       const vector<vector<btVector3> > & target_pts);

	/** Warp the joint angles of ravens using warping and trajectory optimization.*/
	bool transformJointsTrajOpt(const vector<vector<dReal> > &joints, vector<vector<dReal> > &new_joints);

	/** plots a grid representing the warping.*/
	void plot_warped_grid(btVector3 mins, btVector3 maxs, int ncoarse=10, int nfine=30);
	void clear_grid();
};


/** Warp the joint values of the ravens using SRC_PTS as the reference
 *  and TARGETR_PTS as the new points for warping.*/
bool warpRavenJointsBij(Ravens &ravens,
		const vector<vector<btVector3> > &src_pts, const vector< vector<btVector3> > &target_pts,
		const vector< vector<dReal> >& in_joints, vector< vector<dReal> > & out_joints);

/** Returns the warping objective cost based on tps_rpm_bij. */
double getWarpingDistance(const vector<vector<btVector3> > &src_clouds, const vector<vector<btVector3> > &target_clouds);
