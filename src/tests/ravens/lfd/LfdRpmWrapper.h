/** Wrapper for calling John's learning from demonstrations python functions using boost python.*/

#pragma once

#include <boost/python.hpp>
#include <boost/shared_ptr.hpp>
#include "simulation/util.h"
#include "utils_python.h"


using namespace std;
namespace py = boost::python;


/** Wraps around lfd rpm module. */
class RegistrationModule {
	py::object registration_module;
	py::object tps_rpm_func;

public:

	typedef boost::shared_ptr<RegistrationModule> Ptr;

	/** tps-rpm algorithm mostly as described by chui and rangaran
	 *  @params:
	 *    src_pts           : pts in the original demonstration (meters)
	 *    target_pts        : pts int the new situation         (meters)
	 *    n_iter            : number of iterations
	 *    reg_init/reg_final: regularization on curvature; affineness vs. non-affineness
	 *    rad_init/rad_final: radius for correspondence calculation (meters) */
	RegistrationModule(vector <vector<btVector3> > src_clouds,
			vector <vector<btVector3> > target_clouds,
			int n_iter=100,
			float bend_init=0.05, float bend_final=0.0001,
			Eigen::Vector3f rot_init=Eigen::Vector3f(0.1,0.1, 0.025),
			Eigen::Vector3f rot_final=Eigen::Vector3f(0.001, 0.001, 0.00025),
			float scale_init=1, float scale_final=0.001,
			float rad_init=0.5, float rad_final=0.0005);


	/** tps-rpm algorithm mostly as described by chui and rangaran
	 *  @params:
	 *    src_pts           : point-clouds in the original demonstration (meters)
	 *    target_pts        : point-clouds in the new situation          (meters)
	 *    n_iter            : number of iterations
	 *    reg_init/reg_final: regularization on curvature; affineness vs. non-affineness
	 *    rad_init/rad_final: radius for correspondence calculation (meters) */
	RegistrationModule(vector<btVector3> src_pts, vector<btVector3> target_pts,
			int n_iter = 100, float bend_init = .005, float bend_final = .0001,
			Eigen::Vector3f rot_init=Eigen::Vector3f(0.1,0.1, 0.025),
			Eigen::Vector3f rot_final=Eigen::Vector3f(0.001, 0.001, 0.00025),
			float scale_init=1, float scale_final=0.001,
			float rad_init = .1, float rad_final = .0005);

	/** Transform a btVector using tps.
	 *  Performs the mapping: pt in demonstration |--> pt in new setting. */
	btVector3 transform_point(btVector3 pt);
	vector<btVector3>  transform_points(const vector<btVector3> &pts);


	/** Transform a 4x4 btTransform using tps.
	 *  Performs the mapping: pt in demonstration |--> pt in new setting. */
	btTransform transform_frame(btTransform &frame);
	vector<btTransform> transform_frames(const vector<btTransform> &frames);

	/** return a vector of vector of points representing warped grid b/w mins and maxes.
	 *  a vector of points defines points on a single line.
	 *    - it returns three sets of lines, along the x,y,z axis. */
	void warped_grid3d(btVector3 mins, btVector3 maxs,
			int ncoarse, int nfine,
			std::vector<vector<btVector3> > &xlines,
			std::vector<vector<btVector3> > &ylines,
			std::vector<vector<btVector3> > &zlines);
};
