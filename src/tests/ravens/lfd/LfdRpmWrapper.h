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

public:

	typedef boost::shared_ptr<RegistrationModule> Ptr;

	/** tps-rpm algorithm mostly as described by chui and rangaran
	 *  @params:
	 *    src_pts           : pts in the original demonstration (meters)
	 *    target_pts        : pts int the new situation         (meters)
	 *    n_iter            : number of iterations
	 *    reg_init/reg_final: regularization on curvature; affineness vs. non-affineness
	 *    rad_init/rad_final: radius for correspondence calculation (meters) */
	RegistrationModule(vector<btVector3> &src_pts, vector<btVector3> &target_pts,
			int n_iter = 5, float reg_init = .1, float reg_final = .001,
			float rad_init = .2, float rad_final = .001, bool verbose=true) {

		const py::object tps_rpm_func = PyGlobals::lfd_registration_module.attr("tps_rpm");

		py::object srcPts    = ptsToNumpy(src_pts);
		py::object targetPts = ptsToNumpy(src_pts);

		registration_module  = tps_rpm_func(srcPts, targetPts, n_iter,
				                            reg_init, reg_final, rad_init, rad_final, verbose);
	}


    /** Transform a btVector using tps.
     *  Performs the mapping: pt in demonstration |--> pt in new setting. */
	btVector3 transform_pt(btVector3 pt) {
		vector<btVector3> one_pt;
		one_pt.push_back(pt);
		return transform_pts(one_pt)[0];
	}
	vector<btVector3> transform_pts(vector<btVector3> &pts) {
		py::object py_pts = ptsToNumpy(pts);
		py::object transformed_py_pts = registration_module.attr("transform_pts")(py_pts);
		return ptsFromNumpy(transformed_py_pts);
	}


    /** Transform a 4x4 btTransform using tps.
     *  Performs the mapping: pt in demonstration |--> pt in new setting. */
	btTransform transform_frame(btTransform &frame) {
		vector<btTransform> one_frame;
		one_frame.push_back(frame);
		return transform_frames(one_frame)[0];
	}
	vector<btTransform> transform_frames(vector<btTransform> &frames) {

		vector<btMatrix3x3> rots(frames.size());
		for(int i=0; i<frames.size(); i+=1)
			rots[i] = frames[i].getBasis();

		vector<btVector3> trans(frames.size());
		for(int i=0; i<trans.size(); i+=1)
			trans[i] = frames[i].getOrigin();

		py::object py_rots  = rotationsToNumpy(rots);
		py::object py_trans = ptsToNumpy(trans);

		py::object transformed_py_frames = registration_module.attr("transform_frames")(py_trans, py_rots);
		py::object transformed_py_trans = transformed_py_frames[0];
		py::object transformed_py_rots  = transformed_py_frames[1];

		vector<btVector3> transformed_pts    = ptsFromNumpy(transformed_py_trans);
		vector<btMatrix3x3> transformed_rots = rotationsFromNumpy(transformed_py_rots);

		assert((transformed_pts.size()==transformed_rots.size(), "Number of rotations is not equal to number of translations!"));

		vector<btTransform> out_frames(transformed_pts.size());
		for (int i=0; i<transformed_pts.size(); i+=1) {
			out_frames[i] = btTransform::getIdentity();
			out_frames[i].setOrigin(transformed_pts[i]);
			out_frames[i].setBasis(transformed_rots[i]);
		}

		return out_frames;
	}
};
