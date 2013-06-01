/** Wrapper for calling John's learning from demonstrations python functions using boost python.*/

#include "LfdRpmWrapper.h"

/** Wraps around lfd rpm module. */
RegistrationModule::RegistrationModule(vector<btVector3> src_pts, vector<btVector3> target_pts,
		int n_iter, float reg_init, float reg_final,
		float rad_init, float rad_final) {

	tps_rpm_func = PyGlobals::lfd_registration_module.attr("tps_rpm");

	py::object py_src_pts    = pointsToNumpy(src_pts);
	py::object py_target_pts = pointsToNumpy(target_pts);

	try {
		registration_module  = tps_rpm_func(py_src_pts, py_target_pts, n_iter,
				reg_init, reg_final, rad_init, rad_final);
	} catch (...) {
		PyErr_Print();
	}
}

/** Wraps around lfd rpm module. */
RegistrationModule::RegistrationModule(vector <vector<btVector3> > src_clouds,
		vector <vector<btVector3> > target_clouds,
		int n_iter, float reg_init, float reg_final,
		float rad_init, float rad_final) {

	assert (("Different number of point-clouds.",src_clouds.size()==target_clouds.size()));
	tps_rpm_func = PyGlobals::lfd_registration_module.attr("tps_rpm_multi");

	py::list py_src_clouds, py_target_clouds;
	for (int i=0; i<src_clouds.size(); i+=1) {
		py::object py_src_cloud    = pointsToNumpy(src_clouds[i]);
		py::object py_target_cloud = pointsToNumpy(target_clouds[i]);
		py_src_clouds.append(py_src_cloud);
		py_target_clouds.append(py_target_cloud);
	}
	try {
		registration_module  = tps_rpm_func(py_src_clouds, py_target_clouds, n_iter,
				reg_init, reg_final, rad_init, rad_final);
	} catch (...) {
		PyErr_Print();
	}

	//	tps_rpm_func = PyGlobals::lfd_registration_module.attr("tps_rpm");
	//
	//	py::object py_src_pts    = pointsToNumpy(src_clouds[0]);
	//	py::object py_target_pts = pointsToNumpy(target_clouds[0]);
	//
	//	try {
	//		registration_module  = tps_rpm_func(py_src_pts, py_target_pts, n_iter,
	//				reg_init, reg_final, rad_init, rad_final);
	//	} catch (...) {
	//		PyErr_Print();
	//	}

}


/** Transform a btVector using tps.
 *  Performs the mapping: pt in demonstration |--> pt in new setting. */
btVector3 RegistrationModule::transform_point(btVector3 pt) {
	vector<btVector3> one_pt;
	one_pt.push_back(pt);
	return transform_points(one_pt)[0];
}

vector<btVector3> RegistrationModule::transform_points(const vector<btVector3> &pts) {
	py::object py_pts = pointsToNumpy(pts);
	py::object transformed_py_pts = registration_module.attr("transform_points")(py_pts);
	return pointsFromNumpy(transformed_py_pts);
}




/** Transform a 4x4 btTransform using tps.
 *  Performs the mapping: pt in demonstration |--> pt in new setting. */
btTransform RegistrationModule::transform_frame(btTransform &frame) {
	vector<btTransform> one_frame;
	one_frame.push_back(frame);
	return transform_frames(one_frame)[0];
}

vector<btTransform> RegistrationModule::transform_frames(const vector<btTransform> &frames) {

	vector<btMatrix3x3> rots(frames.size());
	for(int i=0; i<frames.size(); i+=1)
		rots[i] = frames[i].getBasis();

	vector<btVector3> trans(frames.size());
	for(int i=0; i<trans.size(); i+=1)
		trans[i] = frames[i].getOrigin();

	py::object py_rots  = rotationsToNumpy(rots);
	py::object py_trans = pointsToNumpy(trans);

	py::object transformed_py_frames = registration_module.attr("transform_frames")(py_trans, py_rots);
	py::object transformed_py_trans = transformed_py_frames[0];
	py::object transformed_py_rots  = transformed_py_frames[1];

	vector<btVector3> transformed_pts    = pointsFromNumpy(transformed_py_trans);
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


/** return a vector of vector of points representing warped grid b/w mins and maxes.
 *  a vector of points defines points on a single line.
 *    - it returns three sets of lines, along the x,y,z axis. */
void RegistrationModule::warped_grid3d(btVector3 mins, btVector3 maxs,
		int ncoarse, int nfine,
		std::vector<vector<btVector3> > &xlines,
		std::vector<vector<btVector3> > &ylines,
		std::vector<vector<btVector3> > &zlines) {

	vector<float> xcoarse, xfine;
	util::linspace(mins.x(), maxs.x(), ncoarse, xcoarse);
	util::linspace(mins.x(), maxs.x(), nfine, xfine);

	vector<float> ycoarse, yfine;
	util::linspace(mins.y(), maxs.y(), ncoarse, ycoarse);
	util::linspace(mins.y(), maxs.y(), nfine, yfine);

	vector<float> zcoarse, zfine;
	util::linspace(mins.z(), maxs.z(), ncoarse, zcoarse);
	util::linspace(mins.z(), maxs.z(), nfine, zfine);

	xlines.clear();
	ylines.clear();
	zlines.clear();

	for(int iz=0; iz< ncoarse; iz++) {

		// generate x-lines
		for (int ix = 0; ix < ncoarse; ix++) {
			vector<btVector3> xline(nfine);
			for(int iy = 0; iy < nfine; iy++)
				xline[iy] = transform_point(btVector3(xcoarse[ix], yfine[iy], zcoarse[iz]));
			xlines.push_back(xline);
		}

		// generate y-lines
		for (int iy = 0; iy < ncoarse; iy++) {
			vector<btVector3> yline(nfine);
			for(int ix = 0; ix < nfine; ix++)
				yline[ix] = transform_point(btVector3(xfine[ix], ycoarse[iy], zcoarse[iz]));
			ylines.push_back(yline);
		}
	}

	//generate z-lines
	for(int ix=0 ; ix < ncoarse; ix++) {
		for(int iy= 0; iy < ncoarse; iy++) {
			vector<btVector3> zline(nfine);
			for(int iz = 0; iz < nfine; iz++) {
				zline[iz] = transform_point(btVector3(xcoarse[ix], ycoarse[iy], zfine[iz]));
			}
			zlines.push_back(zline);
		}
	}
}
