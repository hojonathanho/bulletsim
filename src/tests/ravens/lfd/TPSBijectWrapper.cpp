/** Wrapper for calling John's learning from demonstrations python functions using boost python.*/
#include "TPSBijectWrapper.hpp"
#include <utils/colorize.h>
#include <boost/foreach.hpp>
#include "ravens_config.h"

using namespace std;

PlotLines::Ptr gbLinesLeft1(new PlotLines()), gbLinesLeft2(new PlotLines()), gbLinesRight1(new PlotLines()), gbLinesRight2(new PlotLines()), gbWarpedLinesLeft1(new PlotLines()), gbWarpedLinesLeft2(new PlotLines()), gbWarpedLinesRight1(new PlotLines()), gbWarpedLinesRight2(new PlotLines());
PlotPoints::Ptr gbSrcPlotPoints(new PlotPoints()), gbTargPlotPoints(new PlotPoints()), gbWarpedPlotPoints(new PlotPoints());
bool gbLinesAdded;

RegistrationBijectModule::RegistrationBijectModule(vector<vector<btVector3> > src_clouds, vector<vector<btVector3> > target_clouds,
		int n_iter,
		float bend_init, float bend_final,
		float rad_init, float rad_final,
		float rot_reg) {

	assert (("Different number of point-clouds.",src_clouds.size()==target_clouds.size()));

	tps_rpm_func = PyGlobals::joschu_lfd_registration_module.attr("tps_rpm_bij");

	py::list py_src_clouds, py_target_clouds;
	for (int i=0; i<src_clouds.size(); i+=1) {
		py::object py_src_cloud    = pointsToNumpy(src_clouds[i]);
		py::object py_target_cloud = pointsToNumpy(target_clouds[i]);
		py_src_clouds.append(py_src_cloud);
		py_target_clouds.append(py_target_cloud);
	}
	try {
		py::object none_dict = py::dict();
		py::object f_g_reg_modules = tps_rpm_func(py_src_clouds, py_target_clouds, n_iter, bend_init, bend_final, rad_init, rad_final, rot_reg);
		registration_module = f_g_reg_modules[0];
	} catch (...) {
		PyErr_Print();
	}
}

/** Transform a btVector using tps.
 *  Performs the mapping: pt in demonstration |--> pt in new setting. */
btVector3 RegistrationBijectModule::transform_point(btVector3 pt) {
	vector<btVector3> one_pt;
	one_pt.push_back(pt);
	return transform_points(one_pt)[0];
}

vector<btVector3> RegistrationBijectModule::transform_points(const vector<btVector3> &pts) {
	py::object py_pts = pointsToNumpy(pts);
	py::object transformed_py_pts = registration_module.attr("transform_points")(py_pts);
	return pointsFromNumpy(transformed_py_pts);
}

/** Transform a 4x4 btTransform using tps.
 *  Performs the mapping: pt in demonstration |--> pt in new setting. */
btTransform RegistrationBijectModule::transform_frame(btTransform &frame) {
	vector<btTransform> one_frame;
	one_frame.push_back(frame);
	return transform_frames(one_frame)[0];
}

vector<btTransform> RegistrationBijectModule::transform_frames(const vector<btTransform> &frames) {
	py::object py_tfms  = transformsToNumpy(frames);
	py::object transformed_py_frames;
	try {
		transformed_py_frames = registration_module.attr("transform_hmats")(py_tfms);
	} catch (...) {
		PyErr_Print();
	}
	return transformsFromNumpy(transformed_py_frames);
}

/** Return a vector of vector of points representing warped grid b/w mins and maxes.
 *  a vector of points defines points on a single line.
 *    - it returns three sets of lines, along the x,y,z axis. */
void RegistrationBijectModule::warped_grid3d(btVector3 mins, btVector3 maxs,
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

//##############################################################################################

/** Extract the joints indexed by INDS from IN_JOINT_VALS and store them into OUT_JOINT_VALS.*/
void RavensLFDBij::extractJoints (const vector<int> &inds,
		const vector<dReal> &in_joint_vals, vector<dReal> &out_joint_vals) {
	out_joint_vals.clear();
	out_joint_vals.reserve(inds.size());
	for(int i=0; i<inds.size(); i+=1)
		out_joint_vals.push_back(in_joint_vals[inds[i]]);
}

/** Find joint-angles in the warped space. Uses TrajOpt. **/
vector< vector<double> > RavensLFDBij::doTrajOpt(RaveRobotObject::Manipulator::Ptr manip,
		std::string finger1_name, std::string finger2_name,
		const vector<btTransform> & finger1_transforms,
		const vector<btTransform> & finger2_transforms,
		const vector< vector<dReal> > &old_joints) {
	RobotBasePtr robot     = manip->manip->GetRobot();
	EnvironmentBasePtr env = robot->GetEnv();
	int env_id             = RaveGetEnvironmentId(env);

	py::object py_env   = PyGlobals::openrave_module.attr("RaveGetEnvironment")(env_id);
	py::object py_robot = py_env.attr("GetRobot")(robot->GetName());

	py::object py_link1_name(finger1_name);
	py::object py_link2_name(finger2_name);
	py::object py_mats1   = transformsToNumpy(finger1_transforms);
	py::object py_mats2   = transformsToNumpy(finger2_transforms);

	py::object py_old_joints = jointsToNumpy(old_joints);
	py::object py_manip_name(manip->manip->GetName());
	py::object py_traj;
	try {
		py_traj = PyGlobals::iros_utils_module.attr("plan_follow_traj2")(py_robot, py_manip_name, py_link1_name, py_mats1, py_link2_name, py_mats2, py_old_joints);
	} catch(...) {
		PyErr_Print();
	}
	vector<vector<double> > new_joints = jointsFromNumpy(py_traj);
	return new_joints;
}

/** Warp the joint angles of ravens using warping and trajectory optimization.*/
bool RavensLFDBij::transformJointsTrajOpt(const vector<vector<dReal> > &joints, vector<vector<dReal> > &new_joints) {
	KinBody::LinkPtr r_finger1_link = ravens.ravens->robot->GetLink("rhandfinger1_sp");
	KinBody::LinkPtr r_finger2_link = ravens.ravens->robot->GetLink("rhandfinger2_sp");
	KinBody::LinkPtr l_finger1_link = ravens.ravens->robot->GetLink("lhandfinger1_sp");
	KinBody::LinkPtr l_finger2_link = ravens.ravens->robot->GetLink("lhandfinger2_sp");

	double tol = 0.001;  //DOWNSAMPLE
	std::pair< vector <float>, vector < vector <double> > > times_joints = adaptive_resample(joints, tol);
	vector<float> resampled_times             = times_joints.first;
	vector <vector<double> > resampled_joints = times_joints.second;
	cout << "adaptive resampling (tolerance ="<<tol<<"):\n\tbefore: "<<joints.size()<<"\n\tafter: "<<resampled_joints.size()<<endl;

	/** Do forward-kinematics and get the end-effector transform. */
	vector<btTransform> right1Transforms(resampled_joints.size());
	vector<btTransform> right2Transforms(resampled_joints.size());
	vector<btTransform> left1Transforms(resampled_joints.size());
	vector<btTransform> left2Transforms(resampled_joints.size());

	vector< vector<dReal> > larm_joints, rarm_joints;
	for (int i =0; i< resampled_joints.size(); i+=1) {
		vector<dReal> r_joints;
		extractJoints(rarm_indices, resampled_joints[i], r_joints);
		rarm_joints.push_back(r_joints);

		vector<dReal> l_joints;
		extractJoints(larm_indices, resampled_joints[i], l_joints);
		larm_joints.push_back(l_joints);

		/** work with palm links. */
		right1Transforms[i]  = util::scaleTransform(ravens.manipR->getFK(r_joints, r_finger1_link), 1.f/METERS);
		right2Transforms[i]  = util::scaleTransform(ravens.manipR->getFK(r_joints, r_finger2_link), 1.f/METERS);
		left1Transforms[i]   = util::scaleTransform(ravens.manipL->getFK(l_joints, l_finger1_link), 1.f/METERS);
		left2Transforms[i]   = util::scaleTransform(ravens.manipL->getFK(l_joints, l_finger2_link), 1.f/METERS);
	}

	/** Warp the end-effector transforms. */
	vector<btTransform> warpedRight1Transforms = lfdrpm->transform_frames(right1Transforms);
	vector<btTransform> warpedLeft1Transforms  = lfdrpm->transform_frames(left1Transforms);
	vector<btTransform> warpedRight2Transforms = lfdrpm->transform_frames(right2Transforms);
	vector<btTransform> warpedLeft2Transforms  = lfdrpm->transform_frames(left2Transforms);

	plotPath(right1Transforms, gbLinesRight1, btVector3(1,0,0));
	plotPath(left1Transforms, gbLinesLeft1, btVector3(1,0,0));

	plotPath(warpedRight1Transforms, gbWarpedLinesRight1,btVector3(0,0,1));
	plotPath(warpedLeft1Transforms, gbWarpedLinesLeft1, btVector3(0,0,1));


	plotPath(right2Transforms, gbLinesRight2, btVector3(1,0,0));
	plotPath(left2Transforms, gbLinesLeft2, btVector3(1,0,0));

	plotPath(warpedRight2Transforms, gbWarpedLinesRight2,btVector3(0,0,1));
	plotPath(warpedLeft2Transforms, gbWarpedLinesLeft2, btVector3(0,0,1));


	/** Do trajectory optimization on the warped transforms. */
	vector<vector<dReal> > new_r_joints = doTrajOpt(ravens.manipR, "rhandfinger1_sp", "rhandfinger2_sp",warpedRight1Transforms, warpedRight2Transforms, rarm_joints);
	vector<vector<dReal> > new_l_joints = doTrajOpt(ravens.manipL, "lhandfinger1_sp", "lhandfinger2_sp",warpedLeft1Transforms, warpedLeft2Transforms, larm_joints);

	// upsample : interpolate
	vector<float> new_times(joints.size());
	for (int i = 0.0; i < joints.size(); ++i) new_times[i] = (float) i;
	vector<vector <dReal> > interpolated_r_joints = interpolate(new_times, new_r_joints, resampled_times);
	vector<vector <dReal> > interpolated_l_joints = interpolate(new_times, new_l_joints, resampled_times);

	/** combine the new joint values into one vector while filling in the dofs
	 * which do not correspond to the arm joints from the original input.*/
	assert(("Number of set of joint angles for the arms are different.",
			(interpolated_r_joints.size()==interpolated_l_joints.size() && interpolated_r_joints.size()==joints.size())));
	new_joints.clear();
	const int num_dofs = joints[0].size();
	for(int i=0; i< joints.size(); i+=1) {
		vector<dReal> combined_joints(num_dofs);
		for(int k =0; k < num_dofs; k+=1)
			combined_joints[k] = joints[i][k];
		for(int k =0; k < larm_indices.size(); k+=1)
			combined_joints[larm_indices[k]] = interpolated_l_joints[i][k];
		for(int k =0; k < rarm_indices.size(); k+=1)
			combined_joints[rarm_indices[k]] = interpolated_r_joints[i][k];
		new_joints.push_back(combined_joints);
	}
	return true;
}

void RavensLFDBij::plotPoints (const vector< btTransform > &transforms) {
	vector<btVector3> Ps(transforms.size());
	for (int i =0; i< Ps.size();i+=1)
		Ps[i]    = METERS*transforms[i].getOrigin();
	util::drawSpheres(Ps, Eigen::Vector3f(0,1,1), 0.1, 0.005*METERS, ravens.scene.env);
}


void RavensLFDBij::plotTransforms(const vector< btTransform > &transforms) {
	vector<btTransform> Ts(transforms.size());
	for (int i =0; i< transforms.size();i+=1) {
		Ts[i]    = util::scaleTransform(transforms[i], METERS);
		PlotAxes::Ptr plot_axes(new PlotAxes());
		ravens.scene.env->add(plot_axes);
		plot_axes->setup(Ts[i], 0.01*METERS);
	}
}

void RavensLFDBij::plotPath (const vector< btTransform > &transforms, PlotLines::Ptr plot_lines, btVector3 color) {
	if (transforms.size()) {
		vector<btVector3> pts0;
		for (int i =0; i < transforms.size()-1; i+=1) {
			pts0.push_back( METERS*transforms[i].getOrigin() );
			pts0.push_back( METERS*transforms[i+1].getOrigin());
		}
		plot_lines->clear();
		plot_lines->setPoints(pts0, vector<btVector4>(pts0.size(), btVector4(color.x(),color.y(),color.z(),1)));
	}
}

void RavensLFDBij::plot_warped_grid(btVector3 mins, btVector3 maxs, int ncoarse, int nfine) {
	vector<vector<btVector3> > xlines, ylines, zlines;
	lfdrpm->warped_grid3d(mins, maxs, ncoarse, nfine, xlines, ylines, zlines);

	// scale up for plotting
	for(int i=0; i < xlines.size(); i++) {
		for(int j=0; j < xlines[i].size(); j++) {
			xlines[i][j] = METERS*xlines[i][j];
		}
	}

	// scale up for plotting
	for(int i=0; i < ylines.size(); i++) {
		for(int j=0; j < ylines[i].size(); j++) {
			ylines[i][j] = METERS*ylines[i][j];
		}
	}

	// scale up for plotting
	for(int i=0; i < zlines.size(); i++) {
		for(int j=0; j < zlines[i].size(); j++) {
			zlines[i][j] = METERS*zlines[i][j];
		}
	}

	pxlines.reset(new PlotLinesSet);
	pxlines->setDefaultColor(0.7,0,0,0.6);
	ravens.scene.env->add(pxlines);

	pylines.reset(new PlotLinesSet);
	pylines->setDefaultColor(0,0.7,0,0.6);
	ravens.scene.env->add(pylines);

	pzlines.reset(new PlotLinesSet);
	pzlines->setDefaultColor(0,0,0.7,0.6);
	ravens.scene.env->add(pzlines);


	for(int i=0; i < xlines.size(); i++)
		pxlines->addLineSet(xlines[i]);

	for(int i=0; i < ylines.size(); i++)
		pylines->addLineSet(ylines[i]);

	for(int i=0; i < zlines.size(); i++)
		pzlines->addLineSet(zlines[i]);
}


void RavensLFDBij::clear_grid() {
	pxlines->clear();
	pylines->clear();
	pzlines->clear();

	pxlines.reset();
	pylines.reset();
	pzlines.reset();
}


/** Ravens   : the robot to transform the joints for.
 *  SRC_PTS_ : the reference point locations.
 *  TARGET_PTS_: the new point locations. */
RavensLFDBij::RavensLFDBij (Ravens &ravens_, const vector<vector<btVector3> > &src_clouds,
		const vector<vector<btVector3> > & target_clouds) :
		ravens(ravens_),
		plot_lines_left(new PlotLines),
		plot_lines_right(new PlotLines),
		lfdrpm(new RegistrationBijectModule(src_clouds, target_clouds)) {


	std::cout<<colorize("LFD RPM : Please make sure that the src and target points are scaled down by METERS.", "red", true)<<std::endl;

	larm_indices = ravens.manipL->manip->GetArmIndices();
	rarm_indices = ravens.manipR->manip->GetArmIndices();

	gbLinesAdded = not RavenConfig::plotTfm;

	if (not gbLinesAdded) {
		ravens.scene.env->add(gbLinesLeft1);
		ravens.scene.env->add(gbLinesRight1);
		ravens.scene.env->add(gbWarpedLinesLeft1);
		ravens.scene.env->add(gbWarpedLinesRight1);
		ravens.scene.env->add(gbLinesLeft2);
		ravens.scene.env->add(gbLinesRight2);
		ravens.scene.env->add(gbWarpedLinesLeft2);
		ravens.scene.env->add(gbWarpedLinesRight2);
		ravens.scene.env->add(gbSrcPlotPoints);
		ravens.scene.env->add(gbTargPlotPoints);
		ravens.scene.env->add(gbWarpedPlotPoints);
		gbLinesAdded = true;
	}

	// save clouds to file
	//save_clouds(source_clouds, target_clouds);

	vector<btVector3> srcPoints, targPoints, warpedPoints;
	vector<btVector4> srcCols, targCols, warpedCols;
	BOOST_FOREACH(const vector<btVector3>& cloud, src_clouds) {
		BOOST_FOREACH(const btVector3& pt, cloud) {
			srcPoints.push_back(pt*METERS);
			srcCols.push_back(btVector4(1,0,0,1));
		}
	}

	BOOST_FOREACH(const vector<btVector3>& cloud, target_clouds) {
		BOOST_FOREACH(const btVector3& pt, cloud) {
			targPoints.push_back(pt*METERS);
			targCols.push_back(btVector4(0,0,1,1));
		}
	}
	BOOST_FOREACH(const vector<btVector3>& cloud, src_clouds) {
		vector<btVector3> warped_cloud = lfdrpm->transform_points(cloud);
		BOOST_FOREACH(const btVector3& pt, warped_cloud) {
			warpedPoints.push_back(pt*METERS);
			warpedCols.push_back(btVector4(0,1,0,1));
		}
	}

	gbSrcPlotPoints->setPoints(srcPoints, srcCols);
	gbTargPlotPoints->setPoints(targPoints, targCols);
	gbWarpedPlotPoints->setPoints(warpedPoints, warpedCols);

	plot_warped_grid(btVector3(-0.1,-0.1,0.15), btVector3(0.1,0.1, .17), 10);

	// block for user input
	cout << colorize("Look at the point-clouds. Press any key [in simulation] to continue.", "red", true)<< endl;
	ravens.scene.userInput = false;
	while (!ravens.scene.userInput) {
		ravens.scene.viewer.frame();
	}
}

/** Warp the joint values of the ravens using SRC_PTS as the reference
 *  and TARGETR_PTS as the new points for warping.*/
bool warpRavenJointsBij(Ravens &ravens,
		const vector<vector<btVector3> > &src_clouds, const vector< vector<btVector3> > &target_clouds,
		const vector< vector<dReal> >& in_joints, vector< vector<dReal> > & out_joints) {
	RavensLFDBij lfdrpm(ravens, src_clouds, target_clouds);
	lfdrpm.clear_grid();
	return lfdrpm.transformJointsTrajOpt(in_joints, out_joints);
}
