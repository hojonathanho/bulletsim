#include "RavensLfd.h"
#include "ravens_config.h"
#include <boost/foreach.hpp>
#include <utils/colorize.h>
#include "CustomScene.h"


using namespace std;

bool gLinesAdded;


RavensLfdRpm::RavensLfdRpm (Ravens & ravens_, const vector<btVector3> &src_pts,
		const vector<btVector3> &target_pts) : ravens(ravens_), plot_lines_left(new PlotLines), plot_lines_right(new PlotLines),
		lfdrpm(new RegistrationModule(target_pts, src_pts, 100, 0.1, 0.0001, 0.2, 0.001)){

	gLinesAdded = !RavenConfig::plotTfm;

	if (src_pts.size() < 50 || target_pts.size() < 50)
		cout <<"LFD RPM : Warning too few points!"<<endl;

	std::cout<<"LFD RPM : Please make sure that the src and target points are scaled down by METERS."<<std::endl;

	larm_indices = ravens.manipL->manip->GetArmIndices();
	rarm_indices = ravens.manipR->manip->GetArmIndices();

	ravens.scene.env->add(plot_lines_left);
	ravens.scene.env->add(plot_lines_right);
}

PlotLines::Ptr gLinesLeft1(new PlotLines()), gLinesLeft2(new PlotLines()), gLinesRight1(new PlotLines()), gLinesRight2(new PlotLines()), gWarpedLinesLeft1(new PlotLines()), gWarpedLinesLeft2(new PlotLines()), gWarpedLinesRight1(new PlotLines()), gWarpedLinesRight2(new PlotLines());
PlotPoints::Ptr gSrcPlotPoints(new PlotPoints()), gTargPlotPoints(new PlotPoints()), gWarpedPlotPoints(new PlotPoints());

RavensLfdRpm::RavensLfdRpm (Ravens & ravens_, const vector<vector<btVector3> > &source_clouds,
		const vector<vector<btVector3> > &target_clouds) : ravens(ravens_), plot_lines_left(new PlotLines), plot_lines_right(new PlotLines),
		lfdrpm(new RegistrationModule(source_clouds, target_clouds, 100, 100, 0.0001, 0.9, 0.001)){

	gLinesAdded = !RavenConfig::plotTfm;

	std::cout<<"LFD RPM : Please make sure that the src and target points are scaled down by METERS."<<std::endl;

	larm_indices = ravens.manipL->manip->GetArmIndices();
	rarm_indices = ravens.manipR->manip->GetArmIndices();

	if (!gLinesAdded) {
		ravens.scene.env->add(gLinesLeft1);
		ravens.scene.env->add(gLinesRight1);
		ravens.scene.env->add(gWarpedLinesLeft1);
		ravens.scene.env->add(gWarpedLinesRight1);
		ravens.scene.env->add(gLinesLeft2);
		ravens.scene.env->add(gLinesRight2);
		ravens.scene.env->add(gWarpedLinesLeft2);
		ravens.scene.env->add(gWarpedLinesRight2);
		ravens.scene.env->add(gSrcPlotPoints);
		ravens.scene.env->add(gTargPlotPoints);
		ravens.scene.env->add(gWarpedPlotPoints);
		gLinesAdded = true;
	}

	vector<btVector3> srcPoints, targPoints, warpedPoints;
	vector<btVector4> srcCols, targCols, warpedCols;
	BOOST_FOREACH(const vector<btVector3>& cloud, source_clouds) {
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
	BOOST_FOREACH(const vector<btVector3>& cloud, source_clouds) {
		vector<btVector3> warped_cloud = lfdrpm->transform_points(cloud);
		BOOST_FOREACH(const btVector3& pt, warped_cloud) {
			warpedPoints.push_back(pt*METERS);
			warpedCols.push_back(btVector4(0,1,0,1));
		}
	}

	gSrcPlotPoints->setPoints(srcPoints, srcCols);
	gTargPlotPoints->setPoints(targPoints, targCols);
	gWarpedPlotPoints->setPoints(warpedPoints, warpedCols);

	plot_warped_grid(btVector3(-0.1,-0.1,0.15), btVector3(0.1,0.1, .17), 10);

	// block for user input
	cout << colorize("Look at the point-clouds. Press any key [in simulation] to continue.", "red", true)<< endl;
	ravens.scene.userInput = false;
	while (!ravens.scene.userInput) {
		ravens.scene.viewer.frame();
	}
}

void RavensLfdRpm::plot_warped_grid(btVector3 mins, btVector3 maxs, int ncoarse, int nfine) {
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


void RavensLfdRpm::clear_grid() {
	pxlines->clear();
	pylines->clear();
	pzlines->clear();

	pxlines.reset();
	pylines.reset();
	pzlines.reset();
}

/** Does smooth IK on transforms (in joint space: basically chooses the closest subsequent joint-set [l2 normwise].
 *  Ik is done for each transform in TRANSFORMS and the corresponding joints are stored in JOINTS.*/
bool RavensLfdRpm::doSmoothIK(RaveRobotObject::Manipulator::Ptr manip, const vector<btTransform> & transforms,
		vector< vector<dReal> > &joints)  {
	joints.clear();
	vector<dReal> currentDOFs = manip->getDOFValues();

	for(int i = 0; i < transforms.size(); i+=1) {
		vector <vector<dReal> > values;
		if (manip->solveAllIKUnscaled(util::toRaveTransform(transforms[i]), values)) {
			int solSize = values.size();

			vector<double> * bestDOFs (new vector<double>());
			*bestDOFs = values[0];

			double bestL2 = util::wrapAroundL2(*bestDOFs, currentDOFs);

			for (int j = 1; j < solSize; ++j) {
				double newL2 = util::wrapAroundL2(values[j],currentDOFs);
				if (newL2 < bestL2) {
					*bestDOFs = values[j];
					bestL2 = newL2;
				}
			}
			joints.push_back(*bestDOFs);
			currentDOFs = *bestDOFs;
		} else {//failure
			RAVELOG_INFO("IK failed on warped transforms.\n");
			return false;
		}
	}
	//unwrapWayPointDOFs(curerntDOFs);

	return true;
}


/** Does smooth IK on transforms (in joint space: basically chooses the closest subsequent joint-set [l2 normwise].
 *  Ik is done for each transform in TRANSFORMS and the corresponding joints are stored in JOINTS.*/
bool doSmoothIK2(RaveRobotObject::Manipulator::Ptr manip, const vector<btTransform> & transforms,
		vector< vector<dReal> > &joints) {
	joints.clear();
	for(int i = 0; i < transforms.size(); i+=1) {
		vector<dReal> values;
		if (manip->solveIKUnscaled(util::toRaveTransform(transforms[i]), values)) {
			joints.push_back(values);
		} else {//failure
			RAVELOG_INFO("IK failed on warped transforms.\n");
			return false;
		}
	}
	return true;
}


bool RavensLfdRpm::transformJoints(const vector<vector<dReal> > &joints, vector<vector<dReal> > &new_joints) {

	/** Do forward-kinematics and get the end-effector transform. */
	vector<btTransform> rightEETransforms(joints.size());
	vector<btTransform> leftEETransforms(joints.size());

	for (int i =0; i< joints.size(); i+=1) {
		vector<dReal> r_joints;
		extractJoints(rarm_indices, joints[i], r_joints);

		vector<dReal> l_joints;
		extractJoints(larm_indices, joints[i], l_joints);

		/** work with end-effector transforms. */
		rightEETransforms[i]  = util::scaleTransform(ravens.manipR->getFK(r_joints), 1.f/METERS);
		leftEETransforms[i]   = util::scaleTransform(ravens.manipL->getFK(l_joints), 1.f/METERS);
	}

	/** Warp the end-effector transforms. */
	vector<btTransform> warpedRightEETransforms = lfdrpm->transform_frames(rightEETransforms);
	vector<btTransform> warpedLeftEETransforms  = lfdrpm->transform_frames(leftEETransforms);

	/** Do IK on the warped transforms. */
	vector<vector<dReal> > new_r_joints;
	bool r_success = doSmoothIK(ravens.manipR, warpedRightEETransforms, new_r_joints);

	vector<vector<dReal> > new_l_joints;
	bool l_success = doSmoothIK(ravens.manipL, warpedLeftEETransforms, new_l_joints);

	if (r_success && l_success) {
		/** combine the new joint values into one vector while filling in the dofs
		 * which do not correspond to the arm joints from the original input.*/
		assert(("Number of set of joint angles for the arms are different.",
				(new_r_joints.size()==new_l_joints.size() && new_r_joints.size()==joints.size())));
		new_joints.clear();
		const int num_dofs = joints[0].size();
		for(int i=0; i< joints.size(); i+=1) {
			vector<dReal> combined_joints(num_dofs);
			for(int k =0; k < num_dofs; k+=1)
				combined_joints[k] = joints[i][k];
			for(int k =0; k < larm_indices.size(); k+=1)
				combined_joints[larm_indices[k]] = new_l_joints[i][k];
			for(int k =0; k < rarm_indices.size(); k+=1)
				combined_joints[rarm_indices[k]] = new_r_joints[i][k];
			new_joints.push_back(combined_joints);
		}

		return true;
	} else {
		return false;
	}
}


bool RavensLfdRpm::transformJointsTrajOpt(const vector<vector<dReal> > &joints, vector<vector<dReal> > &new_joints) {

	KinBody::LinkPtr r_finger1_link = ravens.ravens->robot->GetLink("rhandfinger1_sp");
	KinBody::LinkPtr r_finger2_link = ravens.ravens->robot->GetLink("rhandfinger2_sp");
	KinBody::LinkPtr l_finger1_link = ravens.ravens->robot->GetLink("lhandfinger1_sp");
	KinBody::LinkPtr l_finger2_link = ravens.ravens->robot->GetLink("lhandfinger2_sp");

	double tol = 0.02;  //DOWNSAMPLE
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


	plotPath(right1Transforms, gLinesRight1, btVector3(1,0,0));
	plotPath(left1Transforms, gLinesLeft1, btVector3(1,0,0));

	plotPath(warpedRight1Transforms, gWarpedLinesRight1,btVector3(0,0,1));
	plotPath(warpedLeft1Transforms, gWarpedLinesLeft1, btVector3(0,0,1));


	plotPath(right2Transforms, gLinesRight2, btVector3(1,0,0));
	plotPath(left2Transforms, gLinesLeft2, btVector3(1,0,0));

	plotPath(warpedRight2Transforms, gWarpedLinesRight2,btVector3(0,0,1));
	plotPath(warpedLeft2Transforms, gWarpedLinesLeft2, btVector3(0,0,1));



	/** Do trajectory optimization on the warped transforms. */
	vector<vector<dReal> > new_r_joints =	 doTrajectoryOptimization2(ravens.manipR, "rhandfinger1_sp", "rhandfinger2_sp",warpedRight1Transforms, warpedRight2Transforms, rarm_joints);
	vector<vector<dReal> > new_l_joints =	 doTrajectoryOptimization2(ravens.manipL, "lhandfinger1_sp", "lhandfinger2_sp",warpedLeft1Transforms, warpedLeft2Transforms, larm_joints);

	cout<<"Warped 1: "<<warpedRight1Transforms.size()<<endl;
	cout<<"Warped 2: "<<warpedLeft1Transforms.size()<<endl;

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


void RavensLfdRpm::plotPoints (const vector< btTransform > &transforms) {
	vector<btVector3> Ps(transforms.size());
	for (int i =0; i< Ps.size();i+=1)
		Ps[i]    = METERS*transforms[i].getOrigin();
	util::drawSpheres(Ps, Eigen::Vector3f(0,1,1), 0.1, 0.005*METERS, ravens.scene.env);
}


void RavensLfdRpm::plotTransforms(const vector< btTransform > &transforms) {
	vector<btTransform> Ts(transforms.size());
	for (int i =0; i< transforms.size();i+=1) {
		Ts[i]    = util::scaleTransform(transforms[i], METERS);
		PlotAxes::Ptr plot_axes(new PlotAxes());
		ravens.scene.env->add(plot_axes);
		plot_axes->setup(Ts[i], 0.01*METERS);
	}
}

void RavensLfdRpm::plotPath (const vector< btTransform > &transforms, PlotLines::Ptr plot_lines, btVector3 color) {
	if (transforms.size()) {
		vector<btVector3> pts0;
		for (int i =0; i < transforms.size()-1; i+=1) {
			pts0.push_back( METERS*transforms[i].getOrigin() );
			pts0.push_back( METERS*transforms[i+1].getOrigin());
		}
		plot_lines->clear();
		//		float r = ((float)rand())/RAND_MAX, g=((float)rand())/RAND_MAX, b=((float)rand())/RAND_MAX;
		plot_lines->setPoints(pts0, vector<btVector4>(pts0.size(), btVector4(color.x(),color.y(),color.z(),1)));
	}
}

/** Extract the joints indexed by INDS from IN_JOINT_VALS and store them into OUT_JOINT.*/
void RavensLfdRpm::extractJoints (const vector<int> &inds, const vector<dReal> &in_joint_vals, vector<dReal> &out_joint_vals) {
	out_joint_vals.clear();
	out_joint_vals.reserve(inds.size());
	for(int i=0; i<inds.size(); i+=1)
		out_joint_vals.push_back(in_joint_vals[inds[i]]);
}


/** Warp the joint values of the ravens using SRC_PTS as the reference
 *  and TARGETR_PTS as the new points for warping.*/
bool warpRavenJoints(Ravens &ravens,
		const vector<btVector3> &src_pts, const vector<btVector3> &target_pts,
		const vector< vector<dReal> >& in_joints, vector< vector<dReal> > & out_joints) {
	RavensLfdRpm lfdrpm(ravens, src_pts, target_pts);
	// return lfdrpm.transformJoints(in_joints, out_joints);
	return lfdrpm.transformJointsTrajOpt(in_joints, out_joints);
}


/** Warp the joint values of the ravens using SRC_PTS as the reference
 *  and TARGETR_PTS as the new points for warping.*/
bool warpRavenJoints( Ravens &ravens,
		const PointCloudInfo &rope_info,
		//const PointCloudInfo &needle_info,
		const PointCloudInfo &cuts_info,
		const PointCloudInfo &holes_info,
		const vector< vector<dReal> >& in_joints, vector< vector<dReal> > & out_joints) {

	vector<vector<btVector3> > src_clouds, target_clouds;
	if (rope_info.first) {
		src_clouds.push_back(rope_info.second.first);
		target_clouds.push_back(rope_info.second.second);
	}

	//if (needle_info.first) {
	//	src_clouds.push_back(needle_info.second.first);
	//	target_clouds.push_back(needle_info.second.second);
	//}

	if (cuts_info.first) {
		src_clouds.push_back(cuts_info.second.first);
		target_clouds.push_back(cuts_info.second.second);
	}

	if (holes_info.first) {
		src_clouds.push_back(holes_info.second.first);
		target_clouds.push_back(holes_info.second.second);
	}

	RavensLfdRpm lfdrpm(ravens, src_clouds, target_clouds);
	lfdrpm.clear_grid();
	return lfdrpm.transformJointsTrajOpt(in_joints, out_joints);
}

/** Do trajectory optimization to solve for the new joint angles for getting to the new warped trasforms.
 *   Please ensure that the input transforms (OLD_TRANSFORMS) correspond to the palm links of the manipulator.*/
vector< vector<double> > doTrajectoryOptimization(RaveRobotObject::Manipulator::Ptr manip,
		const vector<btTransform> & palm_transforms,
		const vector< vector<dReal> > &old_joints) {

	RobotBasePtr robot     = manip->manip->GetRobot();
	EnvironmentBasePtr env = robot->GetEnv();
	int env_id             = RaveGetEnvironmentId(env);

	py::object py_env   = PyGlobals::openrave_module.attr("RaveGetEnvironment")(env_id);
	py::object py_robot = py_env.attr("GetRobot")(robot->GetName());

	vector<KinBody::LinkPtr> links;
	manip->manip->GetChildLinks(links);
	KinBody::LinkPtr palm_link = links[2];

	//py::object py_link     = py_robot.attr("GetLink")(palm_link->GetName());
	py::object py_link_name(palm_link->GetName());
	py::object py_mats   = transformsToNumpy(palm_transforms); //need to downsample?
	py::object py_old_joints = jointsToNumpy(old_joints);
	py::object py_manip_name(manip->manip->GetName());
	py::object py_traj;
	try {
		py_traj = PyGlobals::iros_utils_module.attr("plan_follow_traj")(py_robot, py_manip_name, py_link_name, py_mats, py_old_joints);
	} catch(...) {
		PyErr_Print();
	}
	vector<vector<double> > new_joints = jointsFromNumpy(py_traj);
	return new_joints;
}



/** Do trajectory optimization to solve for the new joint angles for getting to the new warped trasforms.
 *   Please ensure that the input transforms (OLD_TRANSFORMS) correspond to the palm links of the manipulator.*/
vector< vector<double> > doTrajectoryOptimization2(RaveRobotObject::Manipulator::Ptr manip, std::string finger1_name, std::string finger2_name,
		const vector<btTransform> & finger1_transforms, const vector<btTransform> & finger2_transforms,
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








bool RavensLfdRpm::transformJointsTrajOptWithIK(const vector<vector<dReal> > &joints, vector<vector<dReal> > &new_joints) {

	vector<KinBody::LinkPtr> links;
	ravens.manipR->manip->GetChildLinks(links);
	KinBody::LinkPtr r_finger1_link = links[1];
	KinBody::LinkPtr r_finger2_link = links[2];

	links.clear();
	ravens.manipL->manip->GetChildLinks(links);
	KinBody::LinkPtr l_finger1_link = links[1];
	KinBody::LinkPtr l_finger2_link = links[2];

	double tol = 0.02;  //DOWNSAMPLE
	std::pair< vector <float>, vector < vector <double> > > times_joints = adaptive_resample(joints, tol);
	vector<float> resampled_times             = times_joints.first;
	vector <vector<double> > resampled_joints = times_joints.second;

	/** Do forward-kinematics and get the end-effector transform. */
	vector<btTransform> right1Transforms(resampled_joints.size());
	vector<btTransform> right2Transforms(resampled_joints.size());
	vector<btTransform> left1Transforms(resampled_joints.size());
	vector<btTransform> left2Transforms(resampled_joints.size());
	vector<btTransform> leftEETransforms(resampled_joints.size());
	vector<btTransform> rightEETransforms(resampled_joints.size());

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

		leftEETransforms[i] = util::scaleTransform(ravens.manipL->getFK(l_joints), 1.f/METERS);
		rightEETransforms[i] = util::scaleTransform(ravens.manipR->getFK(r_joints), 1.f/METERS);
	}

	/** Warp the end-effector transforms. */
	vector<btTransform> warpedRight1Transforms = lfdrpm->transform_frames(right1Transforms);
	vector<btTransform> warpedLeft1Transforms  = lfdrpm->transform_frames(left1Transforms);
	vector<btTransform> warpedRight2Transforms = lfdrpm->transform_frames(right2Transforms);
	vector<btTransform> warpedLeft2Transforms  = lfdrpm->transform_frames(left2Transforms);

	vector<btTransform> warpedRightEETransforms = lfdrpm->transform_frames(rightEETransforms);
	vector<btTransform> warpedLeftEETransforms  = lfdrpm->transform_frames(leftEETransforms);


	vector<vector<dReal> > r_ik_joints;
	doSmoothIKAllJoints(ravens.manipR, warpedRightEETransforms, r_ik_joints);
	vector<vector<dReal> > l_ik_joints;
	doSmoothIKAllJoints(ravens.manipL, warpedLeftEETransforms, l_ik_joints);


	/** Do trajectory optimization on the warped transforms. */
	vector<vector<dReal> > new_r_joints =	 doTrajectoryOptimization2(ravens.manipR, r_finger1_link->GetName(), r_finger2_link->GetName(),warpedRight1Transforms, warpedRight2Transforms, r_ik_joints);
	vector<vector<dReal> > new_l_joints =	 doTrajectoryOptimization2(ravens.manipL, l_finger1_link->GetName(), l_finger2_link->GetName(),warpedLeft1Transforms, warpedLeft2Transforms, l_ik_joints);

	// upsample : interpolate
	vector<float> new_times(joints.size());
	for (int i = 0.0; i < joints.size(); ++i) new_times[i] = (float) i;
	vector<vector <dReal> > interpolated_r_joints = interpolate(new_times, new_r_joints, resampled_times);
	vector<vector <dReal> > interpolated_l_joints = interpolate(new_times, new_l_joints, resampled_times);

	/*
	vector<btVector3> pts0(left1Transforms.size()-1);
	vector<btVector3> pts1(left1Transforms.size()-1);
	for (int i =0; i<left1Transforms.size()-1; i+=1) {
		pts0[i]    = METERS*left1Transforms[i].getOrigin();
		pts1[i]    = METERS*left1Transforms[i+1].getOrigin();
	}
	util::drawLines(pts0, pts1, Eigen::Vector3f(1,0,0), 0.5, ravens.scene.env);*/

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



/** Does smooth IK on transforms (in joint space: basically chooses the closest subsequent joint-set [l2 normwise].
 *  Ik is done for each transform in TRANSFORMS and the corresponding joints are stored in JOINTS.*/
bool RavensLfdRpm::doSmoothIKAllJoints(RaveRobotObject::Manipulator::Ptr manip, const vector<btTransform> & transforms,
		vector< vector<dReal> > &joints) {

	vector< vector<dReal> > old_joints;
	vector<float> time_stamps;
	vector<dReal> currentDOFs = manip->getDOFValues();

	for(int i = 0; i < transforms.size(); i+=1) {
		vector <vector<dReal> > values;
		if (manip->solveAllIKUnscaled(util::toRaveTransform(transforms[i]), values)) {
			time_stamps.push_back(i);
			int solSize = values.size();

			vector<double> * bestDOFs (new vector<double>());
			*bestDOFs = values[0];

			double bestL2 = util::wrapAroundL2(*bestDOFs, currentDOFs);

			for (int j = 1; j < solSize; ++j) {
				double newL2 = util::wrapAroundL2(values[j],currentDOFs);
				if (newL2 < bestL2) {
					*bestDOFs = values[j];
					bestL2 = newL2;
				}
			}
			old_joints.push_back(*bestDOFs);
			currentDOFs = *bestDOFs;
		}
	}

	joints.clear();
	vector<float> new_time_stamps(transforms.size());
	for (int i = 0; i < new_time_stamps.size(); ++i) new_time_stamps[i] = i;

	joints = interpolate (new_time_stamps, old_joints, time_stamps);

	return true;
}
