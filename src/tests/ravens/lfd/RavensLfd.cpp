#include "RavensLfd.h"

using namespace std;


RavensLfdRpm::RavensLfdRpm (Ravens & ravens_, const vector<btVector3> &src_pts,
		const vector<btVector3> &target_pts) : ravens(ravens_),
		lfdrpm(new RegistrationModule(src_pts, target_pts)) {

	if (src_pts.size() < 50 || target_pts.size() < 50)
		cout <<"LFD RPM : Warning too few points!"<<endl;

	std::cout<<"LFD RPM : Please make sure that the src and target points are scaled down by METERS."<<std::endl;

	larm_indices = ravens.manipL->manip->GetArmIndices();
	rarm_indices = ravens.manipR->manip->GetArmIndices();
}

/** Does smooth IK on transforms (in joint space: basically chooses the closest subsequent joint-set [l2 normwise].
  *  Ik is done for each transform in TRANSFORMS and the corresponding joints are stored in JOINTS.*/
bool RavensLfdRpm::doSmoothIK(RaveRobotObject::Manipulator::Ptr manip, const vector<btTransform> & transforms,
		vector< vector<dReal> > &joints) {
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


bool RavensLfdRpm::transformJoints(const vector<vector<dReal> > &joints, vector<vector<dReal> > &new_joints) {

	/** Do forward-kinematics and get the end-effector transform. */
	vector<btTransform> rightEETransforms(joints.size());
	vector<btTransform> leftEETransforms(joints.size());
	for (int i =0; i< joints.size(); i+=1) {
		vector<dReal> r_joints;
		extractJoints(rarm_indices, joints[i], r_joints);

		vector<dReal> l_joints;
		extractJoints(larm_indices, joints[i], l_joints);

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
		}
		return true;
	} else {
		return false;
	}
}

/** Extract the joints indexed by INDS from IN_JOINT_VALS and store them into OUT_JOINT_VALS.*/
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
	return lfdrpm.transformJoints(in_joints, out_joints);
}
