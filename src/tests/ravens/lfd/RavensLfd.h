#pragma once
#include "LfdRpmWrapper.h"
#include <iostream>
#include "simulation/util.h"
#include "robots/ravens.h"
#include "simulation/plotting.h"


using namespace std;
typedef pair<bool, pair< vector<btVector3>, vector<btVector3> > >  PointCloudInfo;


/** Simple class which helps apply lfd by transforming the joints angles. */
class RavensLfdRpm {
private:

	PlotLines::Ptr plot_lines_left, plot_lines_right;

	/** Extract the joints indexed by INDS from IN_JOINT_VALS and store them into OUT_JOINT_VALS.*/
	void extractJoints (const vector<int> &inds, const vector<dReal> &in_joint_vals,
			vector<dReal> &out_joint_vals);

	/** Does smooth IK on transforms (in joint space: basically chooses the closest subsequent joint-set [l2 normwise].
	 *  Ik is done for each transform in TRANSFORMS and the corresponding joints are stored in JOINTS.*/
	bool doSmoothIK(RaveRobotObject::Manipulator::Ptr manip, const vector<btTransform> & transforms, vector< vector<dReal> > &joints);
	bool doSmoothIKAllJoints(RaveRobotObject::Manipulator::Ptr manip, const vector<btTransform> & transforms, vector< vector<dReal> > &joints);


	Ravens  &ravens;
	RegistrationModule::Ptr lfdrpm;

	vector<int> larm_indices;
	vector<int> rarm_indices;

	/** Plotting util functions. */
	void plotTransforms (const vector< btTransform > &transforms);
	void plotPoints     (const vector< btTransform > &transforms);
	void plotPath       (const vector< btTransform > &transforms, PlotLines::Ptr plot_lines);

public:
	/** Ravens   : the robot to transform the joints for.
	 *  SRC_PTS_ : the reference point locations.
	 *  TARGET_PTS_: the new point locations. */
	RavensLfdRpm (Ravens &ravens_, const vector<btVector3> &src_pts, const vector<btVector3> &target_pts);

	/** Ravens   : the robot to transform the joints for.
	 *  SRC_PTS_ : the reference point locations.
	 *  TARGET_PTS_: the new point locations. */
	RavensLfdRpm (Ravens &ravens_, const vector<vector<btVector3> > &src_clouds,
			       const vector<vector<btVector3> > & target_pts);

	/** Warp the joint angles of ravens using warping and IK.*/
	bool transformJoints(const vector<vector<dReal> > &joint_vals, vector< vector<dReal> > &new_joint_vals);

	/** Warp the joint angles of ravens using warping and trajectory optimization.*/
	bool transformJointsTrajOpt(const vector<vector<dReal> > &joints, vector<vector<dReal> > &new_joints);
	bool transformJointsTrajOptWithIK(const vector<vector<dReal> > &joints, vector<vector<dReal> > &new_joints) ;


};


/** Do trajectory optimization to solve for the new joint angles for getting to the new warped transforms. */
vector< vector<double> > doTrajectoryOptimization(RaveRobotObject::Manipulator::Ptr manip, const vector<btTransform> & transforms,
		const vector< vector<dReal> > &old_joints);

vector< vector<double> > doTrajectoryOptimization2(RaveRobotObject::Manipulator::Ptr manip, std::string link1_name, std::string link2_name,
		const vector<btTransform> & finger1_transforms, const vector<btTransform> & finger2_transforms,
		const vector< vector<dReal> > &old_joints);


/** Warp the joint values of the ravens using SRC_PTS as the reference
 *  and TARGETR_PTS as the new points for warping.*/
bool warpRavenJoints(Ravens &ravens,
		const vector<btVector3> &src_pts, const vector<btVector3> &target_pts,
		const vector< vector<dReal> >& in_joints, vector< vector<dReal> > & out_joints);


/** Warp the joint values of the ravens using point-clouds with known associations.
 *  ROPE/NEELDE/CUTS/HOLES info are pairs of point-clouds which need to be matched up for fitting TPS.
 *    >> Correspondences are not found across the pair of point-clouds.
 *  IN_JOINTS [in] are the joint-angles which need to be warped.
 *  OUT_JOINTS [out] is the output - the warped joint angles.*/
bool warpRavenJoints( Ravens &ravens,
						 const PointCloudInfo &rope_info,
						 const PointCloudInfo &needle_info,
						 const PointCloudInfo &cuts_info,
						 const PointCloudInfo &holes_info,
						 const vector< vector<dReal> >& in_joints, vector< vector<dReal> > & out_joints);
