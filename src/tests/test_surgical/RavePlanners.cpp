/**
 * Author: Ankush Gupta
 * Date  : 16th November, 2012
 */


#include "RavePlanners.h"



/** _PR2 :  is the PR2 [robot] for which we want to plan.
 *  RAVE :  The rave instance in which we wish to plan.
 *  SIDE :  specifies which arm to plan for:  if 'l' : left else, right.*/
EndTransformPlanner::EndTransformPlanner(RaveRobotObject::Ptr _pr2,
		RaveInstance::Ptr _rave, char side) : rave(_rave), pr2(_pr2),
		baseModule(new RaveCreateModule(_rave->env, "BaseManipulation")) {
	manipName = (side == 'l')? "leftarm" : "rightarm";
	rave->env->Add(baseModule, true, pr2->robot->GetName());
}


/** Tries to ACHIEVE the given transform GOAL PRECISELY (exact [roll, pitch, yaw]).
 *  If no plan is found, it returns failure and an uninitialized trajectory.*/
std::pair<bool, RaveTrajectory::Ptr> EndTransformPlanner::precisePlan(OpenRAVE::Transform goal) {
	OpenRAVE::RobotBase::ManipulatorPtr currentManip = pr2->robot->GetActiveManipulator();
	OpenRAVE::RobotBase::ManipulatorPtr manip =	pr2->robot->SetActiveManipulator(manipName);

	TrajectoryBasePtr traj;
	traj = RaveCreateTrajectory(rave->env,"");

	stringstream ssout, ssin; ssin << "MoveToHandPosition outputtraj execute 0 poses 1  " << goal;
	if (!baseModule->SendCommand(ssout,ssin)) {
		// on failure:
		return std::make_pair(false, RaveTrajectory::Ptr());
	} else {
		// if success, extract the trajectory.
		traj->deserialize(ssout);
	}
	RaveTrajectory::Ptr raveTraj(new RaveTrajectory(traj, pr2, manip->GetArmIndices()));

	// restore the manipulator
	pr2->robot->SetActiveManipulator(currentManip);
	return std::make_pair(true, raveTraj);
}


/** Tries a bunch of [pitch, yaw] of the end-effector around the given goal.
 *  If no plan is found, it returns failure and an uninitialized trajectory.*/
std::pair<bool, RaveTrajectory::Ptr> EndTransformPlanner::forcePlan(OpenRAVE::Transform goal) {
	const float pi = OpenRAVE::PI;
	vector<float> yaws    = {0, -pi/8, pi/8, -pi/6, pi/6, -pi/4, pi/4};
	vector<float> pitches = {0, -pi/8, pi/8, -pi/6, pi/6, -pi/4, pi/4};

	for (int y=0; y < yaws.size(); y++) {
		Transform yMatrix = matrixFromAxisAngle(Vector(0,0,yaws[y]));
		Transform goalTemp   = yMatrix * goal;
		for(int p=0; p < pitches.size(); p++) {
			Transform pMatrix = matrixFromAxisAngle(Vector(0,pitches[p],0));
			Transform goalT = pMatrix * goalTemp;
			std::pair<bool, RaveTrajectory::Ptr> res = precisePlan(goalT);
			if (res.first) return res;
		}
	}
	// on failure:
	return std::make_pair(false, RaveTrajectory::Ptr());
}



/** _PR2 :  is the PR2 [robot] for which we want to plan.
 *  RAVE :  The rave instance in which we wish to plan.
 *  SIDE :  specifies which arm to plan for:  if 'l' : left else, right.*/
WayPointsPlanner::WayPointsPlanner(RaveRobotObject::Ptr _pr2,
		RaveInstance::Ptr _rave, char side) : rave(_rave), pr2(_pr2),
		planner(new RaveCreatePlanner(rave->env, "workspacetrajectorytracker")),
		params(new WorkspaceTrajectoryParameters(_rave->env)),
		maxVelocities(7,1.0), maxAccelerations(7,5.0) {
	manipName = (side == 'l')? "leftarm" : "rightarm";

	OpenRAVE::RobotBase::ManipulatorPtr currentManip = pr2->robot->GetActiveManipulator();
	OpenRAVE::RobotBase::ManipulatorPtr manip =	pr2->robot->SetActiveManipulator(manipName);
	pr2->robot->SetActiveDOFs(manip->GetArmIndices());

	// set planning configuration space to current active dofs
	params->SetRobotActiveJoints(pr2->robot);

	pr2->robot->SetActiveManipulator(currentManip);

}


/** Returns a plan passing through the way-points specified in TRANSFORMS. */
std::pair<bool, RaveTrajectory::Ptr> WayPointsPlanner::plan(std::vector<OpenRAVE::Transform> &transforms) {

	OpenRAVE::RobotBase::ManipulatorPtr currentManip = pr2->robot->GetActiveManipulator();
	OpenRAVE::RobotBase::ManipulatorPtr manip =	pr2->robot->SetActiveManipulator(manipName);

	ConfigurationSpecification spec = IkParameterization::GetConfigurationSpecification(IKP_Transform6D, "linear");
	TrajectoryBasePtr workspacetraj = RaveCreateTrajectory(rave->env,"");
	vector<dReal> values;
	workspacetraj->Init(spec);

	/** Insert the way-points into a trajectory. */
	vector<dReal> values;
	for(int i = 0; i < transforms.size(); ++i) {
		IkParameterization ikparam(transforms[i], IKP_Transform6D);
		values.resize(ikparam.GetNumberOfValues());
		ikparam.GetValues(values.begin());
		workspacetraj->Insert(workspacetraj->GetNumWaypoints(),values);
	}
	planningutils::RetimeAffineTrajectory(workspacetraj,maxVelocities,maxAccelerations);

	TrajectoryBasePtr outputtraj;
	{
		EnvironmentMutex::scoped_lock lock(rave->env->GetMutex()); // lock environment
		params->workspacetraj = workspacetraj;

		RAVELOG_INFO("starting to plan\n");
		if( !planner->InitPlan(pr2->robot, params))
			return std::make_pair(false, RaveTrajectory::Ptr()); //failure

		// create a new output trajectory
		outputtraj = RaveCreateTrajectory(rave->env,"");
		if( !planner->PlanPath(outputtraj))
			return std::make_pair(false, RaveTrajectory::Ptr()); //failure
	}

	RaveTrajectory::Ptr raveTraj(new RaveTrajectory(outputtraj, pr2, manip->GetArmIndices()));
	return std::make_pair(true, raveTraj);
}
