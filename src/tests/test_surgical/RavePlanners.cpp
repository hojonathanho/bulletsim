/**
 * Author: Ankush Gupta
 * Date  : 16th November, 2012
 */


#include "RavePlanners.h"


/** _PR2 :  is the PR2 [robot] for which we want to plan.
 *  RAVE :  The rave instance in which we wish to plan.
 *  SIDE :  specifies which arm to plan for:  if 'l' : left else, right.*/
EndTransformPlanner::EndTransformPlanner(RaveRobotObject::Ptr _pr2,
		RaveInstance::Ptr _rave, char side) : rave(_rave), pr2(_pr2) {
	baseModule = RaveCreateModule(_rave->env, "BaseManipulation");
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
	float _yaws[] = {0, -pi/8, pi/8, -pi/6, pi/6, -pi/4, pi/4};
	float _pitches[] = {0, -pi/8, pi/8, -pi/6, pi/6, -pi/4, pi/4};

	vector<float> yaws;
	vector<float> pitches;
	yaws.assign(_yaws, _yaws+7);
	pitches.assign(_pitches, _pitches+7);


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
		maxVelocities(7,1.0), maxAccelerations(7,5.0) {

	manipName = (side == 'l')? "leftarm" : "rightarm";

	OpenRAVE::RobotBase::ManipulatorPtr currentManip = pr2->robot->GetActiveManipulator();
	OpenRAVE::RobotBase::ManipulatorPtr manip =	pr2->robot->SetActiveManipulator(manipName);
	pr2->robot->SetActiveDOFs(manip->GetArmIndices());

	planner = RaveCreatePlanner(rave->env, "workspacetrajectorytracker");
	params.reset(new WorkspaceTrajectoryParameters(_rave->env));
	params->_nMaxIterations = 2000;

	// set planning configuration space to current active dofs
	params->SetRobotActiveJoints(pr2->robot);

	// restore the manipulator
	pr2->robot->SetActiveManipulator(currentManip);
}


/** Returns a plan passing through the way-points specified in TRANSFORMS. */
std::pair<bool, RaveTrajectory::Ptr> WayPointsPlanner::plan(std::vector<OpenRAVE::Transform> &transforms) {

	OpenRAVE::RobotBase::ManipulatorPtr currentManip = pr2->robot->GetActiveManipulator();
	OpenRAVE::RobotBase::ManipulatorPtr manip =	pr2->robot->SetActiveManipulator(manipName);

	ConfigurationSpecification spec = IkParameterization::GetConfigurationSpecification(IKP_Transform6D, "quadratic");
	TrajectoryBasePtr workspacetraj = RaveCreateTrajectory(rave->env,"");
	workspacetraj->Init(spec);

	/** Insert the way-points into a trajectory. */
	vector<dReal> values;
	for(int i = 0; i < transforms.size(); ++i) {
		IkParameterization ikparam(transforms[i], IKP_Transform6D);
		values.resize(ikparam.GetNumberOfValues());
		ikparam.GetValues(values.begin());
		workspacetraj->Insert(workspacetraj->GetNumWaypoints(),values);
	}
	//RAVELOG_INFO("BEFORE : Retimed Trajectory: %f, Num waypoints: %d\n", workspacetraj->GetDuration(), workspacetraj->GetNumWaypoints());
	planningutils::RetimeAffineTrajectory(workspacetraj,maxVelocities,maxAccelerations);
	//RAVELOG_INFO("AFTER : Retimed Trajectory: %f, Num waypoints: %d\n", workspacetraj->GetDuration(), workspacetraj->GetNumWaypoints());

	TrajectoryBasePtr outputtraj;
	{
		EnvironmentMutex::scoped_lock lock(rave->env->GetMutex()); // lock environment
        pr2->robot->SetActiveDOFs(manip->GetArmIndices());
		params->workspacetraj = workspacetraj;

		if( !planner->InitPlan(pr2->robot, params)) {
			RAVELOG_INFO("Planner initialization failed.\n");
			return std::make_pair(false, RaveTrajectory::Ptr()); //failure
		}

		// create a new output trajectory
		outputtraj = RaveCreateTrajectory(rave->env,"");
		if( !planner->PlanPath(outputtraj)) {
			RAVELOG_INFO("No plan through waypoints found.\n");
			return std::make_pair(false, RaveTrajectory::Ptr()); //failure
		}
	}

	// restore the manipulator
	pr2->robot->SetActiveManipulator(currentManip);

	RaveTrajectory::Ptr raveTraj(new RaveTrajectory(outputtraj, pr2, manip->GetArmIndices()));
	return std::make_pair(true, raveTraj);
}


/** _PR2 :  is the PR2 [robot] for which we want to plan.
 *  SIDE :  specifies which arm to plan for:  if 'l' : left else, right.*/
IKInterpolationPlanner::IKInterpolationPlanner(PR2Manager &_pr2m,
		RaveInstance::Ptr _rave, char side) : rave(_rave), pr2(_pr2m.pr2),
		maxVelocities(7,2.0), maxAccelerations(7,5.0) {

	pr2manip = (side == 'l')? _pr2m.pr2Left :_pr2m.pr2Right;
}


/** Returns a plan passing through the way-points specified in TRANSFORMS. */
std::pair<bool, RaveTrajectory::Ptr> IKInterpolationPlanner::plan(std::vector<OpenRAVE::Transform> &transforms) {

	assert(("IKPlanner Error : Not enough target points given. Expecting at least 1.", transforms.size()>0));

    TrajectoryBasePtr traj = RaveCreateTrajectory(rave->env,"");
    traj->Init(pr2manip->origManip->GetArmConfigurationSpecification());


    /** Insert the way-points into a trajectory. */
    vector<dReal> values;

    if (transforms.size()==1) { // if the user only passed one transform, add another
    	std::vector<OpenRAVE::Transform> transformsN;
    	OpenRAVE::Vector currTrans = pr2manip->origManip->GetEndEffectorTransform().trans;
    	OpenRAVE::Transform interT= transforms[0];

    	interT.trans = (interT.trans  + currTrans)*0.5;
    	transformsN.push_back(interT);
    	transformsN.push_back(transforms[0]);
    	transforms = transformsN;
    	assert(("There should be 2 transforms in the vector. Not Found!", transforms.size()==2));
    }

    for(int i = 0; i < transforms.size(); ++i) {
    	if (pr2manip->solveIKUnscaled(transforms[i], values)) {
    		traj->Insert(traj->GetNumWaypoints(),values);
    	} else {//failure
    		RAVELOG_INFO("No plan through waypoints found : IK Failure.\n");
    		return std::make_pair(false, RaveTrajectory::Ptr());
    	}
    }
    planningutils::RetimeAffineTrajectory(traj,maxVelocities,maxAccelerations);

	RaveTrajectory::Ptr raveTraj(new RaveTrajectory(traj, pr2, pr2manip->origManip->GetArmIndices()));
	return std::make_pair(true, raveTraj);
}

