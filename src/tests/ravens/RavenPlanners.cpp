/**
 * Author: Ankush Gupta
 * Date  : 16th November, 2012
 */


#include "RavenPlanners.h"
#include <math.h>

/** _ROBOT :  is the robot for which we want to plan.
 *  RAVE :  The rave instance in which we wish to plan.
 *  SIDE :  specifies which arm to plan for:  if 'l' : left else, right.*/
EndTransformPlanner::EndTransformPlanner(RaveRobotObject::Ptr _robot,
		RaveInstance::Ptr _rave, char side) : rave(_rave), ravens(_robot) {
	baseModule = RaveCreateModule(_rave->env, "BaseManipulation");
	manipName  = (side == 'l')? "l_arm" : "r_arm";
	rave->env->Add(baseModule, true, ravens->robot->GetName());
}


/** Tries to ACHIEVE the given transform GOAL PRECISELY (exact [roll, pitch, yaw]).
 *  If no plan is found, it returns failure and an uninitialized trajectory.*/
std::pair<bool, RaveTrajectory::Ptr> EndTransformPlanner::precisePlan(OpenRAVE::Transform goal) {
	OpenRAVE::RobotBase::ManipulatorPtr currentManip = ravens->robot->GetActiveManipulator();
	OpenRAVE::RobotBase::ManipulatorPtr manip        = ravens->robot->SetActiveManipulator(manipName);

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
	RaveTrajectory::Ptr raveTraj(new RaveTrajectory(traj, ravens, manip->GetArmIndices()));

	// restore the manipulator
	ravens->robot->SetActiveManipulator(currentManip);
	return std::make_pair(true, raveTraj);
}


/** Tries a bunch of [pitch, yaw] of the end-effector around the given goal.
 *  If no plan is found, it returns failure and an uninitialized trajectory.*/
std::pair<bool, RaveTrajectory::Ptr> EndTransformPlanner::forcePlan(OpenRAVE::Transform goal) {
	const float pi = OpenRAVE::PI;
	float _yaws[]    = {0, -pi/8, pi/8, -pi/6, pi/6, -pi/4, pi/4};
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


/** _ROBOT: is the robot for which we want to plan.
 *  RAVE :  The rave instance in which we wish to plan.
 *  SIDE :  specifies which arm to plan for:  if 'l' : left else, right.*/
WayPointsPlanner::WayPointsPlanner(RaveRobotObject::Ptr _robot,
		RaveInstance::Ptr _rave, char side) : rave(_rave), ravens(_robot),
		maxVelocities(7,1.0), maxAccelerations(7,5.0) {

	manipName = (side == 'l')? "l_arm" : "r_arm";

	OpenRAVE::RobotBase::ManipulatorPtr currentManip = ravens->robot->GetActiveManipulator();
	OpenRAVE::RobotBase::ManipulatorPtr manip        =	ravens->robot->SetActiveManipulator(manipName);
	ravens->robot->SetActiveDOFs(manip->GetArmIndices());

	planner = RaveCreatePlanner(rave->env, "workspacetrajectorytracker");
	params.reset(new WorkspaceTrajectoryParameters(_rave->env));
	params->_nMaxIterations = 2000;

	// set planning configuration space to current active dofs
	params->SetRobotActiveJoints(ravens->robot);

	// restore the manipulator
	ravens->robot->SetActiveManipulator(currentManip);
}


/** Returns a plan passing through the way-points specified in TRANSFORMS. */
std::pair<bool, RaveTrajectory::Ptr> WayPointsPlanner::plan(std::vector<OpenRAVE::Transform> &transforms) {

	OpenRAVE::RobotBase::ManipulatorPtr currentManip = ravens->robot->GetActiveManipulator();
	OpenRAVE::RobotBase::ManipulatorPtr manip        = ravens->robot->SetActiveManipulator(manipName);

	ConfigurationSpecification spec = IkParameterization::GetConfigurationSpecification(IKP_Transform6D, "quadratic");
	TrajectoryBasePtr workspacetraj = RaveCreateTrajectory(rave->env,"");
	workspacetraj->Init(spec);

	/** Insert the way-points into a trajectory. */
    vector<vector <dReal> *> DOFs;
	for(int i = 0; i < transforms.size(); ++i) {
		IkParameterization ikparam(transforms[i], IKP_Transform6D);
		vector<dReal> * values (new vector<dReal>);
		values->resize(ikparam.GetNumberOfValues());
		ikparam.GetValues(values->begin());
		DOFs.push_back(values);
	}

    unwrapWayPointDOFs(DOFs);
    	for (int i = 0; i < DOFs.size(); ++i)
    		workspacetraj->Insert(workspacetraj->GetNumWaypoints(),*DOFs[i]);

	//RAVELOG_INFO("BEFORE : Retimed Trajectory: %f, Num waypoints: %d\n", workspacetraj->GetDuration(), workspacetraj->GetNumWaypoints());
	planningutils::RetimeAffineTrajectory(workspacetraj,maxVelocities,maxAccelerations);
	//RAVELOG_INFO("AFTER : Retimed Trajectory: %f, Num waypoints: %d\n", workspacetraj->GetDuration(), workspacetraj->GetNumWaypoints());

	TrajectoryBasePtr outputtraj;
	{
		EnvironmentMutex::scoped_lock lock(rave->env->GetMutex()); // lock environment
        ravens->robot->SetActiveDOFs(manip->GetArmIndices());
		params->workspacetraj = workspacetraj;

		if( !planner->InitPlan(ravens->robot, params)) {
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
	ravens->robot->SetActiveManipulator(currentManip);

	RaveTrajectory::Ptr raveTraj(new RaveTrajectory(outputtraj, ravens, manip->GetArmIndices()));
	return std::make_pair(true, raveTraj);
}


/** _ROBOT :  is the robot for which we want to plan.
 *  SIDE   :  specifies which arm to plan for:  if 'l' : left else, right.*/
IKInterpolationPlanner::IKInterpolationPlanner(Ravens &_ravens,
		RaveInstance::Ptr _rave, char side) : rave(_rave), ravens(_ravens.ravens),
		maxVelocities(6,2.0), maxAccelerations(6,5.0) {

	manip = (side == 'l')? _ravens.manipL:_ravens.manipR;
}


/** Returns a plan passing through the way-points specified in TRANSFORMS. */
std::pair<bool, RaveTrajectory::Ptr> IKInterpolationPlanner::plan(std::vector<OpenRAVE::Transform> &transforms) {

	assert(("IKPlanner Error : Not enough target points given. Expecting at least 1.", transforms.size()>0));

    TrajectoryBasePtr traj = RaveCreateTrajectory(rave->env,"");
    ConfigurationSpecification spec = IkParameterization::GetConfigurationSpecification(IKP_Transform6D, "quintic");
    traj->Init(spec);

    /** Insert the way-points into a trajectory. */
    vector<dReal> values;

    if (transforms.size()==1) { // if the user only passed one transform, add another
    	std::vector<OpenRAVE::Transform> transformsN;
    	OpenRAVE::Vector currTrans = manip->origManip->GetEndEffectorTransform().trans;
    	OpenRAVE::Transform interT= transforms[0];

    	interT.trans = (interT.trans  + currTrans)*0.5;
    	transformsN.push_back(interT);
    	transformsN.push_back(transforms[0]);
    	transforms = transformsN;
    	assert(("There should be 2 transforms in the vector. Not Found!", transforms.size()==2));
    }

    for(int i = 0; i < transforms.size(); ++i) {
    	if (manip->solveIKUnscaled(transforms[i], values)) {
    		traj->Insert(traj->GetNumWaypoints(),values);
    	} else {//failure
    		RAVELOG_INFO("No plan through waypoints found : IK Failure.\n");
    		return std::make_pair(false, RaveTrajectory::Ptr());
    	}
    }
    planningutils::RetimeAffineTrajectory(traj,maxVelocities,maxAccelerations);

	RaveTrajectory::Ptr raveTraj(new RaveTrajectory(traj, ravens, manip->origManip->GetArmIndices()));
	return std::make_pair(true, raveTraj);
}


/** Returns a plan passing through the way-points specified in TRANSFORMS.
	 *  Picks the IK values which are close to each other.
	 *
	 *	May take a long time, as very large number of IK solutions
	 *	could be generated for each way-point. */
std::pair<bool, RaveTrajectory::Ptr> IKInterpolationPlanner::smoothPlan(std::vector<OpenRAVE::Transform> &transforms) {

	assert(("IKPlanner Error : Not enough target points given. Expecting at least 1.", transforms.size()>0));

	double pi = 3.14159265;

    TrajectoryBasePtr traj = RaveCreateTrajectory(rave->env,"");
    traj->Init(manip->origManip->GetArmConfigurationSpecification());


    /** Insert the way-points into a trajectory. */

    vector <vector<dReal> > values;

    if (transforms.size()==1) { // if the user only passed one transform, add another
    	std::vector<OpenRAVE::Transform> transformsN;
    	OpenRAVE::Vector currTrans = manip->origManip->GetEndEffectorTransform().trans;
    	OpenRAVE::Transform interT= transforms[0];

    	interT.trans = (interT.trans  + currTrans)*0.5;
    	transformsN.push_back(interT);
    	transformsN.push_back(transforms[0]);
    	transforms = transformsN;
    	assert(("There should be 2 transforms in the vector. Not Found!", transforms.size()==2));
    }

    vector<vector <dReal> *> DOFs;
    vector<dReal> currentDOFs = this->manip->getDOFValues();

    for(int i = 0; i < transforms.size(); ++i) {
    	if (manip->solveAllIKUnscaled(transforms[i], values)) {

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
    		DOFs.push_back(bestDOFs);

    		//std::cout<<"Iter "<<i+1<<": ";
    		//for (int k = 0; k < bestDOFs->size(); ++k) std::cout<<bestDOFs->at(k)<<" ";
    		//std::cout<<std::endl;
    		currentDOFs = *bestDOFs;


    	} else {//failure
    		RAVELOG_INFO("No plan through waypoints found : IK Failure.\n");
    		return std::make_pair(false, RaveTrajectory::Ptr());
    	}
    }

    unwrapWayPointDOFs(DOFs);
    for (int i = 0; i < DOFs.size(); ++i) {
    	//std::cout<<"Iter "<<i+1<<": ";
    	//for (int k = 0; k < DOFs[i]->size(); ++k) std::cout<<DOFs[i]->at(k)<<" ";
    	//std::cout<<std::endl;
    	traj->Insert(traj->GetNumWaypoints(),*DOFs[i]);
    }

    planningutils::RetimeAffineTrajectory(traj,maxVelocities,maxAccelerations);

	RaveTrajectory::Ptr raveTraj(new RaveTrajectory(traj, ravens, manip->origManip->GetArmIndices()));
	return std::make_pair(true, raveTraj);
}

std::pair<bool, RaveTrajectory::Ptr> IKInterpolationPlanner::goInDirection (char dir, double dist, int steps) {

	OpenRAVE::Transform initTrans = manip->origManip->GetEndEffectorTransform();

	btVector3 dirVec;
	btTransform btT = util::toBtTransform(initTrans);
	btMatrix3x3 tfm = btT.getBasis();

	switch (dir) {
		case 'f':
			dirVec = tfm.getColumn(2);
			break;
		case 'b':
			dirVec = -1*tfm.getColumn(2);
			break;
		case 'u':
			dirVec = tfm.getColumn(1);
			break;
		case 'd':
			dirVec = -1*tfm.getColumn(1);
			break;
		case 'l':
			dirVec = tfm.getColumn(0);
			break;
		case 'r':
			dirVec = -1*tfm.getColumn(0);
			break;
		default:
			RAVELOG_ERROR("Unknown direction: %c", dir);
			break;
	}

	btVector3 endOffset = dirVec*dist;
	vector< OpenRAVE::Transform > wayPoints;

	for (int currStep = 0; currStep <= steps; ++currStep) {
		btVector3 currVec = btT.getOrigin() + (currStep/(double)steps)*endOffset;

		btTransform T;
		T.setBasis(btT.getBasis());
		T.setOrigin(currVec);

		OpenRAVE::Transform raveT = util::toRaveTransform(T);
		wayPoints.push_back(raveT);
		//util::drawAxes(util::toBtTransform(raveT,GeneralConfig::scale),2,scene.env);
	}

	return smoothPlan(wayPoints);
}

std::pair<bool, RaveTrajectory::Ptr> IKInterpolationPlanner::goInWorldDirection (char dir, double dist, int steps) {

	OpenRAVE::Transform initTrans = manip->origManip->GetEndEffectorTransform();

	btVector3 dirVec;
	btTransform btT = util::toBtTransform(initTrans);
	btMatrix3x3 tfm = ravens->getLinkTransform(ravens->robot->GetLink("base_plate")).getBasis();

	switch (dir) {
		case 'u':
			dirVec = tfm.getColumn(2);
			break;
		case 'd':
			dirVec = -1*tfm.getColumn(2);
			break;
		case 'l':
			dirVec = tfm.getColumn(1);
			break;
		case 'r':
			dirVec = -1*tfm.getColumn(1);
			break;
		case 'f':
			dirVec = tfm.getColumn(0);
			break;
		case 'b':
			dirVec = -1*tfm.getColumn(0);
			break;
		default:
			RAVELOG_ERROR("Unknown direction: %c", dir);
			break;
	}

	btVector3 endOffset = dirVec*dist;
	vector< OpenRAVE::Transform > wayPoints;

	for (int currStep = 0; currStep <= steps; ++currStep) {
		btVector3 currVec = btT.getOrigin() + (currStep/(double)steps)*endOffset;

		btTransform T;
		T.setBasis(btT.getBasis());
		T.setOrigin(currVec);

		OpenRAVE::Transform raveT = util::toRaveTransform(T);
		wayPoints.push_back(raveT);
		//util::drawAxes(util::toBtTransform(raveT,GeneralConfig::scale),2,scene.env);
	}

	return smoothPlan(wayPoints);
}

/** Unwraps vector of way points with DOF values wrapped around from pi to -pi.
 *  Returns a vector with no DOF wrap-around.
 *
 *  This post-processing step gets rid of extraneous turns while
 *  executing the trajectory. */
void unwrapWayPointDOFs (vector< vector <dReal> *> &WayPointDOFs){
	dReal pi = 3.14159265;
	int numWayPoints = WayPointDOFs.size();

	for (int i = 1; i < numWayPoints; ++i) {
		for (int k = 0; k < WayPointDOFs[i]->size(); ++k) {

			int div = floor((WayPointDOFs[i-1]->at(k)+pi)/(2*pi));
			double multOfPi = 2*pi*div - pi;
			double bdofshift = fmod(WayPointDOFs[i]->at(k)+pi,2*pi);
			if (fabs(WayPointDOFs[i-1]->at(k) - (multOfPi-2*pi+bdofshift)) < fabs(WayPointDOFs[i-1]->at(k) - (multOfPi+bdofshift)))
				WayPointDOFs[i]->at(k) = multOfPi-2*pi+bdofshift;
			else if (fabs(WayPointDOFs[i-1]->at(k) - (multOfPi+bdofshift)) < fabs(WayPointDOFs[i-1]->at(k) - (multOfPi+2*pi+bdofshift)))
				WayPointDOFs[i]->at(k) = multOfPi+bdofshift;
			else
				WayPointDOFs[i]->at(k) = multOfPi+2*pi+bdofshift;
		}
	}
}
