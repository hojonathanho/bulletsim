/**
 * Author: Ankush Gupta
 * Date  : 16th November, 2012
 */

#ifndef __RAVE_PLANNERS_H__
#define __RAVE_PLANNERS_H__


#include "simulation/openravesupport.h"
#include "robots/pr2.h"
#include "simulation/simplescene.h"
#include <openrave/plannerparameters.h>
#include <openrave/planningutils.h>

/**
 * Note : The classes below are for planning for PR2 ARMS only.
 * -----  Planning for other manipulators WILL NOT work. */


/** Plans a path to the transform [6D] given by the user.*/
class EndTransformPlanner {


	/** Instance of openrave in which we want to plan.*/
	RaveInstance::Ptr rave;

	/** The robot to plan for. */
	RaveRobotObject::Ptr pr2;

	/** Name of the manipulator of the arm to plan with. */
	std::string manipName;

	/** Base manipulation interface : for planning to end-effector final transforms.*/
	OpenRAVE::ModuleBasePtr baseModule;

public:

	typedef boost::shared_ptr<EndTransformPlanner> Ptr;

	/** _PR2 :  is the PR2 [robot] for which we want to plan.
	 *  RAVE :  The rave instance in which we wish to plan.
	 *  SIDE :  specifies which arm to plan for:  if 'l' : left else, right.*/
	EndTransformPlanner(RaveRobotObject::Ptr _pr2,
			RaveInstance::Ptr _rave, char side='l');

	/** Tries to ACHIEVE the given transform GOAL PRECISELY (exact [roll, pitch, yaw]).
	 *  If no plan is found, it returns failure and an uninitialized trajectory.*/
	std::pair<bool, RaveTrajectory::Ptr> precisePlan(OpenRAVE::Transform goal);


	/** Tries a bunch of [pitch, yaw] of the end-effector around the given goal.
     *  If no plan is found, it returns failure and an uninitialized trajectory.*/
	std::pair<bool, RaveTrajectory::Ptr> forcePlan(OpenRAVE::Transform goal);
};


/** Plans a path passing through given workspace points [transforms in 3d space]. */
class WayPointsPlanner {
	/** Instance of openrave in which we want to plan.*/
	RaveInstance::Ptr rave;

	/** The robot to plan for. */
	RaveRobotObject::Ptr pr2;

	/** Name of the manipulator of the arm to plan with. */
	std::string manipName;

	/** Planner.*/
	OpenRAVE::PlannerBasePtr planner;
	OpenRAVE::WorkspaceTrajectoryParametersPtr params;

	/** Maximum joint velocities and accelerations.*/
    std::vector<dReal> maxVelocities;
    std::vector<dReal> maxAccelerations;

public:

    typedef boost::shared_ptr<WayPointsPlanner> Ptr;

	/** _PR2 :  is the PR2 [robot] for which we want to plan.
	 *  RAVE :  The rave instance in which we wish to plan.
	 *  SIDE :  specifies which arm to plan for:  if 'l' : left else, right.*/
	WayPointsPlanner(RaveRobotObject::Ptr _pr2,
			RaveInstance::Ptr _rave, char side='l');

	/** Returns a plan passing through the way-points specified in TRANSFORMS. */
	std::pair<bool, RaveTrajectory::Ptr> plan(std::vector<OpenRAVE::Transform> &transforms);
};



/** Plans a path through given points [transforms in 3d space].
 *  Uses IK (does not use openrave planner).
 *
 *  This planner has a very high relative success rate compared
 *  to the ones above. */
class IKInterpolationPlanner {

	/** Instance of openrave in which we want to plan.*/
	RaveInstance::Ptr rave;

	/** The robot to plan for. */
	RaveRobotObject::Ptr pr2;
	RaveRobotObject::Manipulator::Ptr pr2manip;

	/** Name of the manipulator of the arm to plan with. */
	std::string manipName;

	/** Maximum joint velocities and accelerations.*/
    std::vector<dReal> maxVelocities;
    std::vector<dReal> maxAccelerations;

public:

    typedef boost::shared_ptr<IKInterpolationPlanner> Ptr;

	/** _PR2 :  is the manager of the robot.
	 *  SIDE :  specifies which arm to plan for:  if 'l' : left else, right.*/
	IKInterpolationPlanner(PR2Manager &_pr2m, RaveInstance::Ptr _rave, char side='l');

	/** Returns a plan passing through the way-points specified in TRANSFORMS. */
	std::pair<bool, RaveTrajectory::Ptr> plan(std::vector<OpenRAVE::Transform> &transforms);

	/** Returns a plan passing through the way-points specified in TRANSFORMS.
	 *  Picks the IK values which are close to each other.
	 *
	 *	May take a long time, as very large number of IK solutions
	 *	could be generated for each way-point. */
	std::pair<bool, RaveTrajectory::Ptr> smoothPlan(std::vector<OpenRAVE::Transform> &transforms);

	/** Goes in the direction specified by dir and distance specified by dist */
	std::pair<bool, RaveTrajectory::Ptr> goInDirection (char dir, double dist, Scene &scene, int steps=10);
};

#endif
