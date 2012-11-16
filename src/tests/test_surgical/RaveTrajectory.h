#ifndef __RAVE_TRAJECTORY_H__
#define __RAVE_TRAJECTORY_H__

#include <openrave/openrave.h>

using namespace OpenRAVE;
using namespace std;

/** A wrapper around OpenRAVE trajectories. */
class RaveTrajectory {

private:
	/** Samples the trajectory at time T, and stores
	 *  the joint-values TIMEDERIVATIVE_th derivative in values.*/
	void sample(std::vector<dReal> &values, dReal t, int timederivative=0) {
		values.clear();
		values.resize(dofIndices.size());
		vector<dReal> s;
		trajectory->Sample(s, t);
		trajectory->GetConfigurationSpecification().ExtractJointValues(values.begin(),
				s.begin(), probot, dofIndices, timederivative);
	}

public:
	typedef boost::shared_ptr<RaveTrajectory> Ptr;

	/** Pointer to the openrave trajectory.*/
	TrajectoryBasePtr trajectory;

	/** Indices of the DOFs of the PR2 rave model to which the
	 *  trajectory's joint values correspond to.*/
	vector<int> dofIndices;

	/** Pointer to the openrave kinbody (e.g. RobotBase)
	 *  whose trajectory is being planned.*/
	RobotBasePtr probot;

	/** @param:
	 *   traj   : the openrave trajectory this class is wrapping
	 *   robot  : the openrave kinbody whose trajectory it is.
	 *	 indices: the indices of the dofs whose joint values need to be sampled. */
	RaveTrajectory(TrajectoryBasePtr traj, RobotBasePtr robot, vector<int> &indices) :
			trajectory(traj), dofIndices(indices), probot(robot) {	}

	/** Returns the time length of the trajectory. */
	float duration() {
		return (float) trajectory->GetDuration();
	}

	/** Samples the trajectory at time T, and stores the joint values in JOINTS.*/
	void sampleJoints(std::vector<dReal> &joints, dReal t) {
		sample(joints, t, 0);
	}

	/** Samples the trajectory at time T, and stores the joint values in JOINTS.*/
	void sampleJointVelocities(std::vector<dReal> &velocities, dReal t) {
		sample(velocities, t, 1);
	}
};
#endif
