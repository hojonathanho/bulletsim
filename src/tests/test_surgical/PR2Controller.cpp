/**
 * Author: Ankush Gupta
 * Date  : 15th November, 2012
 */

#include "PR2Controller.h"

void PR2Controller::execute() {
	if (!enabled) return;

	if (trajectories.empty()) {
		enabled = false;
		return;
	}

	RaveTrajectory::Ptr traj = trajectories.front();
	vector<dReal> jointVals;

	if (currentTime >= traj->duration()) {
		traj->sampleJoints(jointVals, traj->duration());
		pr2->setDOFValues(traj->dofIndices, jointVals);
		trajectories.pop();
		currentTime = 0;
	} else {
		traj->sampleJoints(jointVals, currentTime);
		pr2->setDOFValues(traj->dofIndices, jointVals);
		currentTime += dt;
	}
}
