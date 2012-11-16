/**
 * Author: Ankush Gupta
 * Date  : 15th November, 2012
 */

#ifndef __PR2_CONTROLLER_H__
#define __PR2_CONTROLLER_H__

#include "robots/pr2.h"
#include "RaveTrajectory.h"
#include <queue>

/*Class for executing OpenRAVE trajectories on a PR2 in Bulletsim. */
class PR2Controller {

	/** Pointer to the PR2. */
    RaveRobotObject::Ptr pr2;

    /** Reference to the scene in which PR2 is present. */
    Scene &scene;

    /** Controller moves the robot iff, ENABLED is TRUE. */
    bool enabled;

	/** Queue to hold the trajectories to be executed.
	 *  The trajectories present in this queue will be executed sequentially,
	 *  in order, back-to-back immediately.
	 *  A trajectory is removed from the queue as soon as its execution is finished.*/
	std::queue<RaveTrajectory::Ptr> trajectories;

	/** Stores the time since the beginning of the trajectory it is executing.*/
	double currentTime;

	/** Simulation time step. */
	float dt;

	/** Main loop which executes the trajectory. Called by the scene at each step.*/
	void execute();

public:

	PR2Controller (Scene &_scene, float _dt = -1) : scene(_scene), trajectories(), enabled(false),
		currentTime(0), dt(_dt) {

		scene.addPreStepCallback(boost::bind(&PR2Controller::execute, this));

		if (dt == -1)
			dt = BulletConfig::dt;
	}

	/** Append the TRAJ to the list of trajectories to be executed.*/
	void appendTrajectory(RaveTrajectory::Ptr traj) {
		trajectories.push(traj);
	}

	/** Return the number of trajectories in the queue.*/
	int getNumTrajectories() {return trajectories.size(); }

	/** Start the controller. */
	void run(){
		if (!trajectories.empty())
			enabled = true;
	}

	/* Halt the controller.
	 * Controller does not move the robot until run/ resume is called. */
	void halt() {
		enabled = false;
	}

	/** Resume the operation of the controller after a halt.*/
	void resume() {run();}

	/** Reset the controller. Halts execution if any in progress.
	 *  Clears the list of trajectories.*/
	void reset(){
		enabled = false;
		while(!trajectories.empty()) trajectories.pop();
	}
};

#endif
