#ifndef __JOINT_PLAYBACK__
#define __JOINT_PLAYBACK__

#include <sstream>
#include <string>
#include <assert.h>
//#include <vector>

#include "robots/ravens.h"

/* Class to load a trajectory and play it back. **/
class jointPlayback {

	Scene &scene;					// Scene in which robot is
	Ravens::Ptr ravens;				// Ravens from the scene

	vector<int> joint_inds;		 	// Indices of joints to set DOF values
	vector<dReal> joint_vals;		// Latest joint values loaded from file.

	float freq;			// Frequency of recording
	float dt;						// Time step of simulation
	float currTime;					// Amount of time passed since last execution

	string filename;				// Name of file to read from
	ifstream file;					// Input file stream to read file

	bool enabled;					// Check if trajectory is currently playing back
	bool file_closed;				// Check if the file is closed

public:

	typedef boost::shared_ptr<jointPlayback> Ptr;

	// Constructor
	jointPlayback (Scene &_scene, Ravens * _robot, float _freq = -1.0, float _dt = -1.0) :
		scene(_scene), ravens (_robot), freq (_freq), dt (_dt), currTime (0.0),
		enabled(false), file_closed(true) {

		filename = "/home/ankush/sandbox/bulletsim/src/tests/ravens/recorded/raven_joints.txt";

		if (freq == -1.0)
			freq = RavenConfig::record_freq;

		if (dt == -1.0)
			dt = BulletConfig::dt;

		int dof = ravens->ravens->robot->GetDOF();
		for (int i = 0; i < dof; ++i) joint_inds.push_back(i);

		scene.addPreStepCallback(boost::bind(&jointPlayback::executeNextWaypoint, this));
	}

	/** Start the controller. */
	void run() {
		enabled = true;
		if (file_closed) {
			file.open(filename.c_str(), ios::in);
			file_closed = false;
		}
	}

	void loadTrajectory() {}
	void runTrajectory() {}

	/* Halt the controller.
	 * Controller does not move the robot until run/ resume is called. */
	void halt() {enabled = false;}

	/** Resume the operation of the controller after a halt.*/
	void resume() {run();}

	// Goes without saying what this does
	void toggleEnabled () {
		if (enabled) halt();
		else run();

		std::cout<<"Playback: "<<(enabled ? "true" : "false") << std::endl;
	}

	void executeNextWaypoint () {
		if (!enabled)
			return;

		string joints;
		float jval;

		// If next way point not loaded, load it.
		if (currTime >= 1/freq) {

			if (getline(file, joints)) {
				istringstream in(joints);
				joint_vals.clear();
				while (in >> jval) joint_vals.push_back(jval);
			} else {
				file.close();
				enabled = false;
				file_closed = true;
				return;
			}

			ravens->ravens->setDOFValues(joint_inds, joint_vals);
			currTime = 0.0;
		} else currTime += dt;
	}

	~jointPlayback() {file.close();}

};

#endif
