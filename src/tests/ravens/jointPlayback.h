#ifndef __JOINT_PLAYBACK__
#define __JOINT_PLAYBACK__

#include <sstream>
#include <string>

#include "robots/ravens.h"

#include "RavensRigidBodyAction.h"

/* Class to load a trajectory and play it back. **/
class jointPlayback {

	Scene &scene;									// Scene in which robot is
	Ravens::Ptr ravens;								// Ravens from the scene
	RavensRigidBodyGripperAction::Ptr lAct, rAct;	// Left and right gripper actions

	vector<int> joint_inds;		 					// Indices of joints to set DOF values
	vector<dReal> joint_vals;						// Latest joint values loaded from file.

	float freq;										// Frequency of recording
	float dt;										// Time step of simulation
	float currTime;									// Amount of time passed since last execution

	string filename;								// Name of file to read from
	ifstream file;									// Input file stream to read file

	bool enabled;									// Check if trajectory is currently playing back
	bool file_closed;								// Check if the file is closed

public:

	typedef boost::shared_ptr<jointPlayback> Ptr;

	// Constructor
	jointPlayback (	Scene &_scene, Ravens * _robot,
						float _freq = -1.0, float _dt = -1.0 ) :
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

	void setGripperActions (	RavensRigidBodyGripperAction * _lAct,
								RavensRigidBodyGripperAction * _rAct) {
		lAct.reset(_lAct);
		rAct.reset(_rAct);
	}

	void executeNextWaypoint () {
		if (!enabled)
			return;

		string line;
		float jval;

		// If next way point not loaded, load it.
		if (currTime >= 1/freq) {
			if (getline(file, line)) {
				if (line.c_str()[0] == 'l' || line.c_str()[0] == 'r') {
					cout<<"Different message found."<<endl;
					istringstream in(line);
					string arm; in >> arm;
					RavensRigidBodyGripperAction::Ptr gripAct = (arm == "l" ? lAct : rAct);

					string action; in >> action;
					if (action == "grab") {
						cout<<"Playback: Grabbing."<<endl;
						gripAct->grab(10);
					}
					else if (action == "release") {
						cout<<"Playback: Releasing."<<endl;
						gripAct->reset();
					}
				} else {
					istringstream in(line);
					joint_vals.clear();
					while (in >> jval) joint_vals.push_back(jval);
					ravens->ravens->setDOFValues(joint_inds, joint_vals);
					currTime = 0.0;
				}
			} else {
				file.close();
				enabled = false;
				file_closed = true;
			}
		} else currTime += dt;
	}

	~jointPlayback() {file.close();}

};

#endif
