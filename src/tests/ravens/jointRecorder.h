// TODO: Change this to record joint values at appropriate time-steps w.r.t scene time steps.
// TODO: Have command line options to set name of file
// TODO: Record only a subset of the joint-angles, maybe

#ifndef __JOINT_RECORDER__
#define __JOINT_RECORDER__

#include <string>
#include <fstream>
#include <vector>

#include "simulation/simplescene.h"
#include "robots/ravens.h"
#include "ravens_config.h"

/** Class to record joint values and store them to a file. */
class jointRecorder {

	Scene &scene;						// Scene in which robot is

	Ravens::Ptr ravens;					// Ravens from the scene
	std::vector<dReal> joint_vals;		// Vector to store joint values
	bool recording;						// Check if recording currently
	float record_freq;					// Frequency of recording

	std::string filename; 				// File name
	ofstream file;						// Output file stream
	float currTime;						// Time since last time joint messages were stored
	float dt;							// Time step to move by

public:

	typedef boost::shared_ptr<jointRecorder> Ptr;

	// Constructor
	jointRecorder (Scene &_scene, Ravens * _robot, float _dt = -1, float _record_freq = -1.0) :
		scene (_scene), ravens(_robot), recording(false), dt(_dt), currTime(0.0), record_freq(_record_freq) {

		filename = "/home/ankush/sandbox/bulletsim/src/tests/ravens/recorded/raven_joints.txt";

		if (dt == -1)
			dt = BulletConfig::dt;

		if (record_freq == -1.0)
			record_freq = RavenConfig::record_freq;

		scene.addPreStepCallback(boost::bind(&jointRecorder::recordCallback, this));
	}

	/* Callback which opens file, stores latest joint values, closes file.
	 * Does so only when the time since last check exceeds time period of checks.
	 *
	 * Must be added to the scene's list of callbacks. */
	void recordCallback () {

		if (currTime >= 1.0/record_freq && recording) {

			joint_vals.clear();
			ravens->ravens->robot->GetDOFValues(joint_vals);
			int jsize = joint_vals.size();

			for (int i = 0; i < jsize; ++i)
				file << joint_vals[i] << " ";
			file << "\n";
			file.flush();

			currTime = 0.0;

		} else if (recording) currTime += dt;
	}

	// Toggles file and opens/closes file appropriately
	void toggleRecording () {
		recording = !recording;
		std::cout << "Recording: " << (recording ? "true" : "false") << std::endl;
		if (recording)
			file.open(filename.c_str(), ios::out | ios::app);
		else {
			file.close();
			currTime = 0.0;
		}
	}

	// Function to get frequency of recording
	float getRecordingFrequency () {return record_freq;}

	~jointRecorder () { std::cout<<"Done recording.\n"; file.close();}
};

#endif
