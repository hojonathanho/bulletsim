// TODO: Change this to record joint values at appropriate time-steps w.r.t scene time steps.
// TODO: Have command line options to set name of file
// TODO: Record only a subset of the joint-angles, maybe

#ifndef __JOINT_RECORDER__
#define __JOINT_RECORDER__

#include <string>
#include <fstream>
#include <vector>

#include "ravens_config.h"

using namespace std;

class CustomScene;

/** Class to record joint values/ scene information and store them to a file. */
class jointRecorder {

	CustomScene &scene;					// Scene in which robot is

	vector<double> joint_vals;			// Vector to store joint values
	bool recording;						// Check if recording currently
	float record_freq;					// Frequency of recording
	bool init;

	string filename;	 				// File name
	ofstream file;						// Output file stream
	float currTime;						// Time since last time joint messages were stored
	float dt;							// Time step to move by

public:

	typedef boost::shared_ptr<jointRecorder> Ptr;

	// Constructor
	jointRecorder (CustomScene &_scene, float _dt = -1, float _record_freq = -1.0,
				   string _filename="/home/sibi/sandbox/bulletsim/src/tests/ravens/recorded/raven_joints.txt");

	/* Callback which opens file, stores latest joint values, closes file.
	 * Does so only when the time since last check exceeds time period of checks.
	 *
	 * Must be added to the scene's list of callbacks. */
	void recordCallback ();


	// Toggles file and opens/closes file appropriately
	void toggleRecording ();

	// Adds message to file
	void addMessageToFile(string message) {
		if (!recording) return;
		file << message << "\n";
		file.flush();
	}

	// Function to get frequency of recording
	float getRecordingFrequency () {return record_freq;}

	~jointRecorder () { cout<<"Done recording.\n"; file.close();}
};

#endif
