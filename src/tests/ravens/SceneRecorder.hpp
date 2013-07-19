#pragma once

#include <string>
#include <fstream>
#include <vector>
#include <stdio.h>
#include <boost/shared_ptr.hpp>
#include "ravens_config.h"

using namespace std;

class CustomScene;

/** Class to record joint values/ scene information and store them to a file. */
class SceneRecorder {

	CustomScene &scene;					// Scene in which the robot is

	vector<double> joint_vals;			// Vector to store joint values
	bool recording;						// Check if recording currently

	const string runfilepath;	 		// File directory
	ofstream file;						// Output file stream

	double jLastMsgTime;				// Time stamp when the last JOINT message was recorded
	double pLastMsgTime;				// Time stamp when the last POINT message was recorded

	double joints_freq;					// Frequency of recording JOINTS
	double joints_tp;					// Time period of recording the robot's joints = 1/joints_freq

	double points_freq;					// Frequency of recording POINTS
	double points_tp;					// Time period of recording scene POINTS = 1/points_freq

	const string runnumfname;
	string currentSceneFile;

	// this function returns the current simulation run number and updates the counter (stored in a file).
	int getAndUpdateRunNum();

	// puts a time-stamp on the message.
	string stamp(string msg);

public:

	typedef boost::shared_ptr<SceneRecorder> Ptr;

	// Constructor
	SceneRecorder(CustomScene &_scene, double _joints_freq = -1, double _points_freq=-1);


	/* Callback which opens file, stores latest joint values, closes file.
	 * Does so only when the time since last check exceeds time period of checks.
	 *
	 * Must be added to the scene's list of callbacks. */
	void recordCallback ();

	// Toggles file and opens/closes file appropriately
	void toggleRecording ();

	// Adds message to file. Also adds a time stamp to it.
	void addMessageToFile(string message, bool tstamp=true);

	~SceneRecorder () { cout<<"Done recording.\n"; file.close();}
};
