#pragma once
#include "lfd/SegmentDemo.hpp"

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <fstream>

class CustomScene;


struct pointsToUse{
	bool use_rope;
	bool use_box;
	bool use_hole;
};


/** Plays the scene files as recorded in recorded/simruns by the class SceneRecorder. */
class ScenePlayer {

	// Scene in which the robot is
	CustomScene &scene;

	// ravens joint indices which are set by this player
	std::vector<int> larm_inds;
	std::vector<int> rarm_inds;

	// number of the scene file recording
	int runnumber;

	// frequency and time-period of setting the joints + gripper actions
	double freq;
	double tp;

	// File paths
	const std::string runnumfname;   // file which stores which demo to use as the reference.
	const std::string demolibfname;  // file which stores which demos are in the library of demos
	const std::string demoinfofname; // file for storing look-info for each demo.
	const std::string demodir; // directory where all the demos are stored
	std::string scenefname;

	Segmenter::Ptr tsegmenter;

	int segNum;
	vector<pointsToUse> lookModes;

	bool doLFD;
	bool playing;
	unsigned currentTimeStampIndex;
	unsigned currentGripperActionIndex;
	std::vector<double> playTimeStamps;
	trajSegment::Ptr currentTrajSeg;
	std::vector<std::vector<double> > rjoints;
	double playbackStartTime;

	// if we should look at all the demos in the library and
	// find the one which matches the current setting the most
	bool findClosestDemo;

	int getCurrentPlayNumber();


	/** Returns the demo-file number of the the demo which
	 *  matches the most to the current setting.
	 *
	 *  Note: Only the first segment of a demo in the library is used to calculate the cost.
	 *
	 *  This is done iff findClosestDemo is true. Else getCurrentPlayNumber is used. */
	int getClosestDemoNum();

	/** Returns a list of the demo numbers in the demo-lib file.*/
	std::vector<int> readDemoLibFile();

	/** Changes the lookModes variable to contain the look-information
	 *  (which point-clouds to use for registration), for the given demo file number.*/
	void loadDemoLookInfo(int demonum);

	void resetPlayer();
	/** Generate the time-stamps for joints. */
	void genTimeStamps(double startt, double endt, std::vector<double> &tstamps);

	/** Util functions to get the next trajectory segment. */
	bool inline doNextSegment();
	void setupNewSegment();

public :

	typedef boost::shared_ptr<ScenePlayer> Ptr;

	ScenePlayer(CustomScene & _scene, float _freq=100., bool doLFD=false, bool _findClosestDemo=true, int numfile=-1);
	// do LFD based warping or not
	void toggleLFD();
	// play the demo or not
	void togglePlay();

	// callback for playing back joints on the robot.
	void playCallback();
};
