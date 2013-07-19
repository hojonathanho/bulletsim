#pragma once
#include "lfd/SegmentDemo.hpp"

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <fstream>

class CustomScene;

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

	// File directory
	std::string runnumfname;
	std::string scenefname;

	Segmenter::Ptr tsegmenter;

	bool doLFD;
	bool playing;
	unsigned currentTimeStampIndex;
	unsigned currentGripperActionIndex;
	std::vector<double> playTimeStamps;
	trajSegment::Ptr currentTrajSeg;
	std::vector<std::vector<double> > rjoints;
	double playbackStartTime;


	int getCurrentPlayNumber();

	void resetPlayer();
	/** Generate the time-stamps for joints. */
	void genTimeStamps(double startt, double endt, std::vector<double> &tstamps);

	/** Util functions to get the next trajectory segment. */
	bool inline doNextSegment();
	void setupNewSegment();

public:

	typedef boost::shared_ptr<ScenePlayer> Ptr;

	ScenePlayer(CustomScene & _scene, float _freq=100., bool doLFD=false, int numfile=-1);
	// do LFD based warping or not
	void toggleLFD();
	// play the demo or not
	void togglePlay();

	// callback for playing back joints on the robot.
	void playCallback();
};
