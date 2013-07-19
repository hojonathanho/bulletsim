#include "lfd/SegmentDemo.hpp"
#include "ScenePlayer.hpp"
#include "CustomScene.h"

#include <iostream>
#include <boost/shared_ptr.hpp>
#include <fstream>
#include <algorithm>
#include "lfd/utils_python.h"
#include <utils/colorize.h>

using namespace std;

int ScenePlayer::getCurrentPlayNumber() {
	ifstream inpfile(runnumfname.c_str());
	unsigned int runnum;

	if(!inpfile.is_open()) {
		cout << "[ERROR : ScenePlayer] : Unable to open run-num file : " << runnumfname << endl;
		exit(-1);
	} else {
		while(!inpfile.eof()) {
			string line;
			getline(inpfile,line);
			vector<string> splitline;
			string buf;
			stringstream ss(line);

			while (ss >> buf)  // extract white-space separated strings on a line
				splitline.push_back(buf);

			if (splitline.size()==0 || splitline[0][0] == '#') { // skip blank lines and comments
				continue;
			} else {
				if(splitline.size() != 1) {
					cout << "[ERROR : SceneRecorder] : Unknown file format."<< endl;
					exit(-1);
				}
				runnum = atoi(splitline[0].c_str());
			}
		}

		inpfile.close();
		return runnum;
	}
}

void ScenePlayer::resetPlayer() {
	playing = false;

	// create a new trajectory-segmenter:
	tsegmenter.reset(new Segmenter(scenefname));

	currentTimeStampIndex = 0;
	currentGripperActionIndex = 0;

	playTimeStamps.clear();
	currentTrajSeg.reset();
	rjoints.clear();
}

/** Generate the time-stamps for joints. */
void ScenePlayer::genTimeStamps(double startt, double endt, vector<double> &tstamps) {
	tstamps.clear();
	double laststamp = startt;
	do {
		tstamps.push_back(laststamp);
		laststamp += tp;
	} while(laststamp <= endt);
	if (laststamp < endt)
		tstamps.push_back(endt);
}

ScenePlayer::ScenePlayer(CustomScene & _scene, float _freq, int numfile) :
								scene(_scene),
								freq(_freq),
								doLFD(false),
								playing(false),
								currentTimeStampIndex(-1.),
								runnumfname(string(EXPAND(BULLETSIM_SRC_DIR)) + "/tests/ravens/recorded/playrunnum.txt") {
	if (numfile < 0)
		numfile = getCurrentPlayNumber();

	stringstream scenefnamess;
	scenefnamess << EXPAND(BULLETSIM_SRC_DIR)"/tests/ravens/recorded/simruns/run" << numfile << ".txt";
	scenefname  = scenefnamess.str();

	// set the play-back frequency
	if (freq < 0)
		freq = 100.;
	tp  = 1./freq;

	// the dofs of the robot which are set
	int dof = scene.ravens.ravens->robot->GetDOF();
	for (int i = 0; i < dof; ++i) joint_inds.push_back(i);

	// register a callback with the scene's step
	scene.addPreStepCallback(boost::bind(&ScenePlayer::playCallback, this));

	//resetPlayer();
}

// do LFD based warping or not
void ScenePlayer::toggleLFD() {
	doLFD = not doLFD;
}

// play the demo or not
void ScenePlayer::togglePlay() {
	resetPlayer();
	playing = not playing;
	if (playing)
		cout << colorize("Now playing demo file: " + scenefname, "green", true)<<endl;
}


bool inline ScenePlayer::doNextSegment() {
	return (playTimeStamps.size() ==0 or currentTimeStampIndex==playTimeStamps.size());
}

void ScenePlayer::setupNewSegment() {
	currentTrajSeg = tsegmenter->getNextSegment();

	// stop play-back if no more segments
	if (not currentTrajSeg) {
		playing = false;
		resetPlayer();
		cout << colorize("Done playing demo file: " + scenefname, "green", true)<<endl;
		return;
	}

	// sample joints at the correct freqeuncy.
	double tstart = min(currentTrajSeg->jtimes[0], currentTrajSeg->gtimes[0]);
	double tend   = max(currentTrajSeg->jtimes.back(), currentTrajSeg->gtimes.back());
	genTimeStamps(tstart, tend, playTimeStamps);
	currentTimeStampIndex = 0;
	currentGripperActionIndex = 0;
	playbackStartTime = scene.getSimTime();

	if (doLFD) {// warp joints.

		vector<vector<btVector3> >  src_cloud;

		bool use_rope  = currentTrajSeg->ropePts.size() != 0;
		bool use_box   = currentTrajSeg->boxPts.size() != 0;
		bool use_hole  = currentTrajSeg->holePts.size() != 0;

		// get source clouds
		if (use_rope)
			src_cloud.push_back(currentTrajSeg->ropePts);
		if (use_box)
			src_cloud.push_back(currentTrajSeg->boxPts);
		if (use_hole)
			src_cloud.push_back(currentTrajSeg->holePts);

		// get target clouds
		vector<vector<btVector3> > target_cloud;
		if (use_rope) {
			vector<btVector3> target_rope;
			scene.getBoxHoles(target_rope, 1.0/METERS);
			target_cloud.push_back(target_rope);
		}
		if (use_box) {
			vector<btVector3> target_box;
			scene.getBoxHoles(target_box, 1.0/METERS);
			target_cloud.push_back(target_box);
		}
		if (use_hole) {
			vector<btVector3> target_hole;
			scene.getBoxHoles(target_hole, 1.0/METERS);
			target_cloud.push_back(target_hole);
		}

		// warp the joints using LFD/ Trajopt
		vector<vector<double> > warpedJoints;
		//		warpJointsLFD(src_clouds, target_clouds,
		//				currentTrajSeg->joints, warpedJoints);

		// interpolate the warped-joints at the play-backtimes
		rjoints = interpolateD( playTimeStamps, warpedJoints,currentTrajSeg->jtimes);

	} else {// just interpolate the recorded joints at the play-back time stamps
		rjoints = interpolateD(playTimeStamps, currentTrajSeg->joints, currentTrajSeg->jtimes);
	}
}


// callback for playing back joints on the robot.
void ScenePlayer::playCallback() {
	if (playing) {
		if (doNextSegment())
			setupNewSegment();

		double playTimeSinceStart = playTimeStamps[currentTimeStampIndex] - playTimeStamps[0];
		double simTimeSinceStart  = scene.getSimTime() - playbackStartTime;

		if (playTimeSinceStart >= simTimeSinceStart) {

			// set the robot joints
			double timestamp = playTimeStamps[currentTimeStampIndex];
			scene.ravens.ravens->setDOFValues(joint_inds, rjoints[currentTimeStampIndex]);
			currentTimeStampIndex += 1;

			// set the robot gripper actions
			if (currentGripperActionIndex < currentTrajSeg->grips.size() and currentTrajSeg->gtimes[currentGripperActionIndex] <= timestamp) {
				GripperAction gaction = currentTrajSeg->grips[currentGripperActionIndex];

				currentGripperActionIndex += 1;

				if (gaction == RELEASE_R)
					scene.openGrippers("r");
				else if (gaction == RELEASE_L)
					scene.openGrippers("l");
				else if (gaction == GRAB_R)
					scene.closeGrippers("r");
				else if (gaction == GRAB_L)
					scene.closeGrippers("l");
			}
		}
	}
}
