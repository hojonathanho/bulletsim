#include "lfd/SegmentDemo.hpp"
#include "ScenePlayer.hpp"
#include "CustomScene.h"

#include <iostream>
#include <boost/shared_ptr.hpp>
#include <fstream>
#include <algorithm>
#include "lfd/utils_python.h"
#include <utils/colorize.h>

#include "lfd/TPSBijectWrapper.hpp"

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

ScenePlayer::ScenePlayer(CustomScene & _scene, float _freq, bool _doLFD, int numfile) :
										scene(_scene),
										freq(_freq),
										doLFD(_doLFD),
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
	larm_inds = scene.ravens.manipL->manip->GetArmIndices();
	rarm_inds = scene.ravens.manipR->manip->GetArmIndices();

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

	cout << "jtimes size : "<<  currentTrajSeg->jtimes.size() <<endl;
	cout << "joints size : "<<  currentTrajSeg->joints.size() <<endl;
	cout << "gtimes size : "<<  currentTrajSeg->gtimes.size() <<endl;
	cout << "grips size : " <<  currentTrajSeg->grips.size() <<endl;

	// sample joints at the correct freqeuncy.

	double tstart;
	if (currentTrajSeg->gtimes.size() != 0)
		tstart = min(currentTrajSeg->jtimes[0], currentTrajSeg->gtimes[0]);
	else
		tstart = currentTrajSeg->jtimes[0];

	double tend;
	if (currentTrajSeg->gtimes.size() != 0)
		tend = max(currentTrajSeg->jtimes.back(), currentTrajSeg->gtimes.back());
	else
		tend = currentTrajSeg->jtimes.back();

	genTimeStamps(tstart, tend, playTimeStamps);

	currentTimeStampIndex = 0;
	currentGripperActionIndex = 0;
	playbackStartTime = scene.getSimTime();


	if (doLFD) {// warp joints.

		vector<vector<btVector3> >  src_clouds;


		bool use_rope  = currentTrajSeg->ropePts.size() != 0;
		bool use_box   = currentTrajSeg->boxPts.size() != 0;
		bool use_hole  = currentTrajSeg->holePts.size() != 0;

		// get source clouds
		if (use_rope)
			src_clouds.push_back(currentTrajSeg->ropePts);
		if (use_box)
			src_clouds.push_back(currentTrajSeg->boxPts);
		if (use_hole)
			src_clouds.push_back(currentTrajSeg->holePts);


		// get target clouds
		vector<vector<btVector3> > target_clouds;
		if (use_rope) {
			vector<btVector3> target_rope;
			scene.sRope->getRopePoints(true, target_rope, 1.0/METERS);
			target_clouds.push_back(target_rope);
		}


		if (use_box) {
			vector<btVector3> target_box;
			scene.getBoxPoints(target_box, 1.0/METERS);
			target_clouds.push_back(target_box);
		}

		if (use_hole) {
			vector<btVector3> target_hole;
			scene.getBoxHoles(target_hole, 1.0/METERS);
			target_clouds.push_back(target_hole);
		}

		// warp the joints using LFD/ Trajopt
		vector<vector<double> > warpedJoints;
		warpRavenJointsBij(scene.ravens, src_clouds, target_clouds,
				currentTrajSeg->joints, warpedJoints);

		// interpolate the warped-joints at the play-backtimes
		rjoints = interpolateD( playTimeStamps, warpedJoints,currentTrajSeg->jtimes);

	} else {// just interpolate the recorded joints at the play-back time stamps
		rjoints = interpolateD(playTimeStamps, currentTrajSeg->joints, currentTrajSeg->jtimes);
	}
}

void extractJoints (const vector<int> &inds,
		const vector<dReal> &in_joint_vals, vector<dReal> &out_joint_vals) {
	out_joint_vals.clear();
	out_joint_vals.reserve(inds.size());
	for(int i=0; i<inds.size(); i+=1)
		out_joint_vals.push_back(in_joint_vals[inds[i]]);
}


// callback for playing back joints on the robot.
void ScenePlayer::playCallback() {
	if (playing) {
		if (doNextSegment())
			setupNewSegment();

		double playTimeSinceStart = playTimeStamps[currentTimeStampIndex] - playTimeStamps[0];
		double simTimeSinceStart  = scene.getSimTime() - playbackStartTime;

		if (playTimeSinceStart >= simTimeSinceStart) {

			double timestamp = playTimeStamps[currentTimeStampIndex];

			// set the robot-arms joints
			vector<double> larm_joints, rarm_joints;
			extractJoints(larm_inds, rjoints[currentTimeStampIndex], larm_joints);
			extractJoints(rarm_inds, rjoints[currentTimeStampIndex], rarm_joints);
			scene.ravens.setArmJointAngles(larm_joints, 'l');
			scene.ravens.setArmJointAngles(rarm_joints, 'r');

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
