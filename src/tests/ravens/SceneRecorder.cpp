#include "SceneRecorder.hpp"
#include <boost/bind.hpp>
#include "CustomScene.h"
#include <utils/colorize.h>

using namespace std;

int SceneRecorder::getAndUpdateRunNum() {

	string newfname = runnumfname + ".new";
	ifstream inpfile(runnumfname.c_str());
	ofstream newfile(newfname.c_str());
	unsigned int runnum;

	if(!inpfile.is_open()) {
		cout << "[ERROR : SceneRecorder] : Unable to open runum file : " << runnumfname << endl;
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
				newfile << line << endl;
				continue;
			} else {
				if(splitline.size() != 1) {
					cout << "[ERROR : SceneRecorder] : Unknown file format."<< endl;
					exit(-1);
				}
				runnum = atoi(splitline[0].c_str());
				newfile << (runnum+1);
			}
		}

		inpfile.close();
		newfile.close();
		remove(runnumfname.c_str());
		rename(newfname.c_str(), runnumfname.c_str());
		remove(newfname.c_str());
		return runnum;
	}
}

// puts a time-stamp on the message.
string SceneRecorder::stamp(string msg) {
	stringstream ss;
	ss << scene.getSimTime() << " : " << msg;
	return ss.str();
}


// Constructor
SceneRecorder::SceneRecorder(CustomScene &_scene, double _joints_freq, double _points_freq) :
					scene(_scene),
					joints_freq(_joints_freq),
					points_freq(_points_freq),
					recording(false),
					runfilepath(EXPAND(BULLETSIM_SRC_DIR)"/tests/ravens/recorded/simruns"),
					runnumfname(EXPAND(BULLETSIM_SRC_DIR)"/tests/ravens/recorded/runnum.txt") {

	// this is a fudge factor. The simulation runs about 4 times slower.
	// hence, we need to increase the frequency of logging 4 times.
	const float SIM_FACTOR = 4;

	if (joints_freq <= 0) joints_freq = SIM_FACTOR*100.;
	if (points_freq <= 0) points_freq = SIM_FACTOR*30.;

	joints_tp = 1./joints_freq;
	points_tp = 1./points_freq;

	scene.addPreStepCallback(boost::bind(&SceneRecorder::recordCallback, this));
}

// Adds message to file. Also adds a time stamp to it.
void SceneRecorder::addMessageToFile(string message, bool tstamp) {
	if (!recording) return;
	file << (tstamp? stamp(message) : message);
	file.flush();
}


/* Callback which opens file, stores latest joint values, closes file.
 * Does so only when the time since last check exceeds time period of checks.
 *
 * Must be added to the scene's list of callbacks. */
void SceneRecorder::recordCallback () {

	if (recording) {

		double nowTime = scene.getSimTime();

		if ((nowTime - pLastMsgTime)>= points_tp) {
			addMessageToFile(scene.getPointsMessage());
			pLastMsgTime = nowTime;
		}

		if ((nowTime - jLastMsgTime)>= joints_tp) {
			addMessageToFile(scene.getJointsMessage());
			jLastMsgTime = nowTime;
		}
	}
}


// Toggles file and opens/closes file appropriately
void SceneRecorder::toggleRecording () {
	recording = not recording;

	// display a message on the terminal
	stringstream rec_msg;
	rec_msg << "Recording: " << (recording ? "true" : "false");
	cout << colorize(rec_msg.str(), "red", "true")<<endl;

	if (recording) {
		// open a new file for recording.
		stringstream fnamess;
		fnamess << runfilepath << "/run" << getAndUpdateRunNum() << ".txt";
		currentSceneFile = fnamess.str();
		file.open(currentSceneFile.c_str(), ios::out | ios::app);

		// record current points and joints
		pLastMsgTime = scene.getSimTime();
		jLastMsgTime = scene.getSimTime();
		addMessageToFile("look\n");
		addMessageToFile(scene.getPointsMessage());
		addMessageToFile(scene.getJointsMessage());
	} else {
		cout << colorize(string("Scene saved to file : ") + currentSceneFile, "green", "True")<<endl;
		file.close();
	}
}
