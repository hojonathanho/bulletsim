#include <vector>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <string>
#include <fstream>
#include <utils/colorize.h>

// string operations
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/predicate.hpp>


using namespace std;
using namespace Eigen;

enum GripperAction{NONE, R_OPEN, R_CLOSE, L_OPEN, L_CLOSE};


/** Structure to hold the robot gripper action at a given time-stamp.
 *  An implicit assumption is that there can only be one of
 *  the above five gripper actions possible per time-stamp. */
struct robotInfo {
	typedef boost::shared_ptr<robotInfo> Ptr;

	GripperAction gaction;
	vector<double> joints;
};

/** A segment of the demo trajectory : joints + env data b/w two looks. **/
struct trajSegment {
	typedef boost::shared_ptr<trajSegment> Ptr;

	vector<float> times;
	vector<robotInfo> rInfo;
	vector<Vector3f> ropePts;
	vector<Vector3f> boxPts;
	vector<Vector3f> holePts;
};

/** Just finds a prompt in a vector of strings. */
int getPromptIndex(vector<string> ss) {
	if (ss.size() > 0) {
		int i=0;
		do {
			if (ss[i] == ":") {
				return i;
			}
			i++;
		} while (i < ss.size());
		if (i==ss.size()) {return -1;}
	} else {
		return -1;
	}
}

/** Trivial predicates to resolve the type of line. */
bool inline isPointCloudLine(const string &line) {
	return boost::starts_with(line, "\t") && !boost::starts_with(line, "\t\t");
}

bool inline isPointLine(const string &line) {
	return boost::starts_with(line, "\t\t");
}


/** Reads a point-cloud into tsef from file. */
void readPoints(ifstream &file, trajSegment::Ptr tseg) {
	vector<Vector3f> * currentCloud;
	bool gotCurrentCloud = false;
	string line;


	while(not ( file.eof() or boost::contains(line, "end-points")) ) {
		getline(file, line);

		if (isPointCloudLine(line)) {
			string cloud_type;
			stringstream ss(line);
			ss >> cloud_type;

			if (cloud_type == "rope") {
				currentCloud = &tseg->ropePts;
			}
			else if (cloud_type == "box") {
				currentCloud = &tseg->boxPts;
			}
			else if (cloud_type == "hole") {
				currentCloud = &tseg->holePts;
			}
			else {
				cout << "[ERROR : Trajectory Segmentation] : Unknown point-cloud type : "<< cloud_type<<endl;
				exit(-1);
			}
			if (!gotCurrentCloud) gotCurrentCloud = true;
		}  else if (isPointLine(line)) {
			if (gotCurrentCloud) {
				string sx, sy, sz;
				stringstream coords(line);
				coords >> sx >> sy >> sz;
				Vector3f pt(boost::lexical_cast<float>(sx), boost::lexical_cast<float>(sy), boost::lexical_cast<float>(sz));
				currentCloud->push_back(pt);
			} else {
				cout << "[ERROR : Trajectory Segmentation] : Got point before cloud type."<<endl;
			}
		}
	}
}

/** Returns the command in the data-file. */
string inline getCommand(const vector<string> &splitline) {
	int prompt_idx = getPromptIndex(splitline);
	if (prompt_idx == -1) {
		cout << "[ERROR : Trajectory Segmentation] : Got bad command-type line in the file with no prompt.\n";
		exit(-1);
	}
	return splitline[prompt_idx+1];
}

/** Returns the time-stamp on a command in the date-file. */
float inline getTimeStamp(const vector<string> &splitline) {
	return boost::lexical_cast<float>(splitline[0]);
}


/** Parse the demo data file to generate segment information. */
vector<trajSegment> getTrajSegments(string fpath) {
	ifstream demofile(fpath.c_str());
	if(!demofile.is_open()) {
		stringstream errss;
		errss << "[ERROR : getTrajSegments] Unable to open demo file : " << fpath;
		cout << colorize(errss.str(), "red", true);
		exit(-1);
	}

	vector<trajSegment::Ptr> segments;
	int numSegments = 0;
	bool firstDone  = false;
	bool inPoints   = false;
	bool skipPoints = false;
	float lastTimestamp = -1;
	unsigned rinfoIndex = -1;

	trajSegment::Ptr newSegment(new trajSegment);


	while(!demofile.eof()) {
		string line;
		getline(demofile, line);

		vector<string> splitline;
		string buf;
		stringstream ss(line);
		while(ss >> buf) splitline.push_back(buf);

		if (splitline.size()==0 || splitline[0][0] == '#') // skip blank lines and comments
			continue;

		// read-points
		if (inPoints && !skipPoints) {
			readPoints(demofile, newSegment);
			skipPoints = true;
			inPoints   = false;
		}
		else if (inPoints && skipPoints) {
			trajSegment::Ptr tempSegment(new trajSegment);
			readPoints(demofile, tempSegment);
			inPoints = false;
		}
		else {// it is some command, where command is of the format: time-step : {look, grab/release {r,l}, joints, points}
			string cmd = getCommand(splitline);

			if (cmd == "look") {
				if (not firstDone) {// special case for first traj segment
					firstDone = true;
				} else {// start a new segment at the occurence of a LOOK command.
					segments.push_back(newSegment);
					newSegment.reset(new trajSegment);
				}
			} else if (cmd == "joints") {
				float timestamp = getTimeStamp(splitline);
				if (timestamp == lastTimeStamp) {
					add to the robot-info
				} else {
					create a new robot info
					update rinfoindex
				}
			} else if (cmd == "") {}

		}

	}
}

