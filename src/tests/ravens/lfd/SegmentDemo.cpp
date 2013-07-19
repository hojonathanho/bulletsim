#include "SegmentDemo.hpp"
#include <iostream>
// string operations
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/predicate.hpp>

using namespace std;
using namespace Eigen;

/** Just finds a prompt in a vector of strings. */
int Segmenter::getPromptIndex(vector<string> ss) {
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
bool inline Segmenter::isPointCloudLine(const string &line) {
	return boost::starts_with(line, "\t") && !boost::starts_with(line, "\t\t");
}

bool inline Segmenter::isPointLine(const string &line) {
	return boost::starts_with(line, "\t\t");
}

/** Reads a point-cloud into tseg from file. */
void Segmenter::readPoints(ifstream &file, trajSegment::Ptr tseg) {
	vector<btVector3> * currentCloud;
	bool gotCurrentCloud = false;
	string line;

	while(not ( file.eof() or boost::contains(line, "end-points")) ) {
		getline(file, line);

		vector<string> splitline;
		string buf;
		stringstream ss(line);
		while(ss >> buf) splitline.push_back(buf);

		if (splitline.size()==0 || splitline[0][0] == '#') // skip blank lines and comments
			continue;

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
				btVector3 pt(boost::lexical_cast<float>(sx), boost::lexical_cast<float>(sy), boost::lexical_cast<float>(sz));
				currentCloud->push_back(pt);
			} else {
				cout << "[ERROR : Trajectory Segmentation] : Got point before cloud type."<<endl;
				exit(-1);
			}
		}
	}
}

/** Returns the command in the data-file. */
string inline Segmenter::getCommand(const vector<string> &splitline) {
	int prompt_idx = getPromptIndex(splitline);
	if (prompt_idx == -1) {
		cout << "[ERROR : Trajectory Segmentation] : Got bad command-type line in the file with no prompt.\n";
		exit(-1);
	}
	return splitline[prompt_idx+1];
}

/** Returns the time-stamp on a command in the date-file. */
double inline Segmenter::getTimeStamp(const vector<string> &splitline) {
	return boost::lexical_cast<double>(splitline[0]);
}


Segmenter::Segmenter(string fpath) : demofile(fpath.c_str()), firstSegmentDone(false) {
	if(!demofile.is_open()) {
		stringstream errss;
		errss << "[ERROR : Traj Segmenter] Unable to open demo file : " << fpath;
		cout << errss.str() <<endl;
		exit(-1);
	}
}

/** Parse the demo data file to generate segment information. */
trajSegment::Ptr Segmenter::getNextSegment() {

	if (not demofile.is_open() or demofile.eof())
		return trajSegment::Ptr();

	bool skipPoints = false;

	trajSegment::Ptr tseg(new trajSegment);

	// find the first "look" command in the file
	if (not firstSegmentDone) {
		bool gotFirstLook = false;
		while(not demofile.eof() and not gotFirstLook) {
			string line;
			getline(demofile, line);
			gotFirstLook = boost::contains(line, "look");
		}
		firstSegmentDone = true;
	}

	while(!demofile.eof()) {
		string line;
		getline(demofile, line);

		vector<string> splitline;
		string buf;
		stringstream ss(line);
		while(ss >> buf) splitline.push_back(buf);

		if (splitline.size()==0 || splitline[0][0] == '#') // skip blank lines and comments
			continue;

		string cmd = getCommand(splitline);
		double timestamp = getTimeStamp(splitline);

		if (cmd == "look") {
			return tseg;
		} else if (cmd == "points") {
			if (not skipPoints) {
				readPoints(demofile, tseg);
				skipPoints = true;
			} else {
				trajSegment::Ptr tempSegment(new trajSegment);
				readPoints(demofile, tempSegment);
			}
		} else if (cmd == "joints") {
			tseg->jtimes.push_back(timestamp);
			const int numjoints = splitline.size() - 4;
			vector<double> tjoints(numjoints);
			for (int k=0; k < numjoints; k+=1)
				tjoints[k] = boost::lexical_cast<double>(splitline[k+4]);
			tseg->joints.push_back(tjoints);
		} else if (cmd == "grab") {
			tseg->gtimes.push_back(timestamp);
			tseg->grips.push_back((splitline[3]=="l")? GRAB_L : GRAB_R);
		} else if (cmd == "release") {
			tseg->gtimes.push_back(timestamp);
			tseg->grips.push_back((splitline[3]=="l")? RELEASE_L : RELEASE_R);
		} else {
			cout << "[WARN : Traj Segmenter] : Unknown command "<<cmd << ", skipping.\n";
		}
	}
	return tseg;
}
