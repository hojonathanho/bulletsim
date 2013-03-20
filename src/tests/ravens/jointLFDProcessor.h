#ifndef __LFD_PROCESSOR__
#define __LFD_PROCESSOR__

#include <assert.h>
#include "lfd/RavensLfd.h"

class LFDProcessor {

	string jointFile;
	ifstream iFS;
	bool fileClosed;

public:

	typedef boost::shared_ptr<LFDProcessor> Ptr;

	vector< vector<double> > jointValueVector;

	bool use_rope;
	vector<btVector3> rope_points;
	bool use_needle;
	vector<btVector3> needle_points;
	bool use_box;
	vector<btVector3> box_points;
	bool use_hole;
	vector<btVector3> hole_points;

	unsigned int mode_count;
	vector<string> modes;

	vector< pair<int, string> > grabIndices;
	vector< pair<int, string> > releaseIndices;

	LFDProcessor (string _inputFile = "/home/ankush/sandbox/bulletsim/src/tests/ravens/recorded/raven_joints.txt") :
						   jointFile (_inputFile), fileClosed(false),
						   use_rope (false), use_needle (false), use_box (false), use_hole (false){}

	void setInputFile (const string _inputFile) {jointFile = _inputFile;}

	void initProcessing () {
		iFS.open(jointFile.c_str(), ios::in);
		string line; getline(iFS, line);
		istringstream in(line); string section; in >> section;
		// File always starts with section
		assert (section == "section");
		fileClosed = false;
		mode_count = 0;
	}

	void initializeModes (const vector<string> & _modes) {modes = _modes;}

	bool preProcess (	Ravens &ravens,
						vector<btVector3> new_rope_points, ////
						vector< vector <double> > & processedJointValues) {
		if (fileClosed) {
			cout<<"File has ended."<<endl;
			return false;
		}

		jointValueVector.clear();
		grabIndices.clear();
		releaseIndices.clear();
		use_rope = use_needle = use_box = use_hole = false;

		string line;

		///////////////////////////////////

		fileClosed = true;
		while (getline(iFS, line)) {
			istringstream in(line);
			string command; in >> command;

			if (command == "rope") {
				use_rope = true;
				vector<float> vals;
				string val;
				while (in >> val) {
					if (val == "|") {
						btVector3 point(vals[0], vals[1], vals[2]);
						rope_points.push_back(point);
						vals.clear();
					} else {
						float rVal = atof(val.c_str());
						vals.push_back(rVal);
					}
				}
			} else if (command == "needle") {
				use_needle = true;
				vector<float> vals;
				string val;
				while (in >> val) {
					if (val == "|") {
						btVector3 point(vals[0], vals[1], vals[2]);
						needle_points.push_back(point);
						vals.clear();
					} else {
						float rVal = atof(val.c_str());
						vals.push_back(rVal);
					}
				}
			} else if (command == "box") {
				use_box = true;
				vector<float> vals;
				string val;
				while (in >> val) {
					if (val == "|") {
						btVector3 point(vals[0], vals[1], vals[2]);
						box_points.push_back(point);
						vals.clear();
					} else {
						float rVal = atof(val.c_str());
						vals.push_back(rVal);
					}
				}
			} else if (command == "hole") {
				use_hole = true;
				vector<float> vals;
				string val;
				while (in >> val) {
					if (val == "|") {
						btVector3 point(vals[0], vals[1], vals[2]);
						hole_points.push_back(point);
						vals.clear();
					} else {
						float rVal = atof(val.c_str());
						vals.push_back(rVal);
					}
				}
			} else if (command == "section") 	{
				fileClosed = false;
				break;
			} else if (command == "grab") 	{
				string arm; in >> arm;
				grabIndices.push_back(make_pair (jointValueVector.size() - 1, arm));
			} else if (command == "release") {
				string arm; in >> arm;
				releaseIndices.push_back(make_pair (jointValueVector.size() - 1, arm));
			} else if (command == "joints") {
				vector<double> jointVals; double jval;
				while (in >> jval) jointVals.push_back(jval);
				jointValueVector.push_back(jointVals);
			}
		}

		if (fileClosed) {iFS.close();}

		if (mode_count < modes.size()) {
			if (modes[mode_count] == "knot") {

			} else if (modes[mode_count] == "flap") {

			}
		}

		mode_count ++;


		bool successful = warpRavenJoints (ravens, rope_points, new_rope_points, jointValueVector, processedJointValues);

		//Actually should be:
		/* bool successful = warpRavenJoints (	ravens, make_pair(use_rope, make_pair(rope_points, new_rope_points)),
												make_pair(use_needle, make_pair(needle_points, new_needle_points)),
												make_pair(use_box, make_pair(box_points, new_box_points)),
												make_pair(use_hole, make_pair(hole_points, new_hole_points)),
												jointValueVector, processedJointValues);*/

		if (!successful) {
			cout<<"Unable to find trajectory."<<endl;
			iFS.close(); fileClosed = true;
		}
		return successful;
	}

	void reset () {iFS.open(jointFile.c_str(), ios::in); fileClosed=false;}

	~LFDProcessor () {iFS.close();}
};

#endif
