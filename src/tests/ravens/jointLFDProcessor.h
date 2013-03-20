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

	bool found_rope;
	vector<btVector3> rope_points;
	bool found_needle;
	vector<btVector3> needle_points;
	bool found_box;
	vector<btVector3> box_points;
	bool found_hole;
	vector<btVector3> hole_points;

	vector< pair<int, string> > grabIndices;
	vector< pair<int, string> > releaseIndices;

	LFDProcessor (string _inputFile = "/home/ankush/sandbox/bulletsim/src/tests/ravens/recorded/raven_joints.txt") :
						   jointFile (_inputFile), fileClosed(false),
						   found_rope (false), found_needle (false), found_box (false), found_hole (false){}

	void setInputFile (string _inputFile) {jointFile = _inputFile;}

	void initProcessing () {
		iFS.open(jointFile.c_str(), ios::in);
		string line; getline(iFS, line);
		istringstream in(line); string section; in >> section;
		// File always starts with section
		assert (section == "section");
		fileClosed = false;
	}

	bool preProcess (Ravens &ravens, vector<btVector3> new_rope_points, vector< vector <double> > & processedJointValues) {
		if (fileClosed) {
			cout<<"File has ended."<<endl;
			return false;
		}

		jointValueVector.clear();
		grabIndices.clear();
		releaseIndices.clear();
		found_rope = found_needle = found_box = found_hole = false;

		string line;

		///////////////////////////////////

		fileClosed = true;
		while (getline(iFS, line)) {
			istringstream in(line);
			string command; in >> command;

			if (command == "rope") {
				found_rope = true;
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
				found_needle = true;
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
				found_box = true;
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
				found_hole = true;
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

		bool successful = warpRavenJoints (ravens, rope_points, new_rope_points, jointValueVector, processedJointValues);

		//Actually should be:
		/* warpRavenJoints (ravens, make_pair(found_rope, make_pair(rope_points, new_rope_points)),
									make_pair(found_needle, make_pair(needle_points, new_needle_points)),
									make_pair(found_box, make_pair(box_points, new_box_points)),
									make_pair(found_hole, make_pair(hole_points, new_hole_points)),
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
