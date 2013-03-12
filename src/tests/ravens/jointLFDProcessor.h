#ifndef __LFD_PROCESSOR__
#define __LFD_PROCESSOR__

#include <assert.h>
#include "robots/ravens.h"

//bool lfdProcess2 (vector< vector <float> > & initJoints, vector<btVector3> & src_points, vector<btVector3> & ropePoints, vector< vector<float> > & outJoints) {return true;}

class LFDProcessor {

	string jointFile;
	ifstream iFS;
	bool fileClosed;

public:

	typedef boost::shared_ptr<LFDProcessor> Ptr;

	vector< vector<double> > jointValueVector;
	vector<btVector3> src_points;
	vector< pair<int, string> > grabIndices;
	vector< pair<int, string> > releaseIndices;

	LFDProcessor (string _inputFile = "/home/sibi/sandbox/bulletsim/src/tests/ravens/recorded/raven_joints.txt") :
						   jointFile (_inputFile), fileClosed(false) {}

	void setInputFile (string _inputFile) {jointFile = _inputFile;}

	void initProcessing () {
		iFS.open(jointFile.c_str(), ios::in);
		string line; getline(iFS, line);
		istringstream in(line); string section; in >> section;
		// File always starts with section
		assert (section == "section");
	}
	/*
	void process () {
		iFS.open(jointFile.c_str(), ios::in);
		oFS.open(outFile.c_str(), ios::out);

		string line;

		while (getline(iFS, line)) {
			istringstream in(line);
			if (isalpha(line.c_str()[0])) {
				string command; in >> command;

				if (command == "grab" || command == "release") {
					string arm;	in >> arm;
					oFS << command << " " << arm << "\n";
					oFS.flush();
				} else if (command == "rope") {
					src_points.clear();
					vector<float> rVals;
					string val;

					while (in >> val) {
						if (val == "|") {
							btVector3 point(rVals[0], rVals[1], rVals[2]);
							src_points.push_back(point);
							rVals.clear();
						} else {
							float rVal = atof(val.c_str());
							rVals.push_back(rVal);
						}
					}
				}
			} else {
				vector<float> jointVals; float jval;
				while (in >> jval) jointVals.push_back(jval);

				vector<float> finalJoints = lfdProcess (jointVals, src_points);

				int jsize = jointVals.size();

				for (int i = 0; i < jsize; ++i)
					oFS << jointVals[i] << " ";
				oFS << "\n";
				oFS.flush();
			}
		}

		cout<<"Joint file has been processed for new scene!"<<endl;
	}*/

	bool preProcess (Ravens &ravens, vector<btVector3> ropePoints, vector< vector <double> > & processedJointValues) {
		if (fileClosed) {
			cout<<"File has ended."<<endl;
			return false;
		}
		jointValueVector.clear();

		string line;

		// Assuming we always start at rope.
		// Get rope values out now.
		src_points.clear();
		getline(iFS, line);
		istringstream ropeStream(line);
		string ropeStr; ropeStream >> ropeStr; assert (ropeStr == "rope");

		vector<float> rVals;
		string val;
		while (ropeStream >> val) {
			if (val == "|") {
				btVector3 point(rVals[0], rVals[1], rVals[2]);
				src_points.push_back(point);
				rVals.clear();
			} else {
				float rVal = atof(val.c_str());
				rVals.push_back(rVal);
			}
		}

		///////////////////////////////////

		fileClosed = true;
		while (getline(iFS, line)) {
			istringstream in(line);
			if (isalpha(line.c_str()[0])) {
				string command; in >> command;

				if (command == "section") 	{fileClosed = false; break;}
				if (command == "grab") 	{
					string arm; in >> arm;
					grabIndices.push_back(make_pair (jointValueVector.size() - 1, arm));
				}
				if (command == "release") {
					string arm; in >> arm;
					releaseIndices.push_back(make_pair (jointValueVector.size() - 1, arm));
			}
			} else {
				vector<double> jointVals; double jval;
				while (in >> jval) jointVals.push_back(jval);

				jointValueVector.push_back(jointVals);
			}
		}

		if (fileClosed) {iFS.close();}

		processedJointValues = jointValueVector;
		return true;//lfdProcess2 (jointValueVector, src_points, ropePoints, processedJointValues);
	}

	void reset () {iFS.open(jointFile.c_str(), ios::in); fileClosed=false;}

	~LFDProcessor () {iFS.close();}
};

#endif
