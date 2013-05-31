#include "jointLFDProcessor.h"
#include "CustomScene.h"

void LFDProcessor::initProcessing () {

	iFS.open(jointFile.c_str(), ios::in);
	if (!iFS) {
		cout<<"File does not exist!"<<endl;
		fileClosed = true;
		return;
	}

	string line; getline(iFS, line);
	istringstream in(line); string section; in >> section;
	// File always starts with section
	assert (section == "section");
	fileClosed = false;
	mode_count = 0;

	//HARD CODE MODES:
	hardCodeModes();
}

void LFDProcessor::hardCodeModes () {
	modes.clear();
	modes.push_back("pierce");
	modes.push_back("pierce");
	modes.push_back("pickup");
	modes.push_back("pickup");
	modes.push_back("pickup");
	modes.push_back("pickup");
	modes.push_back("pickup");
	modes.push_back("knot2");
	modes.push_back("knot2");
	//modes.push_back("pickup");
	//modes.push_back("knot");
	//modes.push_back("knot");
	//modes.push_back("knot");
}

bool LFDProcessor::preProcess (	Ravens & ravens,
		vector<btVector3> & new_rope_points,
		//vector<btVector3> & new_needle_points,
		vector<btVector3> & new_box_points,
		vector<btVector3> & new_hole_points,
		vector< vector <double> > & processedJointValues,
		CustomScene & s) {

	if (fileClosed) {
		cout<<"File has ended."<<endl;
		return false;
	}

	jointValueVector.clear();
	grabIndices.clear();
	releaseIndices.clear();
	use_rope = use_needle = use_box = use_hole = false;

	rope_points.clear();
	hole_points.clear();
	box_points.clear();
	needle_points.clear();

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

	if (s.tsuite == SUTURING) {
		if (mode_count < modes.size()) {
			use_needle = false;
			if (modes[mode_count] == "knot1")
				use_box = use_hole = false;
			else if (modes[mode_count] == "flap")
				use_rope = use_hole = false;
			else if (modes[mode_count] == "pierce")
				use_rope = use_box = false;
			else if (modes[mode_count] == "pickup")
				use_box = false;
		}
	} else if (s.tsuite == ROPE_MANIP) {
		use_needle = use_box = use_hole = false;
	}

	mode_count ++;


	//bool successful = warpRavenJoints (ravens, rope_points, new_rope_points, jointValueVector, processedJointValues);

	//vector< pair<bool, pair<vector<btVector3>, vector<btVector3> > > > data;

	//data.push_back(make_pair(use_rope, make_pair(rope_points, new_rope_points)));
	//data.push_back(make_pair(use_needle, make_pair(needle_points, new_needle_points)));
	//data.push_back(make_pair(use_box, make_pair(box_points, new_box_points)));
	//data.push_back(make_pair(use_hole, make_pair(hole_points, new_hole_points)));

	//Actually should be:
	/* bool successful = warpRavenJoints (ravens, data, jointValueVector, processedJointValues);*/
	// OR

	cout<<"Rope source points: "<<rope_points.size()<<" and target: "<<new_rope_points.size()<<endl;
	cout<<"Box source points: "<<box_points.size()<<" and target: "<<new_box_points.size()<<endl;
	cout<<"Hole source points: "<<hole_points.size()<<" and target: "<<new_hole_points.size()<<endl;

	vector <btVector3> old, newpts;
	for (int i = 0; i < rope_points.size(); ++i) old.push_back(rope_points[i]);
	for (int i = 0; i < box_points.size(); ++i) old.push_back(box_points[i]);
	for (int i = 0; i < hole_points.size(); ++i) old.push_back(hole_points[i]);

	for (int i = 0; i < new_rope_points.size(); ++i) newpts.push_back(new_rope_points[i]);
	for (int i = 0; i < new_box_points.size(); ++i) newpts.push_back(new_box_points[i]);
	for (int i = 0; i < new_hole_points.size(); ++i) newpts.push_back(new_hole_points[i]);
	s.plotAllPoints2(old , newpts);

	bool successful = warpRavenJoints (	ravens,
			make_pair(use_rope, make_pair(rope_points, new_rope_points)),
			//make_pair(use_needle, make_pair(needle_points, new_needle_points)),
			make_pair(use_box, make_pair(box_points, new_box_points)),
			make_pair(use_hole, make_pair(hole_points, new_hole_points)),
			jointValueVector, processedJointValues);

	if (!successful) {
		cout<<"Unable to find trajectory."<<endl;
		iFS.close(); fileClosed = true;
	}
	return successful;
}

void LFDProcessor::reset () {
	iFS.close();
	fileClosed = true;
}
