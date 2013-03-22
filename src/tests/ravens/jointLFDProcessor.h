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

	LFDProcessor (string _inputFile = EXPAND(BULLETSIM_SRC_DIR)"/tests/ravens/recorded/raven_joints.txt") :
						   jointFile (_inputFile), fileClosed(false),
						   use_rope (false), use_needle (false), use_box (false), use_hole (false){}

	void setInputFile (const string _inputFile) {jointFile = _inputFile;}

	void initProcessing ();

	void initializeModes (const vector<string> & _modes) {modes = _modes;}

	void hardCodeModes ();

	bool preProcess (	Ravens & ravens,
						vector<btVector3> & new_rope_points,
						//vector<btVector3> & new_needle_points,
						vector<btVector3> & new_box_points,
						vector<btVector3> & new_hole_points,
						vector< vector <double> > & processedJointValues);

	void reset ();

	~LFDProcessor () {iFS.close();}
};

#endif
