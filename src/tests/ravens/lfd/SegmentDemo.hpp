#pragma once
#include <vector>
#include <boost/shared_ptr.hpp>
#include <string>
#include <fstream>
#include <Eigen/Core>
#include <simulation/simplescene.h>

enum GripperAction{RELEASE_R, GRAB_R, RELEASE_L, GRAB_L};

/** A segment of the demo trajectory : joints + env data b/w two looks. **/
struct trajSegment {
	typedef boost::shared_ptr<trajSegment> Ptr;

	// robot joint values and their time-stamps
	std::vector<double> jtimes;
	std::vector<std::vector<double> > joints;

	// robot gripper actions and their time stamps
	std::vector<double> gtimes;
	std::vector<GripperAction> grips;

	// point-cloud data at the start of every segment
	std::vector<btVector3> ropePts;
	std::vector<btVector3> boxPts;
	std::vector<btVector3> holePts;
};


/** Segments a demo-file and sequentially returns segments upon requests. */
class Segmenter {
private:
	std::ifstream demofile;
	bool firstSegmentDone;

	/** Just finds a prompt in a vector of strings. */
	int getPromptIndex(std::vector<std::string> ss);

	/** Trivial predicates to resolve the type of line. */
	bool inline isPointCloudLine(const std::string &line);

	bool inline isPointLine(const std::string &line);

	/** Reads a point-cloud into tseg from file. */
	void readPoints(std::ifstream &file, trajSegment::Ptr tseg);

	/** Returns the command in the data-file. */
	std::string inline getCommand(const std::vector<std::string> &splitline);

	/** Returns the time-stamp on a command in the date-file. */
	double inline getTimeStamp(const std::vector<std::string> &splitline);

public:
	typedef boost::shared_ptr<Segmenter> Ptr;

	Segmenter(std::string fpath);

	/** Parse the demo data file to generate segment information. */
	trajSegment::Ptr getNextSegment();
};
