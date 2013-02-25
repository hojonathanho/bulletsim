#ifndef __JOINT_RECORDER__
#define __JOINT_RECORDER__

#include <string>
#include <fstream>
#include <ctime>
#include <vector>

#include "simulation/openravesupport.h"

/** Class to record joint values and store them to a file. */
class jointRecorder {

	RaveRobotObject::Ptr robot;			// Robot whose joints we are recording
	std::vector<dReal> joint_vals;		// Vector to store joint values
	float freq;							// Frequency of recording

	std::string filename; 				// File name
	ofstream file;						// Output file stream
	clock_t currClockTicks;				// Current # of clock ticks

public:

	typedef boost::shared_ptr<jointRecorder> Ptr;

	// Constructor
	jointRecorder (std::string _fname, RaveRobotObject::Ptr _robot, float _freq = 5.0) :
		filename (_fname), robot(_robot), currClockTicks (clock()), freq(_freq)
		{file.open(filename.c_str(), ios::app | ios::out);}

	/* Callback which opens file, stores latest joint values, closes file.
	 * Does so only when the time since last check exceeds time period of checks.
	 *
	 * Must be added to the scene's list of callbacks. */
	void recordCallback () {

		if (((float)(clock() - currClockTicks))/CLOCKS_PER_SEC >= 1/freq) {

			std::cout<<"hi"<<std::endl;

			robot->robot->GetDOFValues(joint_vals);
			int jsize = joint_vals.size();

			for (int i = 0; i < jsize; ++i)
				//joint_vals_str.append()
				file << joint_vals[i] << " ";
			file << "\n";
			file.flush();

			currClockTicks = clock();
		}
	}

	~jointRecorder () { std::cout<<"dead"; file.close();}
};

#endif
