#ifndef __JOINT_PLAYBACK__
#define __JOINT_PLAYBACK__

#include <sstream>
#include <string>
//#include <vector>

#include "robots/ravens.h"

/* Class to load a trajectory and play it back. **/
class jointPlayback {

	Scene &scene;					// Scene in which robot is
	Ravens::Ptr robot;				// Ravens from the scene
	RaveTrajectory::Ptr traj;		// Trajectory to follow from playback

	vector<dReal> maxVel;			// Velocity limits
	vector<dReal> maxAcc;			// Acceleration limits

	string filename;				// Name of file to read from
	ifstream file;					// Input file stream to read file

public:

	typedef boost::shared_ptr<jointPlayback> Ptr;

	// Constructor
	jointPlayback (Scene &_scene, Ravens * _robot) :
		scene(_scene), robot (_robot), maxVel(16,2.0), maxAcc(16,5.0){
		filename = "/home/ankush/sandbox/bulletsim/src/tests/ravens/recorded/raven_joints.txt";
	}

	// Loads trajectory from specified file
	void loadTrajectory () {

		TrajectoryBasePtr rave_traj;
		int jSize = robot->ravens->robot->GetDOF();

		// Initialize trajectory
		rave_traj = RaveCreateTrajectory(scene.rave->env,"");
	    ConfigurationSpecification spec(robot->ravens->robot->GetConfigurationSpecification().GetGroupFromName("joint_values"));
	    rave_traj->Init(spec);

		file.open(filename.c_str(), ios::in);

		string joints;
		vector<dReal> dof_values(jSize);
		float jval;

		while(getline(file, joints)) {

		    istringstream in(joints);
		    dof_values.clear();
		    while (in >> jval) dof_values.push_back(jval);
			rave_traj->Insert(rave_traj->GetNumWaypoints(), dof_values);
		}

		file.close();

		// Store all joint indices from 0 to 15
		vector<int> joint_inds;
		int i = 0;
		while (i < jSize) joint_inds.push_back(i++);

		// Not sure what has to be done about this. Might want to give actual duration of trajectory
		planningutils::RetimeAffineTrajectory(rave_traj, maxVel, maxAcc);

		traj.reset(new RaveTrajectory(rave_traj, robot->ravens, joint_inds));
		std::cout<<"Size of trajectory: "<<traj->trajectory->GetNumWaypoints()<<std::endl;
	}

	// Plays back the trajectory.
	void runTrajectory () {
		robot->controller->appendTrajectory(traj);
		robot->controller->run();
	}

};

#endif
