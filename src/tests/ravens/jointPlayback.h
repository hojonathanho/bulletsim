#ifndef __JOINT_PLAYBACK__
#define __JOINT_PLAYBACK__

#include <sstream>
#include <string>
#include <assert.h>
//#include <vector>

#include "robots/ravens.h"

/* Class to load a trajectory and play it back. **/
class jointPlayback {

	Scene &scene;					// Scene in which robot is
	Ravens::Ptr ravens;				// Ravens from the scene
	RaveTrajectory::Ptr traj;		// Trajectory to follow from playback

	float recorded_freq;			// Frequency of recording
	vector<dReal> maxVel;			// Velocity limits
	vector<dReal> maxAcc;			// Acceleration limits

	string filename;				// Name of file to read from
	ifstream file;					// Input file stream to read file

public:

	typedef boost::shared_ptr<jointPlayback> Ptr;

	// Constructor
	jointPlayback (Scene &_scene, Ravens * _robot, float _freq = -1) :
		scene(_scene), ravens (_robot), recorded_freq (_freq), maxVel(16,2.0), maxAcc(16,5.0){
		filename = "/home/ankush/sandbox/bulletsim/src/tests/ravens/recorded/raven_joints.txt";
		if (recorded_freq == -1)
			recorded_freq = RavenConfig::record_freq;
	}

	// Loads trajectory from specified file
	void loadTrajectory () {

		TrajectoryBasePtr rave_traj;

		// Initialize trajectory
		rave_traj = RaveCreateTrajectory(scene.rave->env,"");

		std::cout<<"Sup1"<<std::endl;

		ConfigurationSpecification spec(ravens->ravens->robot->GetConfigurationSpecification().GetGroupFromName("joint_values"));
		int dTime_ind = spec.AddDeltaTimeGroup();
	    rave_traj->Init(spec);

	    std::cout<<"Sup2"<<std::endl;
	    //rave_traj->

		file.open(filename.c_str(), ios::in);

		string joints;

		std::cout<<"Size:: "<<rave_traj->GetDOF()<<std::endl;

		vector<dReal> dof_values(rave_traj->GetDOF());
		float jval;
		bool first_line = true;

		while(getline(file, joints)) {

		    istringstream in(joints);
		    dof_values.clear();
		    while (in >> jval) dof_values.push_back(jval);

		    std::cout<<"Sup inner 1"<<std::endl;

		    if (first_line) {
		    	dof_values.push_back(0.0);
		    	first_line = false;
		    }
		    else dof_values.push_back(1/recorded_freq);

		    std::cout<<"Sup inner 2"<< dof_values.size()<<std::endl;
			rave_traj->Insert(rave_traj->GetNumWaypoints(), dof_values, spec);

			std::cout<<"Sup inner 3"<<std::endl;
		}

		file.close();

		std::cout<<"Sup3"<<std::endl;

		// Store all joint indices from 0 to 15
		vector<int> joint_inds;
		extractIndicesFromGroupName(rave_traj->GetConfigurationSpecification().GetGroupFromName("joint_values").name, joint_inds);
		//int i = 0;
		//while (i < jSize) joint_inds.push_back(i++);

		// Not sure what has to be done about this. Might want to give actual duration of trajectory
		//planningutils::RetimeAffineTrajectory(rave_traj, maxVel, maxAcc);

		// Retime trajectory to go at initial frequency.
		/*ConfigurationSpecification retimeSpec;
		retimeSpec.AddDeltaTimeGroup();

		vector<dReal> vnewdeltatimes(rave_traj->GetNumWaypoints(), 1/recorded_freq);
		vnewdeltatimes[0] = 0;
		rave_traj->Insert(0,vnewdeltatimes,retimeSpec,true);
		*/
		std::cout<<"Duration 1: "<<rave_traj->GetDuration()<<std::endl;

		std::vector<dReal> valsasa;
		rave_traj->GetWaypoint(20, valsasa);

		std::cout<<"Size 1: "<<valsasa.size()<<std::endl;

		for (int i = 0; i < valsasa.size(); ++i)
			std::cout<<valsasa[i]<< " woops0 ";
		std::cout<<std::endl;

		std::cout<< rave_traj->GetConfigurationSpecification().GetGroupFromName("del").name << std::endl;

		planningutils::RetimeAffineTrajectory(rave_traj, maxVel, maxAcc, true);

		rave_traj->GetWaypoint(20, valsasa);

		std::cout<<valsasa.size()<<std::endl;

		for (int i = 0; i < valsasa.size(); ++i)
				std::cout<<valsasa[i]<< " woops ";
		std::cout<<std::endl;

		traj.reset(new RaveTrajectory(rave_traj, ravens->ravens, joint_inds));
		std::cout<<"Size of trajectory: "<<traj->trajectory->GetNumWaypoints()<<std::endl;


		std::cout<<"Duration 2: "<<traj->duration()<<std::endl;
	}

	// Plays back the trajectory.
	void runTrajectory () {
		ravens->controller->appendTrajectory(traj);
		ravens->controller->run();

	}

	// Extracts joint indices from group name
	void extractIndicesFromGroupName (string group_name, vector<int> &indices) {
		std::cout<<group_name<<std::endl;
		istringstream in(group_name);

		string ind_str;
		int ind;
		while (in >> ind_str) {
			if (!isdigit(ind_str[0])) continue;
			ind = atoi(ind_str.c_str());
			indices.push_back(ind);
		}
	}
};

#endif
