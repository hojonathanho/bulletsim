/**
 * Simple script to read a scene-recording and generate data:
 *  --> separate "points" file for each segment.
 *  --> for each segment, find the distance of each point from the robot.
 */
#include "SegmentDemo.hpp"
#include <boost/bind.hpp>
#include <fstream>
#include <algorithm>
#include <utils/colorize.h>
#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "simulation/config_bullet.h"
#include "robots/ravens.h"
#include "utils/config.h"

struct FileConfig : Config {
	static int fnum;

	FileConfig() : Config() {
		params.push_back(new Parameter<int>("fnum", &fnum, "number of the scene-recording file: the X in runX.txt"));
	}
};
int FileConfig::fnum = -1;

class CustomScene : public Scene {
public:
	BulletInstance::Ptr bullet2;
	OSGInstance::Ptr osg2;
	Fork::Ptr fork;
	RaveRobotObject::Ptr origRobot, tmpRobot;
	Ravens ravens;

	PlotPoints::Ptr plot_points;
	PlotAxes::Ptr plot_axes1;
	PlotAxes::Ptr plot_axes2;

	void callGripperAction(char lr) {}

	CustomScene(): ravens(*this){}

	void setup() {
		//BulletConfig::internalTimeStep = 0.001;
		const float dt = BulletConfig::dt;

		// position the ravens
		btTransform T;
		T.setIdentity();
		T.setOrigin(btVector3(0,0,0.05));
		ravens.applyTransform(util::toRaveTransform(T));

		// set up the points for plotting
		plot_points.reset(new PlotPoints(5));
		env->add(plot_points);
		plot_axes1.reset(new PlotAxes());
		env->add(plot_axes1);
		plot_axes2.reset(new PlotAxes());
		env->add(plot_axes2);

		ravens.setArmPose("home",'b');

		//startViewer();
		stepFor(dt, 1.);
	}
};

class runOneSegment {
public:
	CustomScene scene;
	vector<int> rarm_inds;
	vector<int> larm_inds;
	bool done;


	void writeSegPoints(trajSegment::Ptr tseg, string fname) {
		cout<<colorize(string("Saving segment points to file : ") + fname, "green", true)<<endl;
		ofstream file;
		file.open(fname.c_str(), ios::out);

		if (!file.is_open()) {
			cout << colorize("[ERROR : writeSegPoints] : Could not open file to write out points.", "red", true)<<endl;
			exit(-1);
		}

		file << "## rope :\n";
		for (int i = 0; i < tseg->ropePts.size(); ++i)
			file << tseg->ropePts[i].x() << "\t" << tseg->ropePts[i].y() << "\t" << tseg->ropePts[i].z() << "\n";

		// record point clouds of the cloth iff we are in the suturing setup.
		file << "## box-cloth points :\n";
		for (int i = 0; i < tseg->boxPts.size(); ++i)
			file << tseg->boxPts[i].x() << "\t" << tseg->boxPts[i].y() << "\t" << tseg->boxPts[i].z() << "\n";

		file << "## hole-points :\n";
		for (int i = 0; i < tseg->holePts.size(); ++i)
			file << tseg->holePts[i].x() << "\t" << tseg->holePts[i].y() << "\t" << tseg->holePts[i].z() << "\n";

		file.flush();
		file.close();
	}

	/** Extract the joints indexed by INDS from IN_JOINT_VALS and store them into OUT_JOINT_VALS.*/
	void extractJoints (const vector<int> &inds, const vector<double> &in_joint_vals, vector<double> &out_joint_vals) {
		out_joint_vals.clear();
		out_joint_vals.reserve(inds.size());
		for(int i=0; i<inds.size(); i+=1)
			out_joint_vals.push_back(in_joint_vals[inds[i]]);
	}

	/** Calculates the distance of each point in the point-cloud
	 *  from the robot grippers and outputs to a file. */
	void writeWeights(trajSegment::Ptr tseg, string fname, string robot_fname) {
		cout<<colorize(string("Saving weights to file : ") + fname, "green", true)<<endl;
		ofstream file;
		file.open(fname.c_str(), ios::out);

		if (!file.is_open()) {
			cout << colorize("[ERROR : writeWeights] : Could not open file to write out weights.", "red", true)<<endl;
			exit(-1);
		}

		// get the gripper positions at each set of joints.
		KinBody::LinkPtr r_finger1_link = scene.ravens.ravens->robot->GetLink("rhandfinger1_sp");
		KinBody::LinkPtr r_finger2_link = scene.ravens.ravens->robot->GetLink("rhandfinger2_sp");
		KinBody::LinkPtr l_finger1_link = scene.ravens.ravens->robot->GetLink("lhandfinger1_sp");
		KinBody::LinkPtr l_finger2_link = scene.ravens.ravens->robot->GetLink("lhandfinger2_sp");

		vector<btVector3> rgrip_pos(tseg->joints.size());
		vector<btVector3> lgrip_pos(tseg->joints.size());


		for(int i=0; i < tseg->joints.size(); i+=1) {
			vector<double> rarm_joints, larm_joints;
			extractJoints(rarm_inds, tseg->joints[i], rarm_joints);
			extractJoints(larm_inds, tseg->joints[i], larm_joints);

			btTransform right1T  = util::scaleTransform(scene.ravens.manipR->getFK(rarm_joints, r_finger1_link), 1.f/METERS);
			btTransform right2T  = util::scaleTransform(scene.ravens.manipR->getFK(rarm_joints, r_finger2_link), 1.f/METERS);
			btTransform left1T   = util::scaleTransform(scene.ravens.manipL->getFK(larm_joints, l_finger1_link), 1.f/METERS);
			btTransform left2T   = util::scaleTransform(scene.ravens.manipL->getFK(larm_joints, l_finger2_link), 1.f/METERS);

			rgrip_pos[i] = (right1T.getOrigin() + right2T.getOrigin())/2;
			lgrip_pos[i] = (left1T.getOrigin() + left2T.getOrigin())/2;
		}
		//////////////////////////
		ofstream robot_file;
		robot_file.open(robot_fname.c_str(), ios::out);
		for (int i=0; i < tseg->joints.size(); i+=1) {
			btVector3 rr = rgrip_pos[i];
			robot_file << rr.x() << "\t" << rr.y() << "\t" << rr.z() << "\n";
		}
		for (int i=0; i < tseg->joints.size(); i+=1) {
			btVector3 rr = lgrip_pos[i];
			robot_file << rr.x() << "\t" << rr.y() << "\t" << rr.z() << "\n";
		}
		robot_file.close();
		////////////////////////////



		file << "## rope :\n";
		for (int i = 0; i < tseg->ropePts.size(); ++i) {
			const btVector3 pt = tseg->ropePts[i];

			vector<double> rdists(tseg->joints.size());
			vector<double> ldists(tseg->joints.size());
			for(int j=0; j < tseg->joints.size(); j+=1) {
				rdists[j] = rgrip_pos[j].distance(pt);
				ldists[j] = lgrip_pos[j].distance(pt);
			}

			double minPtDist = min(*min_element(rdists.begin(),rdists.end()), *min_element(ldists.begin(),ldists.end()));
			file << minPtDist <<"\n";
		}

		// record point clouds of the cloth iff we are in the suturing setup.
		file << "## box-cloth points :\n";
		for (int i = 0; i < tseg->boxPts.size(); ++i) {
			const btVector3 pt = tseg->boxPts[i];

			vector<double> rdists(tseg->joints.size());
			vector<double> ldists(tseg->joints.size());
			for(int j=0; j < tseg->joints.size(); j+=1) {
				rdists[j] = rgrip_pos[j].distance(pt);
				ldists[j] = lgrip_pos[j].distance(pt);
			}

			double minPtDist = min(*min_element(rdists.begin(),rdists.end()), *min_element(ldists.begin(),ldists.end()));
			file << minPtDist <<"\n";
		}

		file << "## hole-points :\n";
		for (int i = 0; i < tseg->holePts.size(); ++i) {
			const btVector3 pt = tseg->holePts[i];

			vector<double> rdists(tseg->joints.size());
			vector<double> ldists(tseg->joints.size());
			for(int j=0; j < tseg->joints.size(); j+=1) {
				rdists[j] = rgrip_pos[j].distance(pt);
				ldists[j] = lgrip_pos[j].distance(pt);
			}

			double minPtDist = min(*min_element(rdists.begin(),rdists.end()), *min_element(ldists.begin(),ldists.end()));
			file << minPtDist <<"\n";
		}

		file.flush();
		file.close();
	}

	void genData() {
		// open the scene-recording file
		stringstream scenerecss;
		scenerecss << EXPAND(BULLETSIM_SRC_DIR)"/tests/ravens/recorded/simruns/run" << FileConfig::fnum << ".txt";
		Segmenter seg(scenerecss.str());

		string fdir = string(EXPAND(BULLETSIM_SRC_DIR)) + string("/tests/ravens/recorded/point_data");

		int segnum = 0;
		trajSegment::Ptr tseg = seg.getNextSegment();
		while(tseg) {
			stringstream pt_fname;
			pt_fname <<  fdir << "/run" << FileConfig::fnum << "-seg" << segnum << "-points.txt";
			writeSegPoints(tseg, pt_fname.str());

			stringstream dists_fname;
			dists_fname <<  fdir << "/run" << FileConfig::fnum << "-seg" << segnum << "-dists.txt";

			stringstream robot_fname;
			robot_fname <<  fdir << "/run" << FileConfig::fnum << "-seg" << segnum << "-robot.txt";
			writeWeights(tseg, dists_fname.str(), robot_fname.str());


			cout << endl;
			tseg  = seg.getNextSegment();
			segnum += 1;
		}
	}

	runOneSegment() {
		// the dofs of the robot which are set
		larm_inds = scene.ravens.manipL->manip->GetArmIndices();
		rarm_inds = scene.ravens.manipR->manip->GetArmIndices();

		scene.setup();
		genData();
	}
};


int main(int argc, char *argv[]) {

	GeneralConfig::scale  = 100.;

	Parser parser;
	parser.addGroup(GeneralConfig());
	parser.addGroup(BulletConfig());
	parser.addGroup(SceneConfig());
	parser.addGroup(FileConfig());
	parser.read(argc, argv);



	runOneSegment();
	return 0;
}
