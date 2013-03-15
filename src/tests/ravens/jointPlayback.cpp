#include "jointPlayback.h"
#include "CustomScene.h"


jointPlayback::jointPlayback (	CustomScene &_scene, bool _processing, float _freq, float _dt, string _filename) :
								scene(_scene), freq (_freq), dt (_dt), currTime (0.0), initialized (false),
								enabled(false), file_closed(true), processing (_processing), filename(_filename) {

	if (freq == -1.0)
		freq = RavenConfig::record_freq;

	if (dt == -1.0)
		dt = BulletConfig::dt;

	int dof = scene.ravens.ravens->robot->GetDOF();
	for (int i = 0; i < dof; ++i) joint_inds.push_back(i);

	lfdProcessor.reset(new LFDProcessor (_filename));

	scene.addPreStepCallback(boost::bind(&jointPlayback::executeNextWaypoint, this));
	scene.addPreStepCallback(boost::bind(&jointPlayback::playProcessed, this));
}


void jointPlayback::executeNextWaypoint () {
	if (processing) return;
	if (!enabled) return;

	string line;
	float jval;

	if (currTime >= 1/freq) {
		float fileEnded = true;
		while (getline(file, line)) {
			istringstream in(line);
			string command; in >> command;
			if (command == "grab" || command == "release") {
				string arm;	in >> arm;
				RavensRigidBodyGripperAction::Ptr gripAct = (arm == "l" ? scene.lAction : scene.rAction);
				if (command == "grab") {
					cout<<"Playback: Grabbing."<<endl;
					gripAct->grab(10);
				}
				else if (command == "release") {
					cout<<"Playback: Releasing."<<endl;
					gripAct->reset();
				}
			} else if (command == "joints") {
				joint_vals.clear();
				while (in >> jval) joint_vals.push_back(jval);
				scene.ravens.ravens->setDOFValues(joint_inds, joint_vals);
				currTime = 0.0;
				fileEnded = false;
				break;
			}
		}
		if (fileEnded) {
			file.close();
			cout<<"Finished playing back file."<<endl;
			enabled = false;
			file_closed = true;
		}
	} else currTime += dt;
}

void jointPlayback::process () {
	processedSuccessfully = lfdProcessor->preProcess(scene.ravens, scene.getRopePoints(true), processedJoints);
	if (processedSuccessfully) {
		jCount = 0;
		gCount = (lfdProcessor->grabIndices.size() > 0 ? 0 : -1);
		rCount = (lfdProcessor->releaseIndices.size() > 0 ? 0 : -1);
	} else { initialized = false; enabled = false;}
}

void jointPlayback::playProcessed () {
	if (!processing) return;
	if (!enabled) return;
	if (!processedSuccessfully) return;

	if (currTime >= 1/freq) {

		// Perform grabbing
		while (true) {
			if (gCount != -1 && lfdProcessor->grabIndices[gCount].first <= jCount) {
				RavensRigidBodyGripperAction::Ptr gripAct = (lfdProcessor->grabIndices[gCount].second == "l" ? scene.lAction : scene.rAction);

				cout<<"Playback: Grabbing."<<endl;
				gripAct->grab(10);

				gCount = (gCount + 1 < lfdProcessor->grabIndices.size() ? gCount + 1 : -1);
			} else break;
		}
		// Perform releasing
		while (true) {
			if (rCount != -1 && lfdProcessor->releaseIndices[rCount].first <= jCount) {
				RavensRigidBodyGripperAction::Ptr gripAct = (lfdProcessor->releaseIndices[rCount].second == "l" ? scene.lAction : scene.rAction);

				cout<<"Playback: Releasing."<<endl;
				gripAct->reset();

				rCount = (rCount + 1 < lfdProcessor->releaseIndices.size() ? rCount + 1 : -1);
			} else break;
		}

		scene.ravens.ravens->setDOFValues(joint_inds, processedJoints[jCount]);
		currTime = 0.0;
		if (++jCount >= processedJoints.size()) {
			process();
			if (!processedSuccessfully)
			{initialized = false; enabled = false;}
		}
	} else currTime += dt;

}
