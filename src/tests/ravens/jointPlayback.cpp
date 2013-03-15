#include "jointPlayback.h"
#include "CustomScene.h"


jointPlayback::jointPlayback (	CustomScene &_scene, bool _processing, float _freq, float _dt, string _filename) :
								scene(_scene), freq (_freq), dt (_dt), currTime (0.0), initialized (false), grabMode(false),
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
				grabAct = (arm == "l" ? scene.lAction : scene.rAction);
				if (command == "grab") {
					cout<<"Playback: Grabbing."<<endl;
					grabMode = true;
					grabAct->setCloseAction();
				}
				else if (command == "release") {
					cout<<"Playback: Releasing."<<endl;
					grabMode = false;
					grabAct->reset();
				}
			} else if (command == "joints") {
				joint_vals.clear();
				while (in >> jval) joint_vals.push_back(jval);
				scene.ravens.ravens->setDOFValues(joint_inds, joint_vals);
				if (grabMode) {
					grabAct->step(max(1/freq, dt));
					if (grabAct->done()) grabMode = false;
				}
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

		char action = 'n';
		if (gCount != -1 && lfdProcessor->grabIndices[gCount].first == jCount) {
			action = 'g';
			grabAct = (lfdProcessor->grabIndices[gCount].second == "l" ? scene.lAction : scene.rAction);
			while (gCount < lfdProcessor->grabIndices.size() && lfdProcessor->grabIndices[gCount].first == jCount) gCount ++;
			if (gCount == lfdProcessor->grabIndices.size()) gCount = -1;
		}
		if (rCount != -1 && lfdProcessor->releaseIndices[rCount].first == jCount) {
			action = 'r';
			grabAct = (lfdProcessor->releaseIndices[rCount].second == "l" ? scene.lAction : scene.rAction);
			while (rCount < lfdProcessor->releaseIndices.size() && lfdProcessor->releaseIndices[rCount].first == jCount) rCount ++;
			if (rCount == lfdProcessor->releaseIndices.size()) rCount = -1;
		}

		if (action == 'g') {
			cout<<"Playback: Grabbing."<<endl;
			grabMode = true;
			grabAct->setCloseAction();
		}
		else if (action == 'r') {
			cout<<"Playback: Releasing."<<endl;
			grabMode = false;
			grabAct->reset();
		}

		scene.ravens.ravens->setDOFValues(joint_inds, processedJoints[jCount]);
		if (grabMode) {
			grabAct->step(3.0*max(1/freq, dt));
			if (grabAct->done()) grabMode = false;
		}

		currTime = 0.0;
		if (++jCount >= processedJoints.size()) {
			process();
			if (!processedSuccessfully)
			{initialized = false; enabled = false;}
		}
	} else currTime += dt;

}
