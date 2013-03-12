#include "jointRecorder.h"
#include "CustomScene.h"

	// Constructor
jointRecorder::jointRecorder (	CustomScene &_scene, float _dt, float _record_freq, string _filename) :
								scene (_scene), recording(false), dt(_dt), init (true), currTime(0.0),
								record_freq(_record_freq), filename(_filename) {

	if (dt == -1)
		dt = BulletConfig::dt;

	if (record_freq == -1.0)
		record_freq = RavenConfig::record_freq;

	scene.addPreStepCallback(boost::bind(&jointRecorder::recordCallback, this));
}

// Toggles file and opens/closes file appropriately
void jointRecorder::toggleRecording () {
	recording = !recording;
	cout << "Recording: " << (recording ? "true" : "false") << endl;
	if (recording) {
		file.open(filename.c_str(), ios::out | ios::app);
		if (init)
			scene.recordRopePoints();
	} else {
		file.close();
		currTime = 0.0;
	}
}

/* Callback which opens file, stores latest joint values, closes file.
 * Does so only when the time since last check exceeds time period of checks.
 *
 * Must be added to the scene's list of callbacks. */
void jointRecorder::recordCallback () {
	if (currTime >= 1.0/record_freq && recording) {
		joint_vals.clear();
		scene.ravens.ravens->robot->GetDOFValues(joint_vals);
		int jsize = joint_vals.size();

		for (int i = 0; i < jsize; ++i)
			file << joint_vals[i] << " ";
		file << "\n";
		file.flush();

		currTime = 0.0;

	} else if (recording) currTime += dt;
}
