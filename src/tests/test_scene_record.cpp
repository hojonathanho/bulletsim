#include "simplescene.h"
#include "recording.h"

int main(int argc, char *argv[]) {
    // first read the configuration from the user

    // and override config values to what we want
    SceneConfig::enableIK = false;
    SceneConfig::enableRobot = false;
    SceneConfig::enableHaptics = false;

    Parser parser;
    parser.addGroup(RecordingConfig());

    parser.read(argc, argv);    
    
    // construct the scene
    Scene scene;
    // manipulate the scene or add more objects, if desired

    // start the simulation
    scene.startViewer();
    scene.step(0);
    ScreenRecorder rec(scene.viewer);

    for (int t=0; t < 100; t++)  {
      rec.snapshot();
      scene.step(.1);
    }

    return 0;
}
