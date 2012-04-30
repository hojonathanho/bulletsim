#include "simulation/simplescene.h"
#include "robots/pr2.h"

int main(int argc, char* argv[]) {
  SceneConfig::enableIK = true;
  SceneConfig::enableHaptics = true;
  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.read(argc, argv);
  
  Scene scene;
  Load(scene.env, scene.rave, "data/lab1.env.xml");
  scene.startViewer();
  scene.step(0);
  scene.idle(true);
  scene.startLoop();
  
}
