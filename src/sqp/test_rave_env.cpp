#include "simulation/simplescene.h"
#include "simulation/config_bullet.h"
#include "robots/pr2.h"
int main(int argc, char* argv[]) {
  Parser parser;
  parser.addGroup(BulletConfig());
  parser.read(argc, argv);
  Scene scene;
  Load(scene.env, scene.rave, "data/pr2test1.env.xml",0);
  PR2Manager pr2m(scene);
  scene.startViewer();
  scene.startLoop();
}
