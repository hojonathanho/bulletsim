#include "simulation/simplescene.h"
#include "robots/pr2.h"
typedef boost::function<void(void)> VoidCallback;
int main(int argc, char* argv[]) {
  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.read(argc, argv);
  Scene scene;
  Load(scene.env, scene.rave, "/home/joschu/bulletsim/data/xml/raven_env.xml",true);
//  PR2Manager pr2m(scene);
  scene.startViewer();
  scene.startLoop();
}
