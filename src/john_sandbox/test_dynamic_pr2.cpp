#include "simulation/simplescene.h"
#include "robots/pr2.h"

int main(int argc, char* argv[]) {
  Scene scene;
  Load(scene.env, scene.rave, "robots/pr2-beta-static.zae",true);
  scene.startViewer();
  scene.startLoop();
}
