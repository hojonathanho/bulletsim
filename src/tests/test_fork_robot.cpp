#include "simulation/simplescene.h"
#include "robots/pr2.h"

int main(int argc, char* argv[]) {
  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.read(argc, argv);
  Scene scene;
  PR2Manager pr2m(scene);

  Fork::Ptr fork(new Fork(scene.env));
  scene.osg->root->addChild(fork->env->osg->root.get());

  Scene scene2(fork->env);

//  pr2m.pr2->setColor(0, 0, 1, 0.1);

  scene.startViewer();
  scene.draw();
  scene.startLoop();

  scene2.startViewer();
  scene2.draw();

  scene2.startLoop();
}
