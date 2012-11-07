#include "particlesystem.h"

#include "ophys_config.h"
#include "sqp/config_sqp.h"
#include "simulation/simplescene.h"

using namespace ophys;

#include <iostream>
using namespace std;

int main(int argc, char *argv[]) {
  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.addGroup(BulletConfig());
  parser.addGroup(SceneConfig());
  parser.addGroup(SQPConfig());
  parser.read(argc, argv);

  GeneralConfig::scale = 1;

  initializeGRB();

  int nparticles = 10;
  ParticleSystemState initState;
  for (int i = 0; i < nparticles; ++i) {
    ParticleState p;
    p.x << (-1 + 2*i/(nparticles-1.0))*METERS, 0, 5*METERS;
    p.v << 0, 0, 0;
    p.a << 0, 0, 0;
    initState.push_back(p);
  }

  Scene scene;

  ParticleSystem ps(initState);
  ps.attachToScene(&scene);

  scene.startViewer();

  for (int iter = 1; iter <= 1000000; ++iter) {
    //cout << "\n\n=" << iter << "=================\n\n" << endl;
    ps.step(OPhysConfig::dt);
    //cout << "==== finished iter " << iter << endl;
    scene.step(0);
//      scene.idleFor(0.2);
  }


  return 0;
}
