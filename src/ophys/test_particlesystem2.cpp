#include "particlesystem2.h"

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
  ParticleSystemOptimizer2::SysState initState(nparticles, ParticleSystemOptimizer2::PARTICLE_STATE_DIM);
  for (int i = 0; i < nparticles; ++i) {
    initState.row(i) <<
      (-1 + 2*i/(nparticles-1.0))*METERS, 0, 5*METERS, // pos
      0, 0, 0, // vel
      0, 0, 0; // acc
  }

  Scene scene;

  ParticleSystem2 ps(initState);
  ps.attachToScene(&scene);

  scene.startViewer();

  for (int iter = 1; iter <= 99999999999; ++iter) {
    //cout << "\n\n=" << iter << "=================\n\n" << endl;
    ps.step(OPhysConfig::dt, 2);
    //cout << "==== finished iter " << iter << endl;
    scene.step(0);
//      scene.idleFor(0.2);
  }


  return 0;
}
