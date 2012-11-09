#include "particlesystem2.h"

#include "ophys_config.h"
#include "sqp/config_sqp.h"
#include "simulation/simplescene.h"
#include <boost/timer.hpp>
using namespace ophys;

#include <iostream>
using namespace std;

struct LocalConfig : public Config {
  static int numParticles;
  static int horizon;
  LocalConfig() {
    params.push_back(new Parameter<int>("numParticles", &numParticles, ""));
    params.push_back(new Parameter<int>("horizon", &horizon, ""));
  }
};
int LocalConfig::numParticles = 10;
int LocalConfig::horizon  = 2;

int main(int argc, char *argv[]) {
  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.addGroup(BulletConfig());
  parser.addGroup(SceneConfig());
  parser.addGroup(SQPConfig());
  parser.addGroup(LocalConfig());
  parser.read(argc, argv);

  GeneralConfig::scale = 1;

  initializeGRB();

  ParticleSystemOptimizer2::SysState initState(LocalConfig::numParticles, ParticleSystemOptimizer2::PARTICLE_STATE_DIM);
  for (int i = 0; i < LocalConfig::numParticles; ++i) {
    initState.row(i) <<
      (-1 + 2*i/(LocalConfig::numParticles-1.0))*METERS, 0, 0.5*METERS, // pos
      0, 0, 0, // vel
      0, 0, 0; // acc
  }

  Scene scene;

  ParticleSystem2 ps(initState);
  ps.attachToScene(&scene);

  scene.startViewer();

#if 0
  for (int iter = 1; iter <= 1000; ++iter) {
    //cout << "\n\n=" << iter << "=================\n\n" << endl;
    ps.step(OPhysConfig::dt, 2);
    //cout << "==== finished iter " << iter << endl;
    scene.step(0);
//      scene.idleFor(0.2);
  }
  cout << "particle 0: " << ps.m_currState.row(0) << endl;
#else
  boost::timer t;
  ps.step(OPhysConfig::dt, LocalConfig::horizon);
  cout << "done optimizing in " << t.elapsed() << " s" << endl;
  scene.idle(true);
  for (int i = 0; i < ps.getAllStates().size(); ++i) {
    ps.draw(ps.getAllStates()[i]);
    scene.step(0);
    scene.idle(true);
    //scene.idleFor(0.2);
  }
  cout << "particle 0: " << ps.m_currState.row(0) << endl;
#endif

  scene.idle(true);

  return 0;
}
