#include "particlesystem.h"

#include "ophys_config.h"
#include "sqp/config_sqp.h"
#include "simulation/simplescene.h"

using namespace ophys;

#include <iostream>
using namespace std;

void addAnchorCost(int p, const Vector3d &anchorpt, ParticleSystemOptimizer *opt) {
  PointAnchorCost::Ptr anchorCost(new PointAnchorCost(*opt, p, anchorpt));
  opt->addCost(anchorCost);
}

int main(int argc, char *argv[]) {
  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.addGroup(BulletConfig());
  parser.addGroup(SceneConfig());
  parser.addGroup(SQPConfig());
  parser.read(argc, argv);

  GeneralConfig::scale = 1;

  initializeGRB();

  const int nparticles = 10;
  const double ropeLen = 2*METERS;

  ParticleSystemState initState;
  double segRlen = ropeLen / (nparticles-1.0);
  for (int i = 0; i < nparticles; ++i) {
    ParticleState p;
    p.x << (-ropeLen/2. + ropeLen*i/(nparticles-1.0)), 0, 5*METERS;
    p.v << 0, 0, 0;
    p.a << 0, 0, 0;
    initState.push_back(p);
  }

  Scene scene;

  RopeSystem ps(initState, segRlen);
  ps.attachToScene(&scene);

  // anchor the first point to its initial spot
  ps.setPreOptCallback(boost::bind(&addAnchorCost, 0, initState[0].x, _1));

  scene.startViewer();

  for (int iter = 1; ; ++iter) {
    ps.step(OPhysConfig::dt);
    scene.step(0);
  }

  return 0;
}
