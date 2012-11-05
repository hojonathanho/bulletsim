#include "particlesystem.h"

#include "ophys_config.h"
#include "sqp/config_sqp.h"

#include <iostream>
using namespace std;

int main(int argc, char *argv[]) {
  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.addGroup(SQPConfig());
  parser.read(argc, argv);

  GeneralConfig::scale = 100;

  initializeGRB();

  int nparticles = 2;
  ParticleSystemState initState;
  for (int i = 0; i < nparticles; ++i) {
    ParticleState p;
    p.x << 1*METERS, 0, 5*METERS;
    p.v << 0, 0, 0;
    p.a << 0, 0, 0;
    initState.push_back(p);
  }

  try {
    ParticleSystem ps(initState);

    for (int iter = 1; iter <= 1000; ++iter) {
      cout << "\n\n=" << iter << "=================\n\n" << endl;
      ps.step(OPhysConfig::dt);
      cout << "==== finished iter " << iter << endl;
    }

  } catch (const GRBException &e) {
    cout << e.getErrorCode() << ' ' << e.getMessage() << endl;
  }

  return 0;
}
