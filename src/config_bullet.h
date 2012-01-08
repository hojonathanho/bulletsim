#include "config.h"
#include <btBulletDynamicsCommon.h>

struct BulletConfig : Config {
  static btVector3 gravity;
  static float dt;
  static int maxSubSteps;
  static float internalTimeStep;
  static float friction;
  static float restitution;

  BulletConfig() : Config() {
    params.push_back(new Parameter<float>("gravity", &gravity.m_floats[2], "gravity (z component)")); 
    params.push_back(new Parameter<float>("dt", &dt, "timestep for fixed-step simulations")); 
    params.push_back(new Parameter<int>("maxSubSteps", &maxSubSteps, "maximum Bullet internal substeps per simulation step"));
    params.push_back(new Parameter<float>("internalTimeStep", &internalTimeStep, "internal Bullet timestep"));
    params.push_back(new Parameter<float>("friction", &friction, "default friction coefficient for rigid bodies"));
    params.push_back(new Parameter<float>("restitution", &restitution, "default restitution coefficient for rigid bodies"));
  }

};
