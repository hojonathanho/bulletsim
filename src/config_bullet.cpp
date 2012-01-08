#include "config_bullet.h"
btVector3 BulletConfig::gravity = btVector3(0,0,-9.8);
int BulletConfig::maxSubSteps = 200;
float BulletConfig::internalTimeStep = 1/200.;
float BulletConfig::friction = 1.;
float BulletConfig::restitution = 0;
