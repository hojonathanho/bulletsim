#include "config_bullet.h"
btVector3 BulletConfig::gravity = btVector3(0,0,-1);
float BulletConfig::dt = .01;
int BulletConfig::maxSubSteps = 200;
float BulletConfig::internalTimeStep = 1/200.;
float BulletConfig::friction = .5;
float BulletConfig::restitution = 0;
