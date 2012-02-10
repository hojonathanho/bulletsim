#pragma once
#include <btBulletDynamicsCommon.h>
#include "simulation/softbodies.h"
#include "simulation/rope.h"

void applyImpulses(const vector<btVector3>&, CapsuleRope::Ptr);
void applyImpulses(const vector<btVector3>&, BulletSoftObject::Ptr);
