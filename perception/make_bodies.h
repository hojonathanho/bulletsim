#pragma once
#include "basicobjects.h"
#include "softbodies.h"

BulletObject::Ptr makeTable(const vector<btVector3>& corners, float thickness);
BulletSoftObject::Ptr makeTowel(const vector<btVector3>& points, btSoftBodyWorldInfo&);
