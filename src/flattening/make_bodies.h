#pragma once
#include "simulation/basicobjects.h"
#include "simulation/softbodies.h"

BulletObject::Ptr makeTable(const vector<btVector3>& corners, float thickness);
BulletSoftObject::Ptr makeTowel(const vector<btVector3>& points, btSoftBodyWorldInfo&);
BulletSoftObject::Ptr makeSelfCollidingTowel(const btVector3& center, btScalar lenx, btScalar leny, int resx, int resy, btSoftBodyWorldInfo& worldInfo);
BulletSoftObject::Ptr makeBoxFromGrid(const vector<btVector3>& points, const btVector3 &thickness,
    int resx, int resy, int resz, btSoftBodyWorldInfo& worldInfo);
