#pragma once
#include "simulation/basicobjects.h"
#include "simulation/softbodies.h"

BulletObject::Ptr makeTable(const vector<btVector3>& corners, float thickness);
BulletSoftObject::Ptr makeTowel(const vector<btVector3>& points, btSoftBodyWorldInfo&);
BulletSoftObject::Ptr makeSelfCollidingTowel(const vector<btVector3>& points, btSoftBodyWorldInfo&);
BulletSoftObject::Ptr makeTetraBox(const vector<btVector3>& points, btScalar thickness, btSoftBodyWorldInfo&);
BulletSoftObject::Ptr makeBoxFromGrid(const vector<btVector3>& points, const btVector3 &thickness,
    int resx, int resy, int resz, btSoftBodyWorldInfo& worldInfo);
