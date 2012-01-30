#pragma once
#include "basicobjects.h"
#include "softbodies.h"

BulletObject::Ptr makeTable(const vector<btVector3>& corners, float thickness);
BulletSoftObject::Ptr makeTowel(const vector<btVector3>& points, btSoftBodyWorldInfo&);
BulletSoftObject::Ptr makeSelfCollidingTowel(const vector<btVector3>& points, btSoftBodyWorldInfo&);
BulletSoftObject::Ptr makeTetraBox(const vector<btVector3>& points, btScalar thickness, btSoftBodyWorldInfo&);
