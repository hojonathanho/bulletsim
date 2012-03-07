#pragma once
// reading bullet vectors, and converting between them and vectors

#include <btBulletDynamicsCommon.h>
#include <vector>
#include <iostream>
using namespace std;

ostream &operator<<(ostream &stream, const btVector3& v);

ostream &operator<<(ostream &stream, const btQuaternion& v);

ostream &operator<<(ostream &stream, const btTransform& v);

ostream &operator<<(ostream &stream, const vector<btVector3>& vs);

