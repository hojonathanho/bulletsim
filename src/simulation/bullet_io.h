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

ostream &operator<<(ostream &stream, const btMatrix3x3& m);

istream &operator>>(istream &stream, btVector3& v);

istream &operator>>(istream &stream, btQuaternion& v);

istream &operator>>(istream &stream, btTransform& v);

istream &operator>>(istream &stream, btMatrix3x3& v);
