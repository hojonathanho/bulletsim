// reading bullet vectors, and converting between them and vectors

#include <btBulletDynamicsCommon.h>
#include <vector>
#include <iostream>
using namespace std;

ostream &operator<<(ostream &stream, btVector3& v);

ostream &operator<<(ostream &stream, btQuaternion& v);

ostream &operator<<(ostream &stream, btTransform& v);
