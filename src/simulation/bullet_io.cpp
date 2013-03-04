#include "bullet_io.h"

ostream &operator<<(ostream &stream, const btVector3& v) {
  // for some reason i have to do this, else i get a segfault
  float x = v.getX();
  float y = v.getY();
  float z = v.getZ();
  stream << x << " " << y << " " << z;
  return stream;
}

ostream &operator<<(ostream &stream, const btQuaternion& v) {
  float x = v.getX();
  float y = v.getY();
  float z = v.getZ();
  float w = v.getW();
  stream << x << " " << y << " " << z << " " << w;
  return stream;
}

ostream &operator<<(ostream &stream, const btTransform& v) {
  btQuaternion rotation = v.getRotation();
  btVector3 origin = v.getOrigin();
  stream << rotation << " " << origin;
  return stream;
}

ostream &operator<<(ostream &stream, const vector<btVector3>& vs) {
  for (int i=0; i < vs.size(); i++) stream << vs[i] << endl;
  return stream;
}

ostream &operator<<(ostream &stream, const btMatrix3x3& m) {
  stream << m.getRow(0) << " ";
  stream << m.getRow(1) << " ";
  stream << m.getRow(2);
  return stream;
}

istream &operator>>(istream &stream, btVector3& v) {
  btScalar x,y,z;
  stream >> x >> y >> z;
  v = btVector3(x, y, z);
  return stream;
}

istream &operator>>(istream &stream, btQuaternion& v) {
  btScalar x,y,z,w;
  stream >> x >> y >> z >> w;
  v = btQuaternion(x, y, z, w);
  return stream;
}

istream &operator>>(istream &stream, btTransform& v) {
  btQuaternion rotation;
  btVector3 origin;
  stream >> rotation >> origin;
  v = btTransform(rotation, origin);
  return stream;
}

istream &operator>>(istream &stream, btMatrix3x3& m) {
  btVector3 r1,r2,r3;
  stream >> r1 >> r2 >> r3;
  m = btMatrix3x3(r1.x(), r1.y(), r1.z(),
		  r2.x(), r2.y(), r2.z(),
		  r3.x(), r3.y(), r3.z());
  return stream;
}
