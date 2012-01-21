#include "bullet_io.h"

ostream &operator<<(ostream &stream, btVector3& v) {
  // for some reason i have to do this, else i get a segfault
  float x = v.getX();
  float y = v.getY();
  float z = v.getZ();
  cout << "hi!" << endl;
  stream << x << " " << y << " " << z;
}

ostream &operator<<(ostream &stream, btQuaternion& v) {
  float x = v.getX();
  float y = v.getY();
  float z = v.getZ();
  float w = v.getW();
  stream << x << " " << y << " " << z << " " << w;
}

ostream &operator<<(ostream &stream, btTransform& v) {
  btQuaternion rotation = v.getRotation();
  btVector3 origin = v.getOrigin();
  stream << origin << ", " << rotation << endl;
}

ostream &operator<<(ostream &stream, vector<btVector3>& vs) {
  for (int i=0; i < vs.size(); i++) stream << vs[i] << endl;
}
