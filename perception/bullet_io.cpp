#include "bullet_io.h"

ostream &operator<<(ostream &stream, btVector3& v) {
  stream << v.getX() << " " << v.getY() << " " << v.getZ();
}

ostream &operator<<(ostream &stream, btQuaternion& v) {
  stream << v.getX() << " " << v.getY() << " " << v.getZ() << " " << v.getW();
}

ostream &operator<<(ostream &stream, btTransform& v) {
  stream << v.getRotation() << ", " << v.getOrigin();
}

