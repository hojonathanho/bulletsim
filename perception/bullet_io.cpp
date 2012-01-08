#include "bullet_io.h"
#include <boost/foreach.hpp>

vector<btVector3> btFromVecVec(vector< vector<float> > vv) {
  vector<btVector3> out;
  out.reserve(vv.size());
  BOOST_FOREACH(vector<float> v, vv) {
    assert(v.size() == 3);
    out.push_back(v);
  }
  return out;
}

ostream &operator<<(ostream &stream, btVector3& v) {
  stream << v.getX() << " " << v.getY() << " " << v.getZ();
}

ostream &operator<<(ostream &stream, btQuaternion& v) {
  stream << v.getX() << " " << v.getY() << " " << v.getZ() << " " << v.getW();
}

ostream &operator<<(ostream &stream, btTransform& v) {
  stream << v.getRotation() << ", " << v.getOrigin();
}

