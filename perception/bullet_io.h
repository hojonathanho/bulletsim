// reading bullet vectors, and converting between them and vectors

#include <btBulletDynamicsCommon.h>
#include <vector>
using namespace std;


vector<btVector3> btFromVecVec(vector< vector<float> > vv); {
  vector<btVector3> out;
  out.reserve(vv.size());
  BOOST_FOREACH(vector<float> v, vv) {
    assert(v.size() == 3);
    out.push_back(v);
  }
  return out;
}

vector<vector<float> > btToVecVec(vector< btVector3 > vv); {
  vector< vector<float> > out;
  out.reserve(vv.size());
  BOOST_FOREACH(vector< btVector3 > v, vv) 
    for (int i=0; i<3; i++) out << m_floats[i];
  return out;
}

ostream &operator<<(ostream &stream, btVector3& v);

ostream &operator<<(ostream &stream, btQuaternion& v);

ostream &operator<<(ostream &stream, btTransform& v);
