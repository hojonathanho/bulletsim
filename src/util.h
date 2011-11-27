#ifndef __UTIL_H__
#define __UTIL_H__

#include <LinearMath/btTransform.h>
#include <osg/Vec3d>
#include <openrave/openrave.h>
#include <string>
#include <fstream>
#include <iostream>
#include <vector>
using namespace std;

namespace util {


// reads input from haptic devices (using getDeviceState),
// and then transforms the rotations/coordinates to our coordinate system
bool getHapticInput(btTransform &trans0, bool buttons0[2], btTransform &trans1, bool buttons1[2]);



  ///////////////// CONVERSIONS ////////////////////////////

inline osg::Vec3d toOSGVector(const btVector3 &v) { return osg::Vec3d(v.x(), v.y(), v.z()); }
inline btVector3 toBtVector(const osg::Vec3d &v) { return btVector3(v.x(), v.y(), v.z()); }
inline btVector3 toBtVector(const OpenRAVE::Vector &v) { return btVector3(v.x, v.y, v.z); }
inline btTransform toBtTransform(const OpenRAVE::Transform &t) {
    return btTransform(btQuaternion(t.rot.y, t.rot.z, t.rot.w, t.rot.x), toBtVector(t.trans));
}
inline btTransform toBtTransform(const OpenRAVE::Transform &t, btScalar scale) {
    return btTransform(btQuaternion(t.rot.y, t.rot.z, t.rot.w, t.rot.x), scale * toBtVector(t.trans));
}

inline OpenRAVE::Vector toRaveQuaternion(const btQuaternion &q) {
    return OpenRAVE::Vector(q.w(), q.x(), q.y(), q.z());
}
inline OpenRAVE::Vector toRaveVector(const btVector3 &v) {
    return OpenRAVE::Vector(v.x(), v.y(), v.z());
}
inline OpenRAVE::Transform toRaveTransform(const btTransform &t) {
    return OpenRAVE::Transform(toRaveQuaternion(t.getRotation()), toRaveVector(t.getOrigin()));
}
inline OpenRAVE::Transform toRaveTransform(const btTransform &t, btScalar scale) {
    return OpenRAVE::Transform(toRaveQuaternion(t.getRotation()), toRaveVector(scale * t.getOrigin()));
}

#define CONVERT_VECTOR3(cls,v,w) cls v(w[0],w[1],w[2])
#define CONVERT_VECTOR4(cls,v,w) cls v(w[0],w[1],w[2],w[3])

  ///////////////// FILE IO ////////////////////////////


template <class T>
void read_2d_array(vector< vector<T> >& arr, string fname) {

  ifstream infile(fname.c_str());
  string line;
  arr.clear();
  while (getline(infile,line)) {
    stringstream ss (stringstream::in | stringstream::out);
    ss << line;
    vector<T> v;
    v.clear();
    while (ss) {
      T f;
      ss >> f;
      v.push_back(f);
    }
    arr.push_back(v);
  }
}

template <class T>
void read_1d_array(vector<T>& arr, string fname) {
  ifstream infile(fname.c_str());
  T i;
  arr.clear();
  while (infile) {
    infile >>i;
    arr.push_back(i);
  }
}

template <class T>
void write_1d_array(const vector<T>& arr, string fname) {
  ofstream outfile(fname.c_str());
  for (int i=0; i < arr.size(); i++) {
    outfile << arr[i] << " ";
  }
  outfile << "\n";
  outfile.close();
}



template <class T>
void write_2d_array(const vector< vector<T> >& arr, string fname) {

  ofstream outfile(fname.c_str());

  for (int i=0; i < arr.size(); i++) {
    vector<T> v = arr[i];
    for (int j=0; j < v.size(); v++) {
      outfile << v[j] << " ";
    }
    outfile << "\n";
  }
  outfile.close();
}

template <class T>
void write_vector(vector<T>& vec, string fname) {
  ofstream outfile(fname.c_str());

  for (int i=0; i < vec.size(); i++) {
    outfile << vec[i] << endl;
  }
  outfile.close();
}

template <class T>
void read_vector(vector<T>& vec, string fname) {
  vec.clear();
  ifstream infile(fname.c_str());
  T i;
  while (infile) {
    infile >>i;
    vec.push_back(i);
  }
}

template <class VecT, int n>
void print_vector(const VecT& vec) {
  for (int i=0; i < n; i++) {
    cout << vec[i] << " ";
  }
  cout << endl;
}

template <class MatT, int n>
void print_matrix(const MatT& mat) {
  for (int i=0; i < n; i++) {
    for (int j=0; j < n; j++) cout << mat[i][j] << " ";
    cout << endl;
  }
}


  ///////////// PRINTING //////////////////


// ostream &operator<<( ostream &out, btVector3 &v ) {
//   out << "[" << v.getX() << " " << v.getY() << " " << v.getZ() << "]";
// }

#define DPRINT(EX) cout << #EX << ": " << EX << endl; 
#define QQ << " " <<


}

#endif // __UTIL_H__
