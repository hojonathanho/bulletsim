#include "utils_perception.h"
#include <boost/foreach.hpp>
#include <pcl/io/pcd_io.h>
#include <algorithm>

inline btVector3 toBulletVector(const vector<float>& vec) {return btVector3(vec[0],vec[1],vec[2]);}
inline btVector3 toBulletVector(const Vector3f& vec) {return btVector3(vec[0],vec[1],vec[2]);}
inline Vector3f toEigenVector(const vector<float>& vec) {return Vector3f(vec[0],vec[1],vec[2]);}
inline Vector3f toEigenVector(const btVector3& vec) {return Vector3f(vec.x(),vec.y(),vec.z());}

vector<btVector3> toBulletVectors(const vector< vector<float> >& in) {
  vector<btVector3> out(in.size());
  for (int i=0; i<in.size(); i++) out[i] = toBulletVector(in[i]);
  return out;
}

vector<btVector3> toBulletVectors(const vector< Vector3f >& in) {
  vector<btVector3> out(in.size());
  for (int i=0; i<in.size(); i++) out[i] = toBulletVector(in[i]);
  return out;
}

vector<btVector3> toBulletVectors(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
  vector<btVector3> out(cloud->size());
  int i=0;
  BOOST_FOREACH(pcl::PointXYZRGB& point, *cloud) {
    out[i] = btVector3(point.x, point.y, point.z);
    i++;
  }
  return out;
}


vector<Vector3f> toEigenVectors(const vector< vector<float> >&);
vector<Vector3f> toEigenVectors(const vector<btVector3>& in) {
  vector<Vector3f> out(in.size());
  for (int i=0; i<in.size(); i++) out[i] = toEigenVector(in[i]);
  return out;
}

MatrixXf toEigenMatrix(const vector<btVector3>& in) {
  MatrixXf out(in.size(), 3);
  for (int i=0; i<in.size(); i++) out.row(i) = toEigenVector(in[i]);
  return out;
}


vector<btVector3> operatorTimes(const btTransform& t, const vector<btVector3>& in) {
  vector<btVector3> out(in.size()); 
  for (int i=0; i<in.size(); i++) out[i] = t*in[i];
  return out;
}



// vector<btVector3> transform_btVectors(const vector<btVector3>& ins, btTransform tf) {
//   vector<btVector3> outs;
//   BOOST_FOREACH(btVector3 vec, ins) outs.push_back(tf*vec);
//   return outs;
// }

// btTransform toBTTransform(Affine3f t) {
//   btTransform out;
//   out.setFromOpenGLMatrix(t.data());
//   return out;
// }

// MatrixXf toEigens(const vector<btVector3>& vecs) {
//   MatrixXf out = MatrixXf(vecs.size(),3);
//   for (int i=0; i < vecs.size(); i++) {
//     //out.row(i).readArray(vecs[i].m_floats);
//     out(i,0) = vecs[i].getX();
//     out(i,1) = vecs[i].getY();
//     out(i,2) = vecs[i].getZ();
//   }
//   return out;
// }

// vector<btVector3> toBTs(vector<Vector3f>& vecs) {
//   vector<btVector3> out;
//   BOOST_FOREACH(Vector3f vec, vecs) out.push_back(btVector3(vec[0],vec[1],vec[2]));
//   return out;
// }

// btVector3 toBT(Vector3f& v) {
//   return btVector3(v[0],v[1],v[2]);
// }


Affine3f Scaling3f(float s) {
  Affine3f T;
  T = s*Matrix3f::Identity();
  return T;
}

btTransform getCamToWorldFromTable(const vector<btVector3>& corners) {
  btMatrix3x3 rotWorldToCamT;
  btVector3 newY = corners[1] - corners[0];
  btVector3 newX = corners[3] - corners[0];
  btVector3 newZ = newX.cross(newY);
  newX.normalize(); newY.normalize(); newZ.normalize();
  rotWorldToCamT[0] = newX;
  rotWorldToCamT[1] = newY;
  rotWorldToCamT[2] = newZ;
  btMatrix3x3 rotCamToWorld = rotWorldToCamT.transpose().inverse();
  float tz = 1-(rotCamToWorld*corners[0]).z();
  return btTransform(rotCamToWorld, btVector3(0,0,tz));
}

// Affine3f getCamToWorldFromTable(const vector<Vector3f>& corners) {
//   btVector3 newY = corners[1] - corners[0];
//   btVector3 newX = corners[3] - corners[0];
//   btVector3 newZ = newX.cross(newY);
//   newX.normalize(); newY.normalize(); newZ.normalize();
//   Matrix3f rotWorldToCam;
//   rotWorldToCam.col(0) = newX;
//   rotWorldToCam.col(1) = newY;
//   rotWorldToCam.col(2) = newZ;
//   Affine3f rotCamToWorld = rotWorldToCam.inverse();
//   float tz = 1-(rotCamToWorld*corners[0]).z();
//   Affine3f trans = Translation3f(tz);
//   return trans * rotCamToWorld;
// }


