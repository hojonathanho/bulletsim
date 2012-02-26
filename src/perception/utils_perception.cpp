#include "utils_perception.h"
#include <boost/foreach.hpp>
#include <pcl/io/pcd_io.h>
#include <algorithm>
#include "utils/config.h"
#include "simulation/util.h"

using namespace Eigen;


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

vector<btVector3> toBulletVectors(const MatrixXf& in) {
  vector<btVector3> out(in.rows());
  for (int row=0; row < in.rows(); row++)
    out[row] = btVector3(in(row,0), in(row,1), in(row,2));      
  return out;
}

vector< vector<float> > toVecVec(const vector<btVector3>& in) {
  vector< vector<float> > out(in.size());
  for (int i=0; i < in.size(); i++) {
    vector<float> row(3);
      for (int j=0; j<3; j++) {
	row[j] = in[i].m_floats[j];
      }
    out[i] = row;
  }
  return out;
}

vector<float> toVec(const Eigen::VectorXf& in) {
  vector<float> out(in.rows());
  for (int i=0; i < out.size(); i++) out[i] = in[i];
  return out;
}

VectorXf toVectorXf(const vector<float>& in) {
  VectorXf out(in.size());
  for (int i=0; i<in.size(); i++) out[i] = in[i];
  return out;
}

btTransform toBulletTransform(const Eigen::Affine3f& affine) {
  Vector3f transEig = affine.translation();
  Matrix3f rotEig = affine.rotation();
  Quaternionf quatEig = Quaternionf(rotEig);
  btVector3 transBullet = toBulletVector(transEig);
  btQuaternion quatBullet = btQuaternion(quatEig.x(), quatEig.y(), quatEig.z(), quatEig.w());
  return btTransform(quatBullet,transBullet);
}

Affine3f toEigenTransform(const btTransform& transform) {
  btVector3 transBullet = transform.getOrigin();
  btQuaternion quatBullet = transform.getRotation();
  Translation3f transEig = Translation3f(toEigenVector(transBullet));
  Matrix3f rotEig = Quaternionf(quatBullet.w(),quatBullet.x(),quatBullet.y(),quatBullet.z()).toRotationMatrix();
  Affine3f out = transEig*rotEig;
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

MatrixXf toEigenMatrix(const vector< vector<float> >& in) {
  ENSURE(in.size() > 1) ;
  MatrixXf out(in.size(),in[0].size()); 
  for (int i=0; i<in.size(); i++) 
    for (int j=0; j<in[0].size(); j++)
      out(i,j) = in[i][j];
  return out;
}

vector<btVector3> operatorTimes(const btTransform& t, const vector<btVector3>& in) {
  vector<btVector3> out(in.size()); 
  for (int i=0; i<in.size(); i++) out[i] = t*in[i];
  return out;
}


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
  if (newZ.z() > 0) {
    newZ *= -1;
    newX *= -1;
  }
  newX.normalize(); newY.normalize(); newZ.normalize();
  rotWorldToCamT[0] = newX;
  rotWorldToCamT[1] = newY;
  rotWorldToCamT[2] = newZ;
  btMatrix3x3 rotCamToWorld = rotWorldToCamT.transpose().inverse();
  float tz = 1-(rotCamToWorld*corners[0]).z();
  return btTransform(rotCamToWorld, btVector3(0,0,tz));
}

CoordinateTransformer::CoordinateTransformer(const btTransform& wfc) { reset(wfc); }

void CoordinateTransformer::reset(const btTransform &wfc) {
  worldFromCamUnscaled = wfc;
  worldFromCamEigen = Scaling3f(GeneralConfig::scale)*toEigenTransform(wfc);
}

inline btVector3 CoordinateTransformer::toWorldFromCam(const btVector3& camVec) {
  return METERS * (worldFromCamUnscaled * camVec);
}

inline btVector3 CoordinateTransformer::toCamFromWorld(const btVector3& worldVec) {
  return worldFromCamUnscaled.inverse() * ( worldVec / METERS);
}

vector<btVector3> CoordinateTransformer::toWorldFromCamN(const vector<btVector3>& camVecs) {
  vector<btVector3> worldVecs(camVecs.size());
  for (int i=0; i<camVecs.size(); i++) worldVecs[i] = toWorldFromCam(camVecs[i]);
  return worldVecs;
}

vector<btVector3> CoordinateTransformer::toCamFromWorldN(const vector<btVector3>& worldVecs) {
  vector<btVector3> camVecs(worldVecs.size());
  for (int i=0; i<worldVecs.size(); i++) camVecs[i] = toCamFromWorld(worldVecs[i]);
  return camVecs;
}

OSGCamParams::OSGCamParams(const btTransform& toWorldFromCam, float scale) {
    btMatrix3x3 rotation = toWorldFromCam.getBasis();
    btVector3 translation = toWorldFromCam.getOrigin();
    eye = util::toOSGVector(translation*scale);
    center = util::toOSGVector(rotation.getColumn(2));
    up = util::toOSGVector(rotation.getColumn(1));
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
