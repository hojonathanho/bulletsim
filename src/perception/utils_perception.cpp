#include "utils_perception.h"
#include <boost/foreach.hpp>
#include <pcl/io/pcd_io.h>
#include <algorithm>
#include "utils/config.h"
#include "simulation/util.h"

using namespace Eigen;

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
  up = -util::toOSGVector(rotation.getColumn(1));
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
