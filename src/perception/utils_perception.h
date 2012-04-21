#pragma once
#include <LinearMath/btTransform.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <osg/Vec3d>
#include "utils/my_assert.h"
#include "utils/conversions.h"
using namespace std;

inline std::vector<btVector3> toBulletVectors(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud) {
  std::vector<btVector3> out(cloud->size());
  int i=0;
  BOOST_FOREACH(pcl::PointXYZRGBA& point, *cloud) {
    out[i] = btVector3(point.x, point.y, point.z);
    i++;
  }
  return out;
}


template<class T, class S>
vector<T> operator*(const vector<T>& xs, S p) {
  vector<T> ys;
  ys.resize(xs.size());
  for (int i=0; i<xs.size(); i++) ys[i] = xs[i]*p;
  return ys;
}

template<class T, class S>
vector<T> operator*(S p, vector<T>& xs) {
  vector<T> ys;
  ys.resize(xs.size());
  for (int i=0; i<xs.size(); i++) ys[i] = xs[i]*p;
  return ys;
}

vector<btVector3> operatorTimes(const btTransform& t, const vector<btVector3>& vecs);
vector<Eigen::Vector3f> operatorTimes(const Eigen::Affine3f& t, const vector<Eigen::Vector3f>& vecs);

Eigen::Affine3f Scaling3f(float s);

//Eigen::Affine3f getCamToWorldFromTable(const vector<Vector3f>& corners);
btTransform getCamToWorldFromTable(const vector<btVector3>& corners);

class CoordinateTransformer {
public:
  CoordinateTransformer(const btTransform& wfc);
  btTransform worldFromCamUnscaled;
  Eigen::Affine3f worldFromCamEigen;
  inline btVector3 toWorldFromCam(const btVector3& camVec);
  inline btVector3 toCamFromWorld(const btVector3& camVec);
  vector<btVector3> toWorldFromCamN(const vector<btVector3>& camVecs);
  vector<btVector3> toCamFromWorldN(const vector<btVector3>& worldVecs);
  void reset(const btTransform &wfc);
};

struct OSGCamParams {
  osg::Vec3d eye;
  osg::Vec3d center;
  osg::Vec3d up;
  OSGCamParams(const btTransform& toWorldFromCam, float scale);
};


inline bool isFinite(const Eigen::MatrixXf& x) {
  for (int row=0; row < x.rows(); row++) for (int col=0; col<x.cols(); col++) if (!isfinite(x(row,col))) return false;
  return true;
}

inline float sq(float x) {return x*x;}


