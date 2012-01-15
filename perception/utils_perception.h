#pragma once
#include <LinearMath/btTransform.h>
#include <pcl/common/eigen.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <string>
using namespace std;

vector<btVector3> toBulletVectors(const vector< vector<float> >&);
vector<btVector3> toBulletVectors(const vector< Eigen::Vector3f >&);
vector<btVector3> toBulletVectors(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);

btTransform toBulletTransform(Eigen::Affine3f);

vector<Eigen::Vector3f> toEigenVectors(const vector< vector<float> >&);
vector<Eigen::Vector3f> toEigenVectors(const vector<btVector3>&);
vector<Eigen::Vector3f> toEigenVectors(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);

Eigen::MatrixXf toEigenMatrix(const vector< vector<float> >&);
Eigen::MatrixXf toEigenMatrix(const vector<btVector3>&);

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
  CoordinateTransformer(btTransform wfc) : worldFromCamUnscaled(wfc) {}
  btTransform worldFromCamUnscaled;
  inline btVector3 toWorldFromCam(const btVector3& camVec);
  vector<btVector3> toWorldFromCamN(const vector<btVector3>& camVecs);
};
