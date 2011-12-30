#pragma once
#include <LinearMath/btTransform.h>
#include <pcl/common/eigen.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <string>
using namespace Eigen;
using namespace std;

void read_btVectors(vector<btVector3>& out, const string& fname);

vector<btVector3> read_btVectors(const string& fname);

vector<btVector3> transform_btVectors(const vector<btVector3>& ins, btTransform tf);

  btTransform toBTTransform(Affine3f t);

MatrixXf toEigens(const vector<btVector3>& vecs);

vector<btVector3> toBTs(vector<Vector3f>& vecs);

btVector3 toBT(Vector3f& v);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr readPCD(string pcdfile);

template<class T, class S>
vector<T> operator*(vector<T>& xs, S p) {
  vector<T> ys;
  ys.resize(xs.size());
  for (int i=0; i<xs.size(); i++) ys[i] = xs[i]*p;
  return ys;
}

Affine3f scaling(float s);

void verts2boxPars(const vector<btVector3>& verts, btVector3& halfExtents, btVector3& origin, btScalar thickness);
