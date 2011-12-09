#pragma once
#include <LinearMath/btTransform.h>
#include <pcl/common/eigen.h>
#include <pcl/common/eigen.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <boost/foreach.hpp>
#include <vector>
#include <string>
using namespace Eigen;
using namespace std;

void read_btVectors(vector<btVector3>& out, const string& fname) {
  float x,y,z;
  out.clear();
  ifstream infile(fname.c_str());
  while (infile) {
    infile >> x >> y >> z;
    if (infile) {
      out.push_back(btVector3(x,y,z));
    }
  }
  assert(out.size() > 0);
}


vector<btVector3> read_btVectors(const string& fname) {
  vector<btVector3> out;
  float x,y,z;
  out.clear();
  ifstream infile(fname.c_str());
  while (infile) {
    infile >> x >> y >> z;
    if (infile) {
      out.push_back(btVector3(x,y,z));
    }
  }
  //assert(out.size() > 0);
  return out;
}


vector<btVector3> transform_btVectors(const vector<btVector3>& ins, btTransform tf) {
  vector<btVector3> outs;
  BOOST_FOREACH(btVector3 vec, ins) outs.push_back(tf*vec);
  return outs;
}

btTransform toBTTransform(Affine3f t) {
  btTransform out;
  out.setFromOpenGLMatrix(t.data());
  return out;
}

MatrixXf toEigens(const vector<btVector3>& vecs) {
  MatrixXf out = MatrixXf(vecs.size(),3);
  for (int i=0; i < vecs.size(); i++) {
    //out.row(i).readArray(vecs[i].m_floats);
    out(i,0) = vecs[i].getX();
    out(i,1) = vecs[i].getY();
    out(i,2) = vecs[i].getZ();
  }
  return out;
}

vector<btVector3> toBTs(vector<Vector3f>& vecs) {
  vector<btVector3> out;
  BOOST_FOREACH(Vector3f vec, vecs) out.push_back(btVector3(vec[0],vec[1],vec[2]));
  return out;
}

btVector3 toBT(Vector3f& v) {
  return btVector3(v[0],v[1],v[2]);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr readPCD(string pcdfile) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcdfile, *cloud) == -1) {
    PCL_ERROR(("couldn't read file " + pcdfile + "\n").c_str());
    throw;
    }
  return cloud;
}
