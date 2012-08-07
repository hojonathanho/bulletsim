#include "table.h"
#include "cloud_ops.h"
#include <Eigen/Dense>
#include <fstream>
#include <boost/foreach.hpp>
#include <pcl/surface/convex_hull.h>
#include <cmath>
using namespace std;
using namespace Eigen;

static const float MIN_HEIGHT=.2; // min height of table
static const float MAX_HEIGHT=100; // max height of table
static const float HIST_RES = .005; // histogram resolution for table finding
static const float TABLE_POINTS_TOLERANCE=.025; // tolerance for making points part of table

template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

ColorCloudPtr getPointsOnTableHull(ColorCloudPtr cloud, ColorCloudPtr hull, vector<pcl::Vertices> polys, float height) {
  ColorCloudPtr cloudOnTable = filterZ(cloud, height, 100);
  ColorCloudPtr cropped = cropToHull(cloudOnTable, hull, polys);
  return cropped;
}

int leastIntGreaterThan(float x) {
  if (ceil(x) == x) return (x+1);
  else return ceil(x);
}

void makeHistogram(const VectorXf& vals, float res, VectorXi& counts, VectorXf& binedges) {
  float low = vals.minCoeff();
  float high = vals.maxCoeff();
  int nbins = leastIntGreaterThan( (high - low) / res);

  counts = VectorXi::Zero(nbins);
  binedges = VectorXf::LinSpaced(nbins+1, low, low + res*nbins);

  for (int i=0; i < vals.size(); i++) {
    int bin = floor((vals(i)  - low) / res);
    counts(bin) += 1;
  }
}

VectorXf clipSet(const VectorXf& in, float low, float high) {
  vector<float> vals;
  for (int i=0; i < in.size(); i++) {
    float x = in(i);
    if (isfinite(x) && x >= low && x <= high) vals.push_back(x);
  }

  VectorXf out(vals.size());
  for (int i=0; i < vals.size(); i++) out(i) = vals[i];
  return out;
}

float getTableHeight(ColorCloudPtr cloud) {
  MatrixXf xyz = toEigenMatrix(cloud);
  VectorXf z = clipSet(xyz.block(0,2,xyz.rows(), 1),MIN_HEIGHT,MAX_HEIGHT);
  if (z.size() < 100) throw runtime_error("getTableHeight: no points at appropriate height");
  float res = HIST_RES;
  VectorXi counts;
  VectorXf binedges;
  makeHistogram(z, res, counts, binedges);
  int iMax;
  counts.maxCoeff(&iMax);
  if (counts.maxCoeff() < 30) throw runtime_error("getTableHeight: not enough points at table height");
  return binedges(iMax) + res/2;

}

ColorCloudPtr getTablePoints(ColorCloudPtr cloud, float height) {
  MatrixXf xyz = toEigenMatrix(cloud);
  vector<int> inds;
  for (int i=0; i < cloud->size(); i++)
    if (fabs(cloud->points[i].z - height) < TABLE_POINTS_TOLERANCE)
      inds.push_back(i);
  return extractInds(cloud, inds);
}

void getTableXYBounds(ColorCloudPtr cloud, float& xmin, float& xmax, float& ymin, float& ymax) {
  MatrixXf xyz = toEigenMatrix(cloud);
  VectorXf x = xyz.col(0);
  VectorXf y = xyz.col(1);
  xmin = x.minCoeff();
  xmax = x.maxCoeff();
  ymin = y.minCoeff();
  ymax = y.maxCoeff();
}
