#include "table.h"
#include "cloud_ops.h"
#include <Eigen/Dense>
#include <boost/foreach.hpp>
#include <cmath>
using namespace Eigen;
using namespace std;

static const float MIN_HEIGHT = .5;
static const float MAX_HEIGHT = 1.5;
static const float HIST_RES = .004;

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

float getTableHeight(ColorCloudPtr cloud) {
  MatrixXf xyz = toEigenMatrix(cloud);
  VectorXf z = clipSet(xyz.block(0,2,xyz.rows(), 1),MIN_HEIGHT,MAX_HEIGHT);
  VectorXi counts;
  VectorXf binedges;
  makeHistogram(z, HIST_RES, counts, binedges);
  int iMax;
  counts.maxCoeff(&iMax);
  return binedges(iMax) + res/2;
}
