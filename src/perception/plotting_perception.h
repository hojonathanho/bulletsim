#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "simulation/plotting.h"
#include "perception/sparse_array.h"

class PointCloudPlot : public PlotPoints {
public:
  typedef boost::shared_ptr<PointCloudPlot> Ptr;  
  PointCloudPlot(int size=2) : PlotPoints(size) {}
  void setPoints1(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, float alpha=1);
};

void drawCorrLines(PlotLines::Ptr lines, const vector<btVector3>& est, const vector<btVector3>& obs, const SparseArray&);
void plotNodesAsSpheres(btSoftBody* psb, const Eigen::VectorXf& pVis, const Eigen::VectorXf& sigs, PlotSpheres::Ptr spheres);
void plotObs(const Eigen::MatrixXf& corr, const vector<btVector3>& obsPts, PointCloudPlot::Ptr plot);
