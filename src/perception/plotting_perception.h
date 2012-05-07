#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "simulation/plotting.h"
#include "perception/sparse_array.h"
#include "clouds/utils_pcl.h"

class PointCloudPlot : public PlotPoints {
public:
  typedef boost::shared_ptr<PointCloudPlot> Ptr;  
  PointCloudPlot(int size=2) : PlotPoints(size) {}
  void setPoints1(const pcl::PointCloud<ColorPoint>::Ptr& cloud, float alpha=1);
};

void drawCorrLines(PlotLines::Ptr lines, const vector<btVector3>& est, const vector<btVector3>& obs, const SparseArray&);
void plotNodesAsSpheres(btSoftBody* psb, const Eigen::VectorXf& pVis, const Eigen::VectorXf& sigs, PlotSpheres::Ptr spheres);
void plotNodesAsSpheres(btSoftBody* psb, const Eigen::VectorXf& pVis, const Eigen::VectorXf& sigs, PlotSpheres::Ptr spheres);
void plotNodesAsSpheres(const vector<btVector3>& nodes, const Eigen::VectorXf& pVis, const Eigen::VectorXf& sigs, PlotSpheres::Ptr spheres);
void plotObs(const vector<btVector3>& obsPts, const Eigen::VectorXf& inlierFrac, PointCloudPlot::Ptr plot);
