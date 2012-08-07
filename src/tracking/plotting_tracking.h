#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "simulation/plotting.h"
#include "clouds/utils_pcl.h"
#include "sparse_utils.h"

class PointCloudPlot : public PlotPoints {
public:
  typedef boost::shared_ptr<PointCloudPlot> Ptr;  
  PointCloudPlot(int size=2) : PlotPoints(size) {}
  void setPoints1(const pcl::PointCloud<ColorPoint>::Ptr& cloud, float alpha=1);
};

void drawCorrLines(PlotLines::Ptr lines, const vector<btVector3>& est, const vector<btVector3>& obs, const SparseMatrixf&,  int aPtInd=-1);
// draw all the lines between aPts and Pts that have a corr>threshold.
// if aPtInd is in the range of aPts, then draw only the lines that comes from aPts[aPtInd]
void drawCorrLines(PlotLines::Ptr lines, const vector<btVector3>& aPts, const vector<btVector3>& bPts, const Eigen::MatrixXf& corr, float threshold, int aPtInd=-1);
void plotNodesAsSpheres(btSoftBody* psb, const Eigen::VectorXf& pVis, const Eigen::VectorXf& sigs, PlotSpheres::Ptr spheres);
void plotNodesAsSpheres(btSoftBody* psb, const Eigen::VectorXf& pVis, const Eigen::VectorXf& sigs, PlotSpheres::Ptr spheres);
void plotNodesAsSpheres(const vector<btVector3>& nodes, const Eigen::VectorXf& pVis, const Eigen::VectorXf& sigs, PlotSpheres::Ptr spheres);
void plotNodesAsSpheres(const Eigen::MatrixXf nodes, const Eigen::VectorXf& pVis, const Eigen::MatrixXf& sigs, PlotSpheres::Ptr spheres);
void plotNodesAsSpheres(const Eigen::MatrixXf centers, const Eigen::MatrixXf colors, const Eigen::VectorXf& pVis, const Eigen::MatrixXf& stdev, PlotSpheres::Ptr spheres);
void plotObs(const vector<btVector3>& obsPts, const Eigen::VectorXf& inlierFrac, PointCloudPlot::Ptr plot);
void plotObsBorder(const Eigen::MatrixXf cloud, PointCloudPlot::Ptr plot);
void plotObs(const Eigen::MatrixXf cloud, PointCloudPlot::Ptr plot);
void plotObs(const Eigen::MatrixXf centers, const Eigen::MatrixXf colors, PointCloudPlot::Ptr plot);
