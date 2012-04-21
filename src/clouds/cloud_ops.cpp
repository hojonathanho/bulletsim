#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "cloud_ops.h"
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/surface/convex_hull.h>
#include <opencv2/imgproc/imgproc.hpp>

typedef pcl::PointXYZRGBA PointT;

using namespace std;
using namespace Eigen;

vector< vector<int> > findClusters(ColorCloudPtr cloud, float tol, float minSize) {
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
  pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
  ec.setClusterTolerance (tol);
  ec.setMinClusterSize (minSize);
  ec.setMaxClusterSize (2500000);
  ec.setSearchMethod (tree);
  ec.setInputCloud(cloud);
  ec.extract (cluster_indices);

  vector< vector<int> > out;

  for (int i=0; i < cluster_indices.size(); i++) {
    out.push_back(cluster_indices[i].indices);
  }
  return out;
}

ColorCloudPtr downsampleCloud(const ColorCloudPtr in, float sz) {
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr out(new pcl::PointCloud<pcl::PointXYZRGBA>());
  pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
  vg.setInputCloud(in);
  vg.setLeafSize(sz,sz,sz);
  vg.filter(*out);
  return out;
}

ColorCloudPtr removeOutliers(const ColorCloudPtr in, float thresh, int k) {
  ColorCloudPtr out(new ColorCloud());
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
  sor.setInputCloud (in);
  sor.setMeanK (k);
  sor.setStddevMulThresh (thresh);
  sor.filter (*out);
  cout << "removeOutliers: removed " << (in->size() - out->size()) << " of " << in->size() << endl;
  return out;
}

ColorCloudPtr projectOntoPlane(const ColorCloudPtr in, Eigen::Vector4f& coeffs) {
  ColorCloudPtr cloud_projected (new ColorCloud());

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  coefficients->values.resize (4);
  coefficients->values[0] = coefficients->values[1] = 0;
  coefficients->values[2] = 1.0;
  coefficients->values[3] = 0;

  // Create the filtering object
  pcl::ProjectInliers<pcl::PointXYZRGBA> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (in);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);
  return cloud_projected;
}

ColorCloudPtr findConcaveHull(ColorCloudPtr in, std::vector<pcl::Vertices>& polygons) {
  ColorCloudPtr out(new ColorCloud());
  pcl::ConcaveHull<PointT> chull;
  chull.setInputCloud (in);
  chull.setAlpha (0.1);
  chull.reconstruct (*out, polygons);
  return out;
}

ColorCloudPtr findConvexHull(ColorCloudPtr in, std::vector<pcl::Vertices>& polygons) {
  ColorCloudPtr out(new ColorCloud());
  pcl::ConvexHull<PointT> chull;
  chull.setInputCloud (in);
  chull.setDimension(2);
  chull.reconstruct (*out, polygons);
  return out;
}

ColorCloudPtr cropToHull(const ColorCloudPtr in, ColorCloudPtr hull_cloud, std::vector<pcl::Vertices>& polygons) {
  ColorCloudPtr out(new ColorCloud());
  pcl::CropHull<PointT> crop_filter;
  crop_filter.setInputCloud (in);
  crop_filter.setHullCloud (hull_cloud);
  crop_filter.setHullIndices (polygons);
  crop_filter.setDim (2);
  crop_filter.filter (*out);
  return out;
}

ColorCloudPtr filterHeight(ColorCloudPtr in, float low, float high) {
  pcl::PassThrough<pcl::PointXYZRGBA> pass;
  pass.setInputCloud (in);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (low, high);

  ColorCloudPtr out(new ColorCloud());
  pass.filter (*out);
  return out;
}

void fixZ(ColorCloudPtr in, float z) {
  BOOST_FOREACH(PointT& p, in->points) p.z = z;
}


VectorXf getCircle(ColorCloudPtr cloud) {
  ColorCloudPtr cloud_hull (new ColorCloud());
  pcl::ConvexHull<pcl::PointXYZRGBA> chull;
  chull.setInputCloud (cloud);
  chull.setDimension(2);
  chull.reconstruct (*cloud_hull);

  boost::shared_ptr<pcl::SampleConsensusModelCircle2D<pcl::PointXYZRGBA> > model(new pcl::SampleConsensusModelCircle2D<pcl::PointXYZRGBA>(cloud_hull));
  pcl::RandomSampleConsensus<pcl::PointXYZRGBA> sac(model, .02);
  bool result = sac.computeModel(2);
  VectorXf zs = toEigenMatrix(cloud_hull).col(2);
  Eigen::VectorXf xyr;
  sac.getModelCoefficients (xyr);

  VectorXf coeffs(7);
  coeffs(0) = xyr(0);
  coeffs(1) = xyr(1);
  coeffs(2) = zs.minCoeff();
  coeffs(3) = 0;
  coeffs(4) = 0;
  coeffs(5) = zs.maxCoeff() - zs.minCoeff();
  coeffs(6) = xyr(2);

  return coeffs;

}

VectorXf getEnclosingCircle(ColorCloudPtr cloud) {

  MatrixXf xyz = toEigenMatrix(cloud);
  VectorXf zs = xyz.col(2);

  vector<cv::Point2f> points;
  for (int i=0; i < xyz.rows(); i++) {
    points.push_back(cv::Point2f(1000*xyz(i,0), 1000*xyz(i,1)));
  }
  cv::Mat mat(points);

  cv::Point2f center;
  float radius;
  cv::minEnclosingCircle(mat, center, radius);
  printf("x: %.2f, y: %.2f, r: %.2f\n", center.x/1000, center.y/1000, radius/1000);

  VectorXf coeffs(7);
  coeffs(0) = center.x/1000;
  coeffs(1) = center.y/1000;
  coeffs(2) = zs.minCoeff();
  coeffs(3) = 0;
  coeffs(4) = 0;
  coeffs(5) = zs.maxCoeff() - zs.minCoeff();
  coeffs(6) = radius/1000;

  return coeffs;

}


ColorCloudPtr getBiggestCluster(ColorCloudPtr in, float tol) {

  vector< vector<int> > cluster_inds = findClusters(in, tol, 10);
  if (cluster_inds.size() == 0) throw runtime_error("didn't find any clusters");
  VectorXi sizes(cluster_inds.size());
  for (int i=0; i < cluster_inds.size(); i++) sizes(i) = cluster_inds[i].size();
  int iBest;
  sizes.maxCoeff(&iBest);
  cout << "sizes: " << sizes.transpose() << endl;
  cout << iBest << endl;
  return extractInds(in, cluster_inds[iBest]);

}
