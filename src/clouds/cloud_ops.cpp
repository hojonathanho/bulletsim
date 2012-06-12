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
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

typedef ColorPoint PointT;

using namespace std;
using namespace Eigen;
using namespace pcl;

vector< vector<int> > findClusters(ColorCloudPtr cloud, float tol, float minSize) {
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<ColorPoint> ec;
  pcl::search::KdTree<ColorPoint>::Ptr tree (new pcl::search::KdTree<ColorPoint>);
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
  pcl::PointCloud<ColorPoint>::Ptr out(new pcl::PointCloud<ColorPoint>());
  pcl::VoxelGrid<ColorPoint> vg;
  vg.setInputCloud(in);
  vg.setLeafSize(sz,sz,sz);
  vg.filter(*out);
  return out;
}

ColorCloudPtr removeOutliers(const ColorCloudPtr in, float thresh, int k) {
  ColorCloudPtr out(new ColorCloud());
  pcl::StatisticalOutlierRemoval<ColorPoint> sor;
  sor.setInputCloud (in);
  sor.setMeanK (k);
  sor.setStddevMulThresh (thresh);
  sor.filter (*out);
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
  pcl::ProjectInliers<ColorPoint> proj;
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
//  chull.setDimension(2);
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

ColorCloudPtr filterX(ColorCloudPtr in, float low, float high) {
  pcl::PassThrough<ColorPoint> pass;
  pass.setInputCloud (in);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (low, high);

  ColorCloudPtr out(new ColorCloud());
  pass.filter (*out);
  return out;
}
ColorCloudPtr filterY(ColorCloudPtr in, float low, float high) {
  pcl::PassThrough<ColorPoint> pass;
  pass.setInputCloud (in);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (low, high);

  ColorCloudPtr out(new ColorCloud());
  pass.filter (*out);
  return out;
}
ColorCloudPtr filterZ(ColorCloudPtr in, float low, float high) {
  pcl::PassThrough<ColorPoint> pass;
  pass.setInputCloud (in);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (low, high);

  ColorCloudPtr out(new ColorCloud());
  pass.filter (*out);
  return out;
}




VectorXf getCircle(ColorCloudPtr cloud) {
  ColorCloudPtr cloud_hull (new ColorCloud());
  pcl::ConvexHull<ColorPoint> chull;
  chull.setInputCloud (cloud);
//  chull.setDimension(2);
  chull.reconstruct (*cloud_hull);

  boost::shared_ptr<pcl::SampleConsensusModelCircle2D<ColorPoint> > model(new pcl::SampleConsensusModelCircle2D<ColorPoint>(cloud_hull));
  pcl::RandomSampleConsensus<ColorPoint> sac(model, .02);
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

ColorCloudPtr clusterFilter(ColorCloudPtr in, float tol, float minSize) {
	vector< vector<int> > cluster_inds = findClusters(in, tol, minSize);
	vector<int> filtered_cluster_inds;
	for (int i=0; i < cluster_inds.size(); i++)
		if (cluster_inds[i].size() > 5)
			for (int j=0; j < cluster_inds[i].size(); j++) filtered_cluster_inds.push_back(cluster_inds[i][j]);
	return extractInds(in, filtered_cluster_inds);
}

ColorCloudPtr maskCloud(const ColorCloudPtr in, const cv::Mat& mask) {
  assert(mask.elemSize() == 1);
  assert(mask.rows == in->height);
  assert(mask.cols == in->width);

  boost::shared_ptr< vector<int> > indicesPtr(new vector<int>());

  cv::MatConstIterator_<bool> it = mask.begin<bool>(), it_end = mask.end<bool>();
  for (int i=0; it != it_end; ++it, ++i) {
    if (*it > 0) indicesPtr->push_back(i);
  }

  ColorCloudPtr out(new ColorCloud());
  pcl::ExtractIndices<ColorPoint> ei;
  ei.setNegative(false);
  ei.setInputCloud(in);
  ei.setIndices(indicesPtr);
  ei.filter(*out);
  return out;
}


ColorCloudPtr maskCloud(const ColorCloudPtr in, const VectorXb& mask) {

  ColorCloudPtr out(new ColorCloud());
  int nOut = mask.sum();
  out->reserve(nOut);
  out->header=in->header;
  out->width = nOut;
  out->height = 1;
  out->is_dense = false;

  int i = 0;
  BOOST_FOREACH(const ColorPoint& pt, in->points) {
    if (mask(i)) out->push_back(pt);
    ++i;
  }

  return out;
}

void labelCloud(ColorCloudPtr in, const cv::Mat& labels) {
  ColorCloudPtr out(new ColorCloud());
  MatrixXi uv = xyz2uv(toEigenMatrix(in));
  for (int i=0; i < in->size(); i++)
    in->points[i]._unused = labels.at<uint8_t>(uv(i,0), uv(i,1));
}

ColorCloudPtr hueFilter(const ColorCloudPtr in, uint8_t minHue, uint8_t maxHue, uint8_t minSat, uint8_t maxSat, uint8_t minVal, uint8_t maxVal) {
  MatrixXu bgr = toBGR(in);
  int nPts = in->size();
  cv::Mat cvmat(in->height,in->width, CV_8UC3, bgr.data());
  cv::cvtColor(cvmat, cvmat, CV_BGR2HSV);
  vector<cv::Mat> hsvChannels;
  cv::split(cvmat, hsvChannels);

  cv::Mat& h = hsvChannels[0];
  cv::Mat& s = hsvChannels[1];
  cv::Mat& v = hsvChannels[2];


  cv::Mat mask = (minHue < maxHue) ?
    (h > minHue) & (h < maxHue) :
    (h > minHue) | (h < maxHue);
  if (minSat > 0) mask &= (s >= minSat);
  if (maxSat < 255) mask &= (s <= maxSat);
  if (minVal > 0) mask &= (v >= minVal);
  if (maxVal < 255) mask &= (v <= maxVal);

  return maskCloud(in, mask);
}

ColorCloudPtr orientedBoxFilter(ColorCloudPtr cloud_in, const Matrix3f& ori, const Vector3f& mins, const Vector3f& maxes) {
	MatrixXf xyz = toEigenMatrix(cloud_in) * ori;

	VectorXb mask(xyz.rows());
	for (int i=0; i < xyz.rows(); i++) {
		mask(i) = (xyz(i,0) >= mins(0)) &&
				  (xyz(i,1) >= mins(1)) &&
				  (xyz(i,2) >= mins(2)) &&
				  (xyz(i,0) <= maxes(0)) &&
				  (xyz(i,1) <= maxes(1)) &&
				  (xyz(i,2) <= maxes(2));
	}
	ColorCloudPtr cloud_out = maskCloud(cloud_in, mask);
	return cloud_out;
}

cv::Mat cloud2Image(const ColorCloudPtr cloud) {
	MatrixXu bgr = toBGR(cloud);
	cv::Mat image(cloud->height, cloud->width, CV_8UC3, bgr.data());
	return image;
}

bool isBad(MatrixXf mat, int i) {
	return (isnan(mat(i,0)) ||
			isnan(mat(i,1)) ||
			isnan(mat(i,2)));
}

ColorCloudPtr checkerBoardCorners(const ColorCloudPtr in, int width_cb, int height_cb) {
	MatrixXu bgr = toBGR(in);
	int nPts = in->size();
	cv::Mat image(in->height,in->width, CV_8UC3, bgr.data());
	cv::Mat gray_image;
	cv::cvtColor(image, gray_image, CV_BGR2GRAY);

	vector<cv::Point2i> corners;
	vector<cv::Point3f> obj(width_cb*height_cb);
	for (int i=0; i<height_cb; i++)
		for (int j=0; j<width_cb; j++)
			obj[i*width_cb+j] = cv::Point3f(j, i, 0.0);
	bool found = findChessboardCorners(gray_image, cv::Size(width_cb,height_cb), corners, cv::CALIB_CB_FAST_CHECK);
	//printf("corners size %d\n", (int) corners.size());

	MatrixXf in_mat = toEigenMatrix(in);
	boost::shared_ptr< vector<int> > indicesPtr(new vector<int>());
	for (int k=0; k<corners.size(); k++) {
		int index = corners[k].y*in->width+corners[k].x;
		if (isBad(in_mat, index)) {
			for (int i=-1; i<=1; i++)
				for (int j=-1; j<=1; j++)
					if (!isBad(in_mat, (corners[k].y+j)*in->width+(corners[k].x+i))) {
						index = (corners[k].y+j)*in->width+(corners[k].x+i);
						break;
					}
		}
		indicesPtr->push_back(index);
	}
	//for (int j=0; j<in->height; j++)
	//	indicesPtr->push_back(j*in->width + in->width/2);
	//for (int i=0; i<in->width; i++)
	//	indicesPtr->push_back((in->height/2)*in->width + i);

	ColorCloudPtr out(new ColorCloud());
	pcl::ExtractIndices<ColorPoint> ei;
	ei.setNegative(false);
	ei.setInputCloud(in);
	ei.setIndices(indicesPtr);
	ei.filter(*out);
	return out;
}
