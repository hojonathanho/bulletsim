#include "cloud_ops.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/range_image/range_image.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <limits>
#include "utils_cv.h"
#include "utils/utils_vector.h"

typedef ColorPoint PointT;

using namespace std;
using namespace Eigen;
using namespace pcl;

ColorPoint makeNanPoint() {
	ColorPoint nan_point(numeric_limits<float>::quiet_NaN(), numeric_limits<float>::quiet_NaN(), numeric_limits<float>::quiet_NaN());
	nan_point.x = nan_point.y = nan_point.z = numeric_limits<float>::quiet_NaN();
	return nan_point;
}

vector< vector<int> > findClusters(ColorCloudPtr cloud, float tol, int minSize) {
	int cloud_size = cloud->size();
	//HACK: a bug in pcl::EuclideanClusterExtraction causes a segfault when no clusters are returned.
	//Hence add a cluster to the point cloud and then remove these indices from the result.
	ColorPoint pt (255,255,255);
	for (int i=0; i<minSize; i++)
		cloud->push_back(pt);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<ColorPoint> ec;
  pcl::search::KdTree<ColorPoint>::Ptr tree (new pcl::search::KdTree<ColorPoint>);
  tree->setInputCloud(cloud);
  ec.setClusterTolerance (tol);
  ec.setMinClusterSize (minSize);
  ec.setMaxClusterSize (2500000);
  ec.setSearchMethod (tree);
  ec.setInputCloud(cloud);
  ec.extract (cluster_indices);

  vector< vector<int> > out;

//  //HACK
//  for (int i=0; i < cluster_indices.size(); i++) {
//		vector<int> outi;
//		for (int j=0; j<cluster_indices[i].indices.size(); j++) {
//			if (cluster_indices[i].indices[j] < cloud_size)
//				outi.push_back(cluster_indices[i].indices[j]);
//		}
//		out.push_back(outi);
//  }

  //ORIGINAL
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
  if (in->size() <= k) return out;
  pcl::StatisticalOutlierRemoval<ColorPoint> sor;
  sor.setInputCloud (in);
  sor.setMeanK (k);
  sor.setStddevMulThresh (thresh);
  sor.filter (*out);
  return out;
}

ColorCloudPtr removeRadiusOutliers(const ColorCloudPtr in, float radius, int minK) {
	ColorCloudPtr out(new ColorCloud());
	pcl::RadiusOutlierRemoval<ColorPoint> outrem;
	outrem.setInputCloud(in);
	outrem.setRadiusSearch(radius);
	outrem.setMinNeighborsInRadius(minK);
	outrem.filter (*out);
	return out;
}

ColorCloudPtr smoothSurface(const ColorCloudPtr in) {
	ColorCloudPtr out(new ColorCloud());

	// Create a KD-Tree
	pcl::search::KdTree<ColorPoint>::Ptr tree (new pcl::search::KdTree<ColorPoint>);

	// Output has the PointNormal type in order to store the normals calculated by MLS
	pcl::PointCloud<ColorPoint> mls_points;

	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<ColorPoint, pcl::PointNormal> mls;

	// Set parameters
	mls.setInputCloud (in);
	mls.setPolynomialFit (true);
	mls.setSearchMethod (tree);
	mls.setSearchRadius (0.03);

	// Reconstruct
	mls.reconstruct (*out);

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

ColorCloudPtr filterPlane(ColorCloudPtr in, float dist_thresh) {
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<ColorPoint> seg;
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (dist_thresh);

	seg.setInputCloud (in->makeShared ());
	seg.segment (*inliers, *coefficients);

	pcl::ExtractIndices<ColorPoint> extract;
	extract.setInputCloud (in);
	extract.setIndices (inliers);

//	extract.setNegative (true);
//	ColorCloudPtr outliers_cloud(new ColorCloud());
//	extract.filter (*outliers_cloud);
//	return outliers_cloud;

	extract.setNegative (false);
	ColorCloudPtr inliers_cloud(new ColorCloud());
	extract.filter (*inliers_cloud);
	return inliers_cloud;
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

ColorCloudPtr clusterFilter(ColorCloudPtr in, float tol, int minSize) {
	if (in->size() < minSize) return ColorCloudPtr(new ColorCloud());
	vector< vector<int> > cluster_inds = findClusters(in, tol, minSize);
	vector<int> filtered_cluster_inds;
	for (int i=0; i < cluster_inds.size(); i++)
		if (cluster_inds[i].size() > minSize)
			for (int j=0; j < cluster_inds[i].size(); j++) filtered_cluster_inds.push_back(cluster_inds[i][j]);
	return extractInds(in, filtered_cluster_inds);
}

ColorCloudPtr maskCloud(const ColorCloudPtr in, cv::Mat mask, bool organized, bool negative) {
	if (mask.type() == CV_8UC3)
		cv::cvtColor(mask, mask, CV_BGR2GRAY);

  assert(mask.elemSize() == 1);
  assert(mask.rows == in->height);
  assert(mask.cols == in->width);
  ColorCloudPtr out;

  if (organized) {
		out = ColorCloudPtr(new ColorCloud(*in));

  	if (negative) {
			for (int i=0; i<in->height; i++) {
				for (int j=0; j<in->width; j++) {
					if (mask.at<bool>(i,j)) {
						out->at(in->width*i+j).x =  numeric_limits<float>::quiet_NaN();
						out->at(in->width*i+j).y =  numeric_limits<float>::quiet_NaN();
						out->at(in->width*i+j).z =  numeric_limits<float>::quiet_NaN();
						out->at(in->width*i+j).r = 0;
						out->at(in->width*i+j).g = 0;
						out->at(in->width*i+j).b = 0;
					}
				}
			}
		} else {
			for (int i=0; i<in->height; i++) {
				for (int j=0; j<in->width; j++) {
					if (!mask.at<bool>(i,j)) {
						out->at(in->width*i+j).x =  numeric_limits<float>::quiet_NaN();
						out->at(in->width*i+j).y =  numeric_limits<float>::quiet_NaN();
						out->at(in->width*i+j).z =  numeric_limits<float>::quiet_NaN();
						out->at(in->width*i+j).r = 0;
						out->at(in->width*i+j).g = 0;
						out->at(in->width*i+j).b = 0;
					}
				}
			}
		}

  } else {

		boost::shared_ptr< vector<int> > indicesPtr(new vector<int>());
		cv::MatConstIterator_<bool> it = mask.begin<bool>(), it_end = mask.end<bool>();
		for (int i=0; it != it_end; ++it, ++i) {
			if (*it > 0) indicesPtr->push_back(i);
		}

		out = ColorCloudPtr(new ColorCloud());
		pcl::ExtractIndices<ColorPoint> ei;
		ei.setNegative(negative);
		ei.setInputCloud(in);
		ei.setIndices(indicesPtr);
		ei.filter(*out);
  }

  return out;
}

ColorCloudPtr maskCloud(const ColorCloudPtr in, const VectorXb& mask, bool organized, bool negative) {
	assert(mask.size() == (in->height*in->width));
  ColorCloudPtr out;

  if (organized) {
		out = ColorCloudPtr(new ColorCloud(*in));

  	if (negative) {
  		for (int i=0; i<in->height; i++) {
				for (int j=0; j<in->width; j++) {
					if (mask(in->width*i+j)) {
						out->at(in->width*i+j).x = numeric_limits<float>::quiet_NaN();
						out->at(in->width*i+j).y = numeric_limits<float>::quiet_NaN();
						out->at(in->width*i+j).z = numeric_limits<float>::quiet_NaN();
						out->at(in->width*i+j).r = 0;
						out->at(in->width*i+j).g = 0;
						out->at(in->width*i+j).b = 0;
					}
				}
			}
		} else {
			for (int i=0; i<in->height; i++) {
				for (int j=0; j<in->width; j++) {
					if (!mask(in->width*i+j)) {
						out->at(in->width*i+j).x = numeric_limits<float>::quiet_NaN();
						out->at(in->width*i+j).y = numeric_limits<float>::quiet_NaN();
						out->at(in->width*i+j).z = numeric_limits<float>::quiet_NaN();
						out->at(in->width*i+j).r = 0;
						out->at(in->width*i+j).g = 0;
						out->at(in->width*i+j).b = 0;
					}
				}
			}
		}

  } else {

  	out = ColorCloudPtr(new ColorCloud());
		int nOut = mask.sum();
		out->reserve(nOut);
		out->header=in->header;
		out->width = nOut;
		out->height = 1;
		out->is_dense = false;

		if (negative) {
			int i = 0;
			BOOST_FOREACH(const ColorPoint& pt, in->points) {
				if (!mask(i)) out->push_back(pt);
				++i;
			}
		} else {
			int i = 0;
			BOOST_FOREACH(const ColorPoint& pt, in->points) {
				if (mask(i)) out->push_back(pt);
				++i;
			}
		}
  }

  return out;
}

void labelCloud(ColorCloudPtr in, const cv::Mat& labels) {
  ColorCloudPtr out(new ColorCloud());
  MatrixXi uv = xyz2uv(toEigenMatrix(in));
  for (int i=0; i < in->size(); i++)
    in->points[i]._unused = labels.at<uint8_t>(uv(i,0), uv(i,1));
}

ColorCloudPtr hueFilter(const ColorCloudPtr in, uint8_t minHue, uint8_t maxHue, uint8_t minSat, uint8_t maxSat, uint8_t minVal, uint8_t maxVal, bool organized, bool negative) {
  if (in->size() == 0) return ColorCloudPtr(new ColorCloud());
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

  return maskCloud(in, mask, organized, negative);
}

// As of now, mins should be smaller than maxs
// If negative==false, the filtered cloud have points with colors within the given channel ranges
// If negative==true, the filtered cloud is the complement of the above case
ColorCloudPtr colorSpaceFilter(const ColorCloudPtr in, uint8_t minx, uint8_t maxx, uint8_t miny, uint8_t maxy, uint8_t minz, uint8_t maxz, int code, bool organized, bool negative) {
	if (in->size() == 0) return ColorCloudPtr(new ColorCloud(*in));
	MatrixXu bgr = toBGR(in);
  cv::Mat cvmat(in->height,in->width, CV_8UC3, bgr.data());
  cv::cvtColor(cvmat, cvmat, code);
  vector<cv::Mat> channels;
  cv::split(cvmat, channels);

  cv::Mat& x = channels[0];
  cv::Mat& y = channels[1];
  cv::Mat& z = channels[2];

//  cv::Mat maskx = (minx < maxx) ?
//      (x > minx) & (x < maxx) :
//      (x > minx) | (x < maxx);
//  cv::Mat masky = (miny < maxy) ?
//        (y > miny) & (y < maxy) :
//        (y > miny) | (y < maxy);
//  cv::Mat maskz = (minz < maxz) ?
//        (z > minz) & (z < maxz) :
//        (z > minz) | (z < maxz);
//  cv::Mat mask = maskx & masky & maskz;

  cv::Mat mask = (x > minx) & (x < maxx);
	if (miny > 0) mask &= (y >= miny);
	if (maxy < 255) mask &= (y <= maxy);
	if (minz > 0) mask &= (z >= minz);
	if (maxz < 255) mask &= (z <= maxz);

  return maskCloud(in, mask, organized, negative);
}

ColorCloudPtr orientedBoxFilter(ColorCloudPtr cloud_in, const Matrix3f& ori, const Vector3f& mins, const Vector3f& maxes, bool organized, bool negative) {
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
	ColorCloudPtr cloud_out = maskCloud(cloud_in, mask, organized, negative);
	return cloud_out;
}

ColorCloudPtr boxFilter(ColorCloudPtr cloud_in, const Vector3f& mins, const Vector3f& maxes) {
	MatrixXf xyz = toEigenMatrix(cloud_in);

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

ColorCloudPtr chessBoardCorners(const ColorCloudPtr in, int width_cb, int height_cb) {
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
	bool found = findChessboardCorners(gray_image, cv::Size(width_cb,height_cb), corners); //, cv::CALIB_CB_FAST_CHECK);
	//printf("corners size %d\n", (int) corners.size());
//	if (found) {
//		drawChessboardCorners(image, cv::Size(width_cb,height_cb), corners, found);
//		imshow("win", image);
//	}

	boost::shared_ptr< vector<int> > indicesPtr(new vector<int>());
	for (int k=0; k<corners.size(); k++) {
		int index = corners[k].y*in->width+corners[k].x;
		if (!pointIsFinite(in->at(corners[k].x, corners[k].y))) {
			vector<float> ranges;
			vector<int> indexes;
			for (int i=-1; i<=1; i++) {
				if (((corners[k].x+i) < 0) || ((corners[k].x+i) > in->width)) continue;
				for (int j=-1; j<=1; j++) {
					if (((corners[k].y+i) < 0) || ((corners[k].y+i) > in->height)) continue;
					if (pointIsFinite(in->at(corners[k].x+i, corners[k].y+j))) {
						ColorPoint pt = in->at(corners[k].x+i, corners[k].y+j);
						ranges.push_back(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
						indexes.push_back((corners[k].y+j)*in->width+(corners[k].x+i));
					}
				}
			}
			int medianInd = argMedian(ranges);
			if (medianInd > 0 && medianInd < indexes.size()) index = indexes[medianInd];
		}
		indicesPtr->push_back(index);
	}
	//Debug: draw a vertical and a horizontal line in the middles of the point cloud
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

//the transform brings points from the camera coordinate system to the reference (center of chess board) coordinate system
//i.e. cloudref_corners[i] = transform * cloud_corners[i]
int getChessBoardPose(const ColorCloudPtr cloud_in, int width_cb, int height_cb, double square_size, Matrix4f& transform) {
	ColorCloudPtr cloud_corners = chessBoardCorners(cloud_in, width_cb, height_cb);
	int nAllPoints = width_cb * height_cb;
	if (cloud_corners->size() == nAllPoints) {
		ColorCloudPtr cloudref_corners(new ColorCloud());
		for (int i=0; i<height_cb; i++) {
			for (int j=(width_cb-1); j>=0; j--) {
				ColorPoint pt;
				pt.x = square_size * (j - ((float) width_cb - 1.0)/2.0);
				pt.y = square_size * (i - ((float) height_cb - 1.0)/2.0);
				pt.z = 0;
				cloudref_corners->push_back(pt);
			}
		}

		//Filter out the bad points from both point clouds (cloud_corners and cloudref_corners)
		vector<int> badPoints;
		for (int i=0; i<cloud_corners->size(); i++) {
			if (!pointIsFinite(cloud_corners->at(i)))
				badPoints.push_back(i);
		}

		for (int i=(badPoints.size()-1); i>=0; i--) {
			cloud_corners->erase(cloud_corners->begin() + badPoints[i]);
			cloudref_corners->erase(cloudref_corners->begin() + badPoints[i]);
		}

//		pcl::PassThrough<ColorPoint> ptfilter (true); // Initializing with true will allow us to extract the removed indices
//		ptfilter.setInputCloud(cloud_corners);
//		ptfilter.filter(*cloud_corners);
//		pcl::IndicesConstPtr invalid_points_indices = ptfilter.getRemovedIndices(); // The invalid_points_indices indexes all non-finite points of cloud_corners
//
//		if (invalid_points_indices->size() > 0) {
//			pcl::ExtractIndices<ColorPoint> extract_indices;
//			extract_indices.setNegative(true);
//			extract_indices.setIndices(invalid_points_indices);
//			extract_indices.setInputCloud(cloudref_corners);
//			extract_indices.filter(*cloudref_corners);
//		}

		if (cloud_corners->size()>=0.5*nAllPoints) {
			pcl::registration::TransformationEstimationSVD<ColorPoint, ColorPoint> estimation_svd;
			estimation_svd.estimateRigidTransformation(*cloud_corners, *cloudref_corners, transform);

			// There are two possible transforms. One of them represents when the camera looks
			// at the chess board from above it, and the other one when the camera is at the
			// mirror position and orientation with respect to the chess board plane.
			// If the z coordinate (transform(3,2)) of the camera is positive, then the transform
			// represents when the camera is looks from above.
			if (transform(2,3) < 0)
				transform = Vector4f(-1,1,-1,1).asDiagonal() * transform;
			return cloud_corners->size();
		}
	}
	return 0;
}

// Returns the point in cloud that corresponds to pixel. If such point is invalid, the point is the one around a 3x3 pixel window.
// If all of those points are invalid, then the point is invalid.
ColorPoint getCorrespondingPoint(ColorCloudPtr cloud, cv::Point2i pixel) {
	int index = pixel.y*cloud->width+pixel.x;
	if (!pointIsFinite(cloud->at(pixel.x, pixel.y))) {
		vector<float> ranges;
		vector<int> indexes;
		for (int i=-1; i<=1; i++) {
			if (((pixel.x+i) < 0) || ((pixel.x+i) > cloud->width)) continue;
			for (int j=-1; j<=1; j++) {
				if (((pixel.y+i) < 0) || ((pixel.y+i) > cloud->height)) continue;
				if (pointIsFinite(cloud->at(pixel.x+i, pixel.y+j))) {
					ColorPoint pt = cloud->at(pixel.x+i, pixel.y+j);
					ranges.push_back(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
					indexes.push_back((pixel.y+j)*cloud->width+(pixel.x+i));
				}
			}
		}
		int medianInd = argMedian(ranges);
		if (medianInd > 0 && medianInd < indexes.size()) index = indexes[medianInd];
	}
	return cloud->at(index);
}

//Filters out the points that are likely to be skin
ColorCloudPtr skinFilter(ColorCloudPtr cloud_dense) {
	MatrixXu bgr = toBGR(cloud_dense);
  cv::Mat image(cloud_dense->height,cloud_dense->width, CV_8UC3, bgr.data());
  cv::Mat skin_mask = skinMask(image);
  return maskCloud(cloud_dense, skin_mask, false, true);
}

// if negative false, returns points in cloud_in that are neighbors to any point in cloud_neighbor
// if negative true, returns points in cloud_in that are not neighbors to any point in cloud_neighbor
ColorCloudPtr filterNeighbors(ColorCloudPtr cloud_in, ColorCloudPtr cloud_neighbor, float radius_search, int color_squared_dist, bool negative) {
	pcl::PointIndices::Ptr indices_neighbor = neighborIndices(cloud_in, cloud_neighbor, radius_search, color_squared_dist);
	pcl::ExtractIndices<ColorPoint> extract;
	extract.setInputCloud(cloud_in);
	extract.setIndices(indices_neighbor);
	extract.setNegative(negative);
	ColorCloudPtr cloud_neighbors(new ColorCloud());
	extract.filter (*cloud_neighbors);
	return cloud_neighbors;
}

// indices of cloud_in that are within radius_search and within color_squared_dist of any point in cloud_neighbor
pcl::PointIndices::Ptr neighborIndices(ColorCloudPtr cloud_in, ColorCloudPtr cloud_neighbor, float radius_search, int color_squared_dist) {
	pcl::PointIndices::Ptr indices_neighbor (new pcl::PointIndices);
	pcl::KdTreeFLANN<ColorPoint> kdtree;
	kdtree.setInputCloud(cloud_in);
	for (size_t j = 0; j < cloud_neighbor->size(); j++) {
		ColorPoint searchPoint = cloud_neighbor->at(j);
		if (!pointIsFinite(searchPoint)) continue;
		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;
		if ( kdtree.radiusSearch (searchPoint, radius_search, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
			for (size_t i = 0; i < pointIdxRadiusSearch.size(); i++) {
				int r_diff = ((int) cloud_in->points[pointIdxRadiusSearch[i]].r) - ((int) searchPoint.r);
				int g_diff = ((int) cloud_in->points[pointIdxRadiusSearch[i]].g) - ((int) searchPoint.g);
				int b_diff = ((int) cloud_in->points[pointIdxRadiusSearch[i]].b) - ((int) searchPoint.b);
				if ((r_diff*r_diff + g_diff*g_diff + b_diff*b_diff) < color_squared_dist)
					indices_neighbor->indices.push_back(pointIdxRadiusSearch[i]);
			}
	}
	return indices_neighbor;
}

//Returns the border cloud as green points, the veil cloud as red points, and the shadow cloud as blue points.
ColorCloudPtr extractBorder(ColorCloudPtr cloud_in, ColorCloudPtr cloud_veil, ColorCloudPtr cloud_shadow) {
	Eigen::Affine3f scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (cloud_in->sensor_origin_[0], cloud_in->sensor_origin_[1], cloud_in->sensor_origin_[2])) * Eigen::Affine3f (cloud_in->sensor_orientation_);
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
	float noise_level = 0.0;
	float min_range = 0.0f;
	int border_size = 1;
	boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage);
	pcl::RangeImage& range_image = *range_image_ptr;
	range_image.createFromPointCloud (*cloud_in, pcl::deg2rad(0.5), pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
																	 scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
	pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
	range_image.integrateFarRanges (far_ranges);
	range_image.setUnseenToMaxRange ();

	pcl::PointCloud<pcl::BorderDescription> border_descriptions;
	pcl::RangeImageBorderExtractor border_extractor (&range_image);
	border_extractor.compute (border_descriptions);

	pcl::PointCloud<pcl::PointWithRange>::Ptr border_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
																						veil_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
																						shadow_points_ptr(new pcl::PointCloud<pcl::PointWithRange>);
	pcl::PointCloud<pcl::PointWithRange>& border_points = *border_points_ptr,
																			& veil_points = * veil_points_ptr,
																			& shadow_points = *shadow_points_ptr;
	for (int y=0; y< (int)range_image.height; ++y)
	{
		for (int x=0; x< (int)range_image.width; ++x)
		{
			if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER])
				border_points.points.push_back (range_image.points[y*range_image.width + x]);
			if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__VEIL_POINT])
				veil_points.points.push_back (range_image.points[y*range_image.width + x]);
			if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__SHADOW_BORDER])
				shadow_points.points.push_back (range_image.points[y*range_image.width + x]);
		}
	}

	ColorCloudPtr cloud_border(new ColorCloud());
	for (int i=0; i<border_points.size(); i++) {
		ColorPoint pt(0,255,0);
		pt.x = border_points.at(i).x;
		pt.y = border_points.at(i).y;
		pt.z = border_points.at(i).z;
		cloud_border->push_back(pt);
	}
	for (int i=0; i<veil_points.size(); i++) {
		ColorPoint pt(255,0,0);
		pt.x = veil_points.at(i).x;
		pt.y = veil_points.at(i).y;
		pt.z = veil_points.at(i).z;
		cloud_veil->push_back(pt);
	}
	for (int i=0; i<shadow_points.size(); i++) {
		ColorPoint pt(0,255,255);
		pt.x = shadow_points.at(i).x;
		pt.y = shadow_points.at(i).y;
		pt.z = shadow_points.at(i).z;
		cloud_shadow->push_back(pt);
	}

	return cloud_border;
}
