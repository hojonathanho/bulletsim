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
	vector< vector<int> > cluster_inds = findClusters(in, tol, minSize);
	vector<int> filtered_cluster_inds;
	for (int i=0; i < cluster_inds.size(); i++)
		if (cluster_inds[i].size() > minSize)
			for (int j=0; j < cluster_inds[i].size(); j++) filtered_cluster_inds.push_back(cluster_inds[i][j]);
	return extractInds(in, filtered_cluster_inds);
}

ColorCloudPtr maskCloud(const ColorCloudPtr in, const cv::Mat& mask, bool negative) {
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
  ei.setNegative(negative);
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

ColorCloudPtr hueFilter(const ColorCloudPtr in, uint8_t minHue, uint8_t maxHue, uint8_t minSat, uint8_t maxSat, uint8_t minVal, uint8_t maxVal, bool negative) {
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

  return maskCloud(in, mask, negative);
}

// As of now, mins should be smaller than maxs
ColorCloudPtr colorSpaceFilter(const ColorCloudPtr in, uint8_t minx, uint8_t maxx, uint8_t miny, uint8_t maxy, uint8_t minz, uint8_t maxz, int dstCn, bool negative) {
  MatrixXu bgr = toBGR(in);
  int nPts = in->size();
  cv::Mat cvmat(in->height,in->width, CV_8UC3, bgr.data());
  cv::cvtColor(cvmat, cvmat, dstCn);
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

  return maskCloud(in, mask, negative);
}

ColorCloudPtr orientedBoxFilter(ColorCloudPtr cloud_in, const Matrix3f& ori, const Vector3f& mins, const Vector3f& maxes) {
	MatrixXf xyz = toEigenMatrix(cloud_in) * ori;
//	Vector3f min(1000,1000,1000);
//	Vector3f max(-1000,-1000,-1000);
//	for (int i=0; i < xyz.rows(); i++) {
//		for( int j=0; j<3; j++) {
//			if (xyz(i,j) < min(j))
//				min(j) = xyz(i,j);
//			if (xyz(i,j) > max(j))
//				max(j) = xyz(i,j);
//		}
//	}
//	cout << "min" << endl << min << endl;
//	cout << "max" << endl << max << endl;
//	cout << "xyz" << endl << xyz << endl;
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

void pointToImageInd(ColorPoint point, ColorCloudPtr cloud, cv::Mat image) {
	pcl::KdTreeFLANN<ColorPoint> kdtree;
	kdtree.setInputCloud (cloud);
	// K nearest neighbor search
	int K = 1;
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);
	if ( kdtree.nearestKSearch (point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
	{
		ColorPoint found_point = cloud->points[ pointIdxNKNSearch[0] ];
	}
}

//returns point cloud that is likely to be from skin
//YCrCb thresholds from http://waset.org/journals/waset/v43/v43-91.pdf except for Ymin = 40
//merging from here http://www.csee.wvu.edu/~richas/papers/tkjse.pdf
ColorCloudPtr skinFilter(ColorCloudPtr cloud_dense) {
	MatrixXu bgr = toBGR(cloud_dense);
  cv::Mat image(cloud_dense->height,cloud_dense->width, CV_8UC3, bgr.data());
  cv::Mat imageYCrCb(image.rows, image.cols, CV_8UC3);
	cv::cvtColor(image, imageYCrCb, CV_BGR2YCrCb);
	cv::Mat imageLab(image.rows, image.cols, CV_8UC3);
	cv::cvtColor(image, imageLab, CV_BGR2Lab);

  for (int i=0; i<image.rows; i++) {
		for (int j=0; j<image.cols; j++) {
			cv::Vec3b YCrCb = imageYCrCb.at<cv::Vec3b>(i,j);
			cv::Vec3b Lab = imageLab.at<cv::Vec3b>(i,j);
				if (!	((YCrCb[0] > 80 &&
						YCrCb[1] > 135 && YCrCb[1] < 180 &&
						YCrCb[2] > 85 && YCrCb[2] < 135) ||
						(Lab[1] > 130 && Lab[1] < 150 &&
								Lab[2] > 130)) ) {
				image.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
			}
		}
	}
	cv::erode(image, image, cv::Mat(), cv::Point(-1, -1), 2);
	cv::dilate(image, image, cv::Mat(), cv::Point(-1, -1), 2);

	ColorCloudPtr cloud_skin(new ColorCloud());
	for (int i=0; i<image.rows; i++) {
		for (int j=0; j<image.cols; j++) {
			if (image.at<cv::Vec3b>(i,j) != cv::Vec3b(0,0,0)) {
				cloud_skin->push_back(cloud_dense->at(j,i));
			}
		}
	}

	return cloud_skin;
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
