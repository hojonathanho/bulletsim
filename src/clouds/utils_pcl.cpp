#include "utils_pcl.h"
#include <pcl/io/pcd_io.h>
#include "utils/my_exceptions.h"

using namespace Eigen;
using namespace pcl;
using namespace std;

pcl::PointCloud<ColorPoint>::Ptr readPCD(const std::string& pcdfile) {
  pcl::PointCloud<ColorPoint>::Ptr cloud(new pcl::PointCloud<ColorPoint>);
  if (pcl::io::loadPCDFile<ColorPoint> (pcdfile, *cloud) == -1) {
    throw FileOpenError(pcdfile);
    }
  return cloud;
}


static const float cx = 320-.5;
static const float cy = 240-.5;
static const float f = 525;

Eigen::MatrixXi xyz2uv(const Eigen::MatrixXf& xyz) {
 // http://www.pcl-users.org/Using-Kinect-with-PCL-How-to-project-a-3D-point-x-y-z-to-the-depth-rgb-image-and-how-to-unproject-a--td3164499.html
  VectorXf x = xyz.col(0);
  VectorXf y = xyz.col(1);
  VectorXf z = xyz.col(2);

  VectorXf v = f*(x.array() / z.array()) + cx;
  VectorXf u = f*(y.array() / z.array()) + cy;
  MatrixXi uv(u.rows(),2);
  uv.col(0) = u.cast<int>();
  uv.col(1) = v.cast<int>();
  return uv;
}

Eigen::MatrixXf toEigenMatrix(ColorCloudPtr cloud) {
  return cloud->getMatrixXfMap(3,8,0);
}

MatrixXu toBGR(ColorCloudPtr cloud) {
  MatrixXf bgrFloats = cloud->getMatrixXfMap(1,8,4);
  MatrixXu bgrBytes4 = Map<MatrixXu>(reinterpret_cast<uint8_t*>(bgrFloats.data()), bgrFloats.rows(),4);
  MatrixXu bgrBytes3 = bgrBytes4.block(0,0,bgrBytes4.rows(),3);
  return bgrBytes3;
}

MatrixXf getDepthImage(ColorCloudPtr cloud) {
  MatrixXf xyz = toEigenMatrix(cloud);
  MatrixXf norms = xyz.rowwise().norm();
  norms.resize(cloud->height, cloud->width);
  return norms;
}

cv::Mat toCVMat(Eigen::MatrixXf in) {
  return cv::Mat(in.rows(), in.cols(), CV_32FC1, in.data());
}

MatrixXf toEigenMatrix(const cv::Mat& in) {
	if (in.type() != CV_32FC1) throw runtime_error("input matrix has the wrong type");
	return Map<MatrixXf>((float*)in.data, in.rows, in.cols);
}

cv::Mat toCVMatImage(const ColorCloudPtr cloud) {
	MatrixXu bgr = toBGR(cloud);
	cv::Mat image(cloud->height, cloud->width, CV_8UC3, bgr.data());
	return image.clone();
}

cv::Mat toCVMatDepthImage(const ColorCloudPtr cloud) {
	MatrixXf depth = getDepthImage(cloud);
	cv::Mat image(cloud->height, cloud->width, CV_32FC1, depth.data());
	return image.clone();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr toPointCloud(const std::vector< std::vector<float> >& in) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>());
  BOOST_FOREACH(vector<float> v, in) out->push_back(PointXYZ(v[0],v[1],v[2]));
  out->width=in.size();
  out->height=1;
  return out;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr toPointCloud(const Eigen::MatrixXf& in) {
  assert(in.cols() == 3);
  pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>());
  out->points.reserve(in.rows());
  out->width = in.rows();
  out->height = 1;
  for (int i=0; i < in.rows(); ++i) out->points.push_back(pcl::PointXYZ(in(i,0), in(i,1), in(i,2)));
  return out;
}

bool pointIsFinite(const ColorPoint& pt) {
  return std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z);
}

ColorCloudPtr transformPointCloud1(ColorCloudPtr in, Eigen::Affine3f transform) {
  ColorCloudPtr out(new ColorCloud(*in));
  BOOST_FOREACH(ColorPoint& p, out->points) {
    p.getVector3fMap() = transform * p.getVector3fMap();
  }
  return out;
}

ColorCloudPtr extractInds(ColorCloudPtr in, const std::vector<int>& inds) {
  ColorCloudPtr out(new ColorCloud());
  out->reserve(inds.size());
  out->width = inds.size();
  out->height = 1;
  out->is_dense = false;

  BOOST_FOREACH(int i, inds) {
    out->push_back(in->points[i]);
  }
  return out;
}

bool saveTransform(const string& filename, const Matrix4f& t) {
	ofstream file;
	file.open(filename.c_str());
	if (file.fail()) {
		cout << "Transform couldn't be saved to " << filename << endl;
		return false;
	}
	file.precision(20);

	for (int r=0; r<4; r++)
		for (int c=0; c<4; c++)
			file << t(r,c) << " ";
	file << "\n";

	file.close();
	cout << "Transform saved to " << filename << endl;
	return true;
}

bool loadTransform(const string& filename, Matrix4f& t) {
  ifstream file;
  file.open(filename.c_str());
  if (file.fail()) {
		cout << "Transform couldn't be loaded from " << filename << endl;
  	return false;
  }

  while (!file.eof()) {
		for (int r=0; r<4; r++)
			for (int c=0; c<4; c++)
				file >> t(r,c);
  }

  file.close();
	cout << "Transform loaded from " << filename << endl;
  return true;
}

bool saveTransform(const std::string& filename, const Affine3f& t) {
	return saveTransform(filename, t.matrix());
}

bool loadTransform(const std::string& filename, Affine3f& t) {
	Matrix4f mat = t.matrix();
	bool ret = loadTransform(filename, mat);
	t = (Affine3f) mat;
	return ret;
}

ColorCloudPtr addColor(CloudPtr in, uint8_t r, uint8_t g, uint8_t b) {
  ColorCloudPtr out(new ColorCloud());
  out->points.reserve(in->size());
  BOOST_FOREACH(Point& p, in->points) {
    ColorPoint cpt;
    cpt.x = p.x;
    cpt.y = p.y;
    cpt.z = p.z;
    cpt.r = r;
    cpt.g = g;
    cpt.b = b;
    out->points.push_back(cpt);
  }
  out->width = in->width;
  out->height = in->height;
  out->is_dense = in->is_dense;
  out->header = in->header;
  return out;
}




ColorCloudPtr fromROSMsg1(const sensor_msgs::PointCloud2& msg) {
  bool colorFound = false;
  BOOST_FOREACH(const sensor_msgs::PointField& field, msg.fields) {
    if (field.name == "r") colorFound = true;
  }
  if (colorFound) {
    ColorCloudPtr cloud(new ColorCloud());
    pcl::fromROSMsg(msg,*cloud);
    return cloud;
  }
  else {
    CloudPtr cloud(new Cloud());
    pcl::fromROSMsg(msg, *cloud);
    return addColor(cloud, 255, 255, 255);

  }
}
