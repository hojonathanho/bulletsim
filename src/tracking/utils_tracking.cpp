#include "utils_tracking.h"
#include "simulation/config_bullet.h"
#include "utils/conversions.h"
#include <boost/foreach.hpp>
#include "simulation/recording.h"
#include <cv_bridge/cv_bridge.h>
#include "clouds/utils_cv.h"

using namespace std;
using namespace Eigen;

Affine3f Scaling3f(float s) {
  Affine3f T;
  T = s*Matrix3f::Identity();
  return T;
}

CoordinateTransformer::CoordinateTransformer() {}
CoordinateTransformer::CoordinateTransformer(const btTransform& wfc) { reset(wfc); }

void CoordinateTransformer::reset(const btTransform &wfc) {
  worldFromCamUnscaled = wfc;
  worldFromCamEigen = Scaling3f(GeneralConfig::scale)*toEigenTransform(wfc);
  camFromWorldUnscaled = wfc.inverse();
  camFromWorldEigen = worldFromCamEigen.inverse();
}

vector<btVector3> CoordinateTransformer::toWorldFromCamN(const vector<btVector3>& camVecs) {
  vector<btVector3> worldVecs(camVecs.size());
  for (int i=0; i<camVecs.size(); i++) worldVecs[i] = toWorldFromCam(camVecs[i]);
  return worldVecs;
}

vector<btVector3> CoordinateTransformer::toCamFromWorldN(const vector<btVector3>& worldVecs) {
  vector<btVector3> camVecs(worldVecs.size());
  for (int i=0; i<worldVecs.size(); i++) camVecs[i] = toCamFromWorld(worldVecs[i]);
  return camVecs;
}

Eigen::MatrixXf CoordinateTransformer::toCamFromWorldMatrixXf(const MatrixXf& in) {
	MatrixX3f out(in.rows(), in.cols());
	for (int row=0; row<in.rows(); ++row)
		out.row(row) = camFromWorldEigen * Map<const Vector3f>(in.row(row).data(), 3, 1);
	return out;
}


ImageTopicRecorder::ImageTopicRecorder(ros::NodeHandle& nh, std::string image_topic) :
		m_subscriber(nh.subscribe(image_topic, 3, &ImageTopicRecorder::callback, this)),
		m_cycle_time(boost::posix_time::microseconds(1000000.0 * RecordingConfig::speed_up/RecordingConfig::frame_rate)),
		m_full_filename(RecordingConfig::dir + "/" +  RecordingConfig::video_file + ".avi")
{}

ImageTopicRecorder::ImageTopicRecorder(ros::NodeHandle& nh, std::string image_topic, std::string full_filename) :
		m_subscriber(nh.subscribe(image_topic, 3, &ImageTopicRecorder::callback, this)),
		m_cycle_time(boost::posix_time::microseconds(1000000.0 * RecordingConfig::speed_up/RecordingConfig::frame_rate)),
		m_full_filename(full_filename)
{}

void ImageTopicRecorder::callback(sensor_msgs::ImageConstPtr image_msg) {
  cv::Mat image;
  extractImage(cv_bridge::toCvCopy(image_msg)->image, image);

  boost::posix_time::ptime current_time = boost::posix_time::microsec_clock::local_time();

	if (!m_video_writer.isOpened()) {
		m_video_writer.open(m_full_filename,  CV_FOURCC('P','I','M','1'), RecordingConfig::frame_rate, cv::Size(image.cols, image.rows));
		if (!m_video_writer.isOpened()) {
			runtime_error("Video destination file " + m_full_filename + " doesn't exits.");
		}

		m_video_writer << image;
		m_last_time = current_time + m_cycle_time;

	} else {
		while (m_last_time <= current_time) {
			assert(!m_last_image.empty());
			m_video_writer << m_last_image;
			m_last_time += m_cycle_time;
		}
	}

	m_last_image = image;
}


std::vector<btVector3> scaleVecs(const std::vector<btVector3>& in, float scale) {
	vector<btVector3> out = in;
	BOOST_FOREACH(btVector3& pt, out) pt *= scale;
	return out;
}

ColorCloudPtr scaleCloud(ColorCloudPtr in, float scale) {
	ColorCloudPtr out(new ColorCloud(*in));
	BOOST_FOREACH(ColorPoint& pt, out->points) {
		pt.x *= scale;
		pt.y *= scale;
		pt.z *= scale;
	}
	return out;
}


MatrixXf pairwiseSquareDist(const Eigen::MatrixXf& x_m3, const Eigen::MatrixXf& y_n3) {
  // vectors are rows of x and y
  MatrixXf dots_mn = x_m3 * y_n3.transpose();
  VectorXf xnorm_m = x_m3.rowwise().squaredNorm();
  VectorXf ynorm_n = y_n3.rowwise().squaredNorm();
  MatrixXf sumpart_mn = xnorm_m.replicate(1,y_n3.rows()) + ynorm_n.transpose().replicate(x_m3.rows(),1);
  MatrixXf sqdists_mn = sumpart_mn - 2*dots_mn;
  return sqdists_mn;
}

vector<int> argminAlongRows(const MatrixXf& d_mn) {
  int nRows = d_mn.rows();
  vector<int> out(nRows);
  for (int i=0; i<nRows; i++){
    int j;
    d_mn.row(i).minCoeff(&j);
    out[i]=j;
  }
  return out;
}

std::vector<btVector3> toBulletVectors(ColorCloudPtr in) {
  std::vector<btVector3> out(in->size());
  for (int i=0; i < in->size(); ++i) out[i] = btVector3(in->points[i].x,in->points[i].y,in->points[i].z);
  return out;
}
