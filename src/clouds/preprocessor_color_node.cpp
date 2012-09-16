#include <cmath>
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PolygonStamped.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "clouds/utils_pcl.h"
#include "clouds/cloud_ops.h"
#include "get_table2.h"
#include "utils_cv.h"
#include "utils/my_exceptions.h"
#include "utils/config.h"
#include "utils/conversions.h"
#include "utils_ros.h"
#include "utils/file.h"
#include "grabcut.h"
#include "utils/logging.h"
#include "utils/clock.h"

using namespace std;
using namespace Eigen;

enum boundary_t {
  NONE, LOAD_FILE, RED_MARKERS, TABLE
};

struct LocalConfig: Config {
  static std::string inputTopic;
  static string nodeNS;
  static float zClipLow;
  static float zClipHigh;
  static bool updateParams;
  static float downsample;
  static bool removeOutliers;
  static float clusterTolerance;
  static float clusterMinSize;
  static float outlierRadius;
  static int outlierMinK;
  static float offset;
  static string boundaryFile;
  static int boundaryType;
  static bool debugMask;
  static int threads;
  static string groundFrame;
  static int i0;
  static int i1;
  static int i2;
  static int i3;

  LocalConfig() :
    Config() {
    params.push_back(new Parameter<string> ("inputTopic", &inputTopic, "input topic"));
    params.push_back(new Parameter<string> ("nodeNS", &nodeNS, "node namespace"));
    params.push_back(new Parameter<float> ("zClipLow", &zClipLow, "clip points that are less than this much above table"));
    params.push_back(new Parameter<float> ("zClipHigh", &zClipHigh, "clip points that are more than this much above table"));
    params.push_back(new Parameter<bool> ("updateParams", &updateParams, "start a thread to periodically update the parameters thru the parameter server"));
    params.push_back(new Parameter<float> ("downsample", &downsample, "downsample voxel grid size. 0 means no"));
    params.push_back(new Parameter<bool> ("removeOutliers", &removeOutliers, "remove outliers"));
    params.push_back(new Parameter<float> ("clusterTolerance", &clusterTolerance, "points within this distance are in the same cluster"));
    params.push_back(new Parameter<float> ("clusterMinSize", &clusterMinSize, "the clusters found must have at least this number of points. 0 means no filtering"));
    params.push_back(new Parameter<float> ("outlierRadius", &outlierRadius, "radius search RadiusOutlierRemoval filter"));
    params.push_back(new Parameter<int> ("outlierMinK", &outlierMinK, "minimum neighbors in radius search for RadiusOutlierRemoval filter"));
    params.push_back(new Parameter<float> ("offset", &offset, "offset for the box filter (shrinks the box by offset at each side of the x and y axes)"));
    params.push_back(new Parameter<string> ("boundaryFile", &boundaryFile, "file from which the boundary gets loaded/saved"));
    params.push_back(new Parameter<int> ("boundaryType", &boundaryType, "the vertices defining the prism convex hull filter could be: NONE=0, LOAD_FILE, RED_MARKERS, TABLE"));
    params.push_back(new Parameter<bool> ("debugMask", &debugMask, "set to true if you want to debug the intermediate mask filters"));
    params.push_back(new Parameter<int> ("threads", &threads, "number of threads"));
    params.push_back(new Parameter<string> ("groundFrame", &groundFrame, "ground frame"));
    params.push_back(new Parameter<int> ("i0", &i0, "miscellaneous variable 0"));
    params.push_back(new Parameter<int> ("i1", &i1, "miscellaneous variable 1"));
    params.push_back(new Parameter<int> ("i2", &i2, "miscellaneous variable 2"));
    params.push_back(new Parameter<int> ("i3", &i3, "miscellaneous variable 3"));
  }
};

string LocalConfig::inputTopic = "/kinect1/depth_registered/points";
string LocalConfig::nodeNS = "/preprocessor/kinect1";
float LocalConfig::zClipLow = -0.02;
float LocalConfig::zClipHigh = 0.5;
bool LocalConfig::updateParams = true;
float LocalConfig::downsample = .02;
bool LocalConfig::removeOutliers = false;
float LocalConfig::clusterTolerance = 0.03;
float LocalConfig::clusterMinSize = 0;
float LocalConfig::outlierRadius = 0.02;
int LocalConfig::outlierMinK = 0;
float LocalConfig::offset = 0.02;
string LocalConfig::boundaryFile = "polygon";
int LocalConfig::boundaryType = LOAD_FILE;
bool LocalConfig::debugMask = false;
int LocalConfig::threads = 4;
string LocalConfig::groundFrame = "/ground";
int LocalConfig::i0 = 1;
int LocalConfig::i1 = 0;
int LocalConfig::i2 = 0;
int LocalConfig::i3 = 0;

static int MIN_L = 0, MAX_L = 255, MIN_A = 115, MAX_A = 255, MIN_B = 0, MAX_B = 255;

void gcPlotMask(cv::Mat_<uint8_t> mask, cv::Mat plotImg, uint8_t r, uint8_t g, uint8_t b) {
  vector<vector<cv::Point2i> > contours;
  cv::findContours(mask, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
  cv::drawContours(plotImg, contours, -1, cv::Scalar(r, g, b), 2);
}

class ThreadPool {
public:
  ThreadPool(int n_threads) : io_service(n_threads), work(io_service) {
    LOG_DEBUG("Spawning " << n_threads << " threads");
    for (int i = 0; i < n_threads; ++i) {
      threads.create_thread(boost::bind(&boost::asio::io_service::run, &io_service));
    }
  }

  ~ThreadPool() {
    io_service.stop();
    threads.join_all();
  }

  template<typename T>
  void post(T task) {
    io_service.post(task);
  }

private:
  boost::asio::io_service io_service;
  boost::asio::io_service::work work;
  boost::thread_group threads;
};

class PreprocessorNode {
public:
  ros::NodeHandle& m_nh;
  ros::Publisher m_cloudPub, m_imagePub, m_depthPub;
  ros::Publisher m_maskPub; // for debugging

  ros::Publisher m_polyPub;
  tf::TransformBroadcaster m_broadcaster;
  tf::TransformListener m_listener;
  ros::Subscriber m_sub;

  bool m_inited;
  bool m_frame_exists;

  Matrix3f m_axes;
  Vector3f m_mins, m_maxes;

  btTransform m_transform;
  geometry_msgs::Polygon m_poly;

  cv::Mat m_fgdModel, m_bgdModel;

  ColorCloudPtr cloud_hull;
  std::vector<pcl::Vertices> hull_vertices;

  boost::mutex m_mutex;

  ThreadPool threadpool;

  void callback0(const sensor_msgs::PointCloud2& msg_in) {
    if (m_inited && LocalConfig::threads > 1) {
      threadpool.post(boost::bind(&PreprocessorNode::callback, this, msg_in));
      //boost::thread cbthread(&PreprocessorNode::callback, this, msg_in);
    } else {
      callback(msg_in);
    }
  }

  void callback(const sensor_msgs::PointCloud2& msg_in) {
    double tStartCB = GetClock();
    // Needs this to update the opencv windows
    if (LocalConfig::debugMask) {
      char key = cv::waitKey(20);
      if (key == 'q') exit(0);
    }

    ColorCloudPtr cloud_in(new ColorCloud());
    pcl::fromROSMsg(msg_in, *cloud_in);

    if (!m_inited) {
      if (LocalConfig::boundaryType != NONE) initPrismBoundaries(cloud_in, msg_in);
    }

    //    boost::thread cropThread(&PreprocessorNode::cropCloud, this, cloud_in);
    // prism filter

    // image-based filters: color, morphological and connected components
    cv::Mat image = toCVMatImage(cloud_in);

    cv::Mat mask;

    if (!m_inited) {
      cv::Mat neg_green = colorSpaceMask(image, MIN_L, MAX_L, MIN_A, MAX_A, MIN_B, MAX_B, CV_BGR2Lab); // remove green
      if (LocalConfig::debugMask) cv::imshow("mask negative green", neg_green);

      mask = neg_green;
      if (LocalConfig::debugMask) cv::imshow("mask original", mask);

      m_bgdModel = cv::Mat(1, 65, CV_64FC1, (double) 0);
      m_fgdModel = cv::Mat(1, 65, CV_64FC1, (double) 0);
      mask &= 1;
      mask += 2;
      cv::Rect rect;
      LOG_INFO("doing grabcut");
      cv::resize(mask, mask, cv::Size(320, 240), CV_INTER_NN);
      cv::Mat image_small;
      cv::resize(image, image_small, cv::Size(320, 240), cv::INTER_LINEAR);
      cv::grabCut(image_small, mask, rect, m_bgdModel, m_fgdModel, 5, cv::GC_INIT_WITH_MASK);
      cv::resize(mask, mask, cv::Size(640, 480), CV_INTER_NN);
      mask -= 2;
      mask *= 255;
      cv::imshow("initmask", mask);

    } else {
      cv::Mat image_small;
      cv::resize(image, image_small, cv::Size(320, 240), cv::INTER_LINEAR);
      TIC();
      cv::Mat mask_small = gmmGraphCut(image_small, m_bgdModel, m_fgdModel);
      LOG_INFO_FMT("grabcut time: %2f", TOC());
      cv::resize(mask_small, mask, cv::Size(640, 480), CV_INTER_NN);
    }

    m_inited = true;

    //    cropThread.join();


    if (LocalConfig::debugMask) {
      gcPlotMask(mask, image, 0, 255, 0);
      cv::imshow("cloud image", image);
      //      gcPlotMask(crop_mask, image,0,0,255);
      cv::imshow("cloud image", image);
    }

    cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), LocalConfig::i0);
    ColorCloudPtr cloud_out = maskCloud(cloud_in, mask,true);

    mask = toBinaryMask(toCVMatImage(cloud_out));

    cv::Mat image_and_mask;
    cv::cvtColor(image, image_and_mask, CV_BGR2BGRA);
    vector<cv::Mat> channels;
    cv::split(image_and_mask, channels);
    channels[3] = mask;
    cv::merge(channels, image_and_mask);


    if (LocalConfig::removeOutliers) cloud_out = removeOutliers(cloud_out, 1, 15);
    if (LocalConfig::downsample > 0) cloud_out = downsampleCloud(cloud_out, LocalConfig::downsample);
    if (LocalConfig::outlierMinK > 0) cloud_out = removeRadiusOutliers(cloud_out, LocalConfig::outlierRadius, LocalConfig::outlierMinK);
    cloud_out = cropToHull(cloud_out, cloud_hull, hull_vertices, false);
    if (LocalConfig::clusterMinSize > 0) cloud_out = clusterFilter(cloud_out, LocalConfig::clusterTolerance, LocalConfig::clusterMinSize);

    //TIC();
    //LOG_INFO_FMT("crop time: %2f", TOC());


    //Publish cloud
    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(*cloud_out, msg_out);
    msg_out.header = msg_in.header;

    if (LocalConfig::debugMask) {
      cv::namedWindow("lab params");
      cv::createTrackbar("min l (lightness)     ", "lab params", &MIN_L, 255);
      cv::createTrackbar("max l (lightness)     ", "lab params", &MAX_L, 255);
      cv::createTrackbar("min a (green -> red)  ", "lab params", &MIN_A, 255);
      cv::createTrackbar("max a (green -> red)  ", "lab params", &MAX_A, 255);
      cv::createTrackbar("min b (blue -> yellow)", "lab params", &MIN_B, 255);
      cv::createTrackbar("max b (blue -> yellow)", "lab params", &MAX_B, 255);
    }

    //Publish image version of cloud
    cv_bridge::CvImage image_msg;
    image_msg.header = msg_in.header;
    image_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC4;
    image_msg.image = image_and_mask;

    cv_bridge::CvImage depth_msg;
    depth_msg.header   = msg_in.header;
    depth_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    depth_msg.image    = toCVMatDepthImage(cloud_in);

    cv_bridge::CvImage mask_msg;
    mask_msg.header   = msg_in.header;
    mask_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    mask_msg.image    = mask;

    geometry_msgs::PolygonStamped polyStamped;
    polyStamped.polygon = m_poly;
    polyStamped.header.frame_id = LocalConfig::groundFrame;
    polyStamped.header.stamp = ros::Time::now();

    m_mutex.lock();
    m_cloudPub.publish(msg_out);
    m_imagePub.publish(image_msg.toImageMsg());
    m_depthPub.publish(depth_msg.toImageMsg());
    m_maskPub.publish(mask_msg.toImageMsg());
    m_polyPub.publish(polyStamped);
    m_mutex.unlock();

    LOG_INFO_FMT("total time: %.2f",GetClock()-tStartCB);

  }

  void initPrismBoundaries(ColorCloudPtr cloud, const sensor_msgs::PointCloud2& msg_in) {
    if (LocalConfig::boundaryType == LOAD_FILE) {
      loadPoints(string(getenv("BULLETSIM_SOURCE_DIR")) + "/data/boundary/" + LocalConfig::boundaryFile, m_poly.points);
      // transform the poly points from ground to camera frame
      btTransform transform = waitForAndGetTransform(m_listener, LocalConfig::groundFrame, msg_in.header.frame_id).inverse();
      cloud_hull->resize(m_poly.points.size());
      for (int i = 0; i < m_poly.points.size(); i++)
        cloud_hull->at(i) = toColorPoint(transform * toBulletVector(m_poly.points[i]));
      hull_vertices.resize(1);
      for (int i = 0; i < cloud_hull->size(); i++)
        hull_vertices[0].vertices.push_back(i);
      hull_vertices[0].vertices.push_back(0);

    } else {

      ColorCloudPtr cloud_filtered = downsampleCloud(cloud, .005);
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      cloud_filtered = filterPlane(cloud_filtered, 0.03, coefficients);
      cloud_filtered = getBiggestCluster(cloud_filtered, .02);
      ColorCloudPtr cloud_projected = projectOntoPlane(cloud_filtered, coefficients); // full table cloud

//      if (LocalConfig::boundaryType == RED_MARKERS) cloud_projected = colorSpaceFilter(cloud_projected, 0, 255, 0, 166, 0, 255, CV_BGR2Lab, false, true); // red table cloud
      if (LocalConfig::boundaryType == RED_MARKERS) cloud_projected = hueFilter(cloud_projected, 160, 10, 150, 255, 100, 255);
      cloud_hull = findConvexHull(cloud_projected, hull_vertices);

      // transform the poly points from camera to ground frame
      btTransform transform = waitForAndGetTransform(m_listener, LocalConfig::groundFrame, msg_in.header.frame_id);
      m_poly.points.resize(cloud_hull->size());
      for (int i = 0; i < cloud_hull->size(); i++)
        m_poly.points[i] = toROSPoint32(transform * toBulletVector(cloud_hull->at(i)));

      savePoints(string(getenv("BULLETSIM_SOURCE_DIR")) + "/data/boundary/" + LocalConfig::boundaryFile, m_poly.points);
    }
  }

  void initTable(ColorCloudPtr cloud) {
    MatrixXf corners = getTableCornersRansac(cloud);

    Vector3f xax = corners.row(1) - corners.row(0);
    xax.normalize();
    Vector3f yax = corners.row(3) - corners.row(0);
    yax.normalize();
    Vector3f zax = xax.cross(yax);

    //if chess_board frame id exists, then z axis is already pointing up
    float zsgn = (m_listener.frameExists("chess_board") || (zax(2) < 0)) ? 1 : -1;
    xax *= zsgn;
    zax *= zsgn; // so z axis points up

    m_axes.col(0) = xax;
    m_axes.col(1) = yax;
    m_axes.col(2) = zax;

    MatrixXf rotCorners = corners * m_axes;

    m_mins = rotCorners.colwise().minCoeff();
    m_maxes = rotCorners.colwise().maxCoeff();
    m_mins(0) += LocalConfig::offset;
    m_mins(1) += LocalConfig::offset;
    m_maxes(0) -= LocalConfig::offset;
    m_maxes(1) -= LocalConfig::offset;
    m_mins(2) = rotCorners(0, 2) + LocalConfig::zClipLow;
    m_maxes(2) = rotCorners(0, 2) + LocalConfig::zClipHigh;

    m_transform.setBasis(toBulletMatrix(m_axes));
    m_transform.setOrigin(btVector3(corners(0, 0), corners(0, 1), corners(0, 2)));

    m_poly.points = toROSPoints32(toBulletVectors(corners));

    m_inited = true;
  }

  PreprocessorNode(ros::NodeHandle& nh) :
    m_inited(false), m_nh(nh), m_cloudPub(nh.advertise<sensor_msgs::PointCloud2> (LocalConfig::nodeNS + "/points", 5)),
    m_imagePub(nh.advertise<sensor_msgs::Image> (LocalConfig::nodeNS + "/image", 5)),
    m_depthPub(nh.advertise<sensor_msgs::Image> (LocalConfig::nodeNS + "/depth", 5)),
    m_maskPub(nh.advertise<sensor_msgs::Image> (LocalConfig::nodeNS + "/debug_mask", 5)),
    m_polyPub(nh.advertise<geometry_msgs::PolygonStamped> (LocalConfig::nodeNS + "/polygon", 5)),
    m_sub(nh.subscribe(LocalConfig::inputTopic, 30, &PreprocessorNode::callback0, this)),
    m_mins(-10, -10, -10), m_maxes(10, 10, 10),
    m_transform(toBulletTransform(Affine3f::Identity())),
    cloud_hull(new ColorCloud),
    threadpool(LocalConfig::threads) {}
};


int main(int argc, char* argv[]) {
  Parser parser;
  parser.addGroup(LocalConfig());
  parser.addGroup(GeneralConfig());
  parser.read(argc, argv);

  ros::init(argc, argv, "preprocessor");
  ros::NodeHandle nh(LocalConfig::nodeNS);

  PreprocessorNode tp(nh);

  ros::spin();
}
