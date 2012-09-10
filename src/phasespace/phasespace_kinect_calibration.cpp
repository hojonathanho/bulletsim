#include "simulation/simplescene.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "clouds/pcl_typedefs.h"
#include "utils/config.h"
#include "clouds/cloud_ops.h"
#include "phasespace/phasespace.h"
#include "utils/conversions.h"
#include "utils/utils_vector.h"
#include "tracking/utils_tracking.h"

using sensor_msgs::PointCloud2;
using namespace std;
using namespace Eigen;

struct LocalConfig : Config {
  static string cloudTopic;
  static string phasespaceTopic;
  static string filename;
  static vector<ledid_t> commonLedIds;
  static vector<ledid_t> kinectLedIds;

  LocalConfig() : Config() {
    params.push_back(new Parameter<string>("cloudTopic", &cloudTopic, "Topic of cloud coming from the camera to be calibrated."));
    params.push_back(new Parameter<string>("phasespaceTopic", &phasespaceTopic, "Topic of the OWLPhasespaceMsg's."));
    params.push_back(new Parameter<string>("filename", &filename, "The computed rigid body info is saved at this file."));
    params.push_back(new Parameter<vector<ledid_t> >("commonLedIds", &commonLedIds, "ID of the LEDs that are seen by both cameras. The order matters."));
    params.push_back(new Parameter<vector<ledid_t> >("kinectLedIds", &kinectLedIds, "ID of the LEDs that are on the kinect's frame."));
  }
};

string LocalConfig::cloudTopic = "/kinect1/depth_registered/points";
string LocalConfig::phasespaceTopic = "/phasespace_markers";
//string LocalConfig::filename = "/home/alex/rll/bulletsim/data/phasespace_rigid_info/pr2head";
string LocalConfig::filename = "/home/alex/rll/bulletsim/data/phasespace_rigid_info/tripod";
static const ledid_t commonLedIds_a[] = { 24,8,9,10,11,12,13,14,15 };
vector<ledid_t> LocalConfig::commonLedIds = std::vector<ledid_t>(commonLedIds_a, commonLedIds_a+sizeof(commonLedIds_a)/sizeof(ledid_t));
//static const ledid_t kinectLedIds_a[] = { 0,1,2,3,4,5,6 };
static const ledid_t kinectLedIds_a[] = { 42,43,44,45,46 };
vector<ledid_t> LocalConfig::kinectLedIds = std::vector<ledid_t>(kinectLedIds_a, kinectLedIds_a+sizeof(kinectLedIds_a)/sizeof(ledid_t));

ColorCloudPtr cloud(new ColorCloud());
cv::Mat image;
MarkerSystemPhasespaceMsg::Ptr marker_system;
ColorCloudPtr led_points_k(new ColorCloud());
vector<cv::Point2i> led_pixels;

void plotPointCloud (ColorCloudPtr cloud, PlotSpheres::Ptr plot_spheres, float r, float g, float b) {
	osg::ref_ptr<osg::Vec3Array> centers = new osg::Vec3Array();
	osg::ref_ptr<osg::Vec4Array> rgba = new osg::Vec4Array();
	for (int i=0; i<cloud->size(); i++) {
		centers->push_back(osg::Vec3(cloud->at(i).x, cloud->at(i).y, cloud->at(i).z));
		rgba->push_back(osg::Vec4(r,g,b,1));
		//rgba->push_back(osg::Vec4(((float)i)/((float)cloud->size()-1)*r,((float)i)/((float)cloud->size()-1)*g,((float)i)/((float)cloud->size()-1)*b,1));
	}
	vector<float> sizes(cloud->size(), 0.005*METERS);
	plot_spheres->plot(centers, rgba, sizes);
}

void cloudCallback (const sensor_msgs::PointCloud2ConstPtr& cloudMsg) {
	pcl::fromROSMsg(*cloudMsg, *cloud);
  image = toCVMatImage(cloud);
}

void phasespaceCallback (const bulletsim_msgs::OWLPhasespaceConstPtr& phasespaceMsg) {
	marker_system->updateIteration(*phasespaceMsg);
}

void mouseCallback( int event, int x, int y, int flags, void* param ){
	switch( event ){
		case CV_EVENT_LBUTTONUP:
			ColorPoint pt = getCorrespondingPoint(cloud, cv::Point2i(x,y));
			pt.x *= METERS;
			pt.y *= METERS;
			pt.z *= METERS;
			led_points_k->push_back(pt);
	  	printf("Pixel (%d,%d)\t Depth %.4f\n", x, y, sqrt((pt.x*pt.x+pt.y*pt.y+pt.z*pt.z)));
	  	led_pixels.push_back(cv::Point2i(x,y));
	  	break;
	}
}

int main(int argc, char* argv[]) {
  GeneralConfig::scale = 1;

  Parser parser;
	parser.addGroup(GeneralConfig());
	parser.addGroup(SceneConfig());
  parser.addGroup(LocalConfig());
	parser.addGroup(PhasespaceConfig());
  parser.read(argc, argv);

  ros::init(argc, argv,"phasespace_kinect_calibration");
  ros::NodeHandle nh;

  ros::Subscriber cloudSub = nh.subscribe(LocalConfig::cloudTopic, 1, &cloudCallback);

  cout << "Click at the LEDs in the image in the order of LEDs IDs: " << LocalConfig::commonLedIds << endl;

  cv::namedWindow("Calibration", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);
  cv::setMouseCallback("Calibration", mouseCallback);

  char key = cv::waitKey(15);
  while (ros::ok() && key!='q') {
  	if (!image.empty()) {
  		for (int i=0; i<led_pixels.size(); i++)
  			cv::circle(image, cv::Point2i(led_pixels[i].x, led_pixels[i].y), 5, cv::Scalar(0,255,0), 1);
  		cv::imshow("Calibration", image);
  	}
  	key = cv::waitKey(15);
  	if (led_pixels.size() == LocalConfig::commonLedIds.size()) break;
  	ros::spinOnce();
  }
  if (key == 'q') exit(0);
  cv::destroyWindow("Calibration");

  Scene scene;
  scene.env->remove(scene.ground);

	MarkerPointCollection::Ptr marker_common_points(new MarkerPointCollection(LocalConfig::commonLedIds, scene.env));
	MarkerPointCollection::Ptr marker_kinect_points(new MarkerPointCollection(LocalConfig::kinectLedIds, scene.env));
	marker_system.reset(new MarkerSystemPhasespaceMsg());
	marker_system->add(marker_common_points);
	marker_system->add(marker_kinect_points);
	ros::Subscriber phasespaceSub = nh.subscribe(LocalConfig::phasespaceTopic, 1, &phasespaceCallback);
	while (ros::ok) {
		if (marker_system->isAllValid()) break;
  	ros::spinOnce();
	}

	bool verifyingAlignment = true;
  scene.addVoidKeyCallback('v',boost::bind(toggle, &verifyingAlignment));
  scene.addVoidKeyCallback('q',boost::bind(exit, 0));
  scene.startViewer();
	util::drawAxes(btTransform::getIdentity(),0.10*METERS, scene.env);

  ColorCloudPtr led_points_ps(new ColorCloud());
  for (int ind=0; ind<LocalConfig::commonLedIds.size(); ind++)
  	led_points_ps->push_back(toColorPoint(marker_common_points->getPosition(ind)));
	assert(led_points_k->size() == led_points_ps->size());
	for (int i=(led_points_k->size()-1); i>=0; i--) {
		if (!pointIsFinite(led_points_k->at(i))) {
			led_points_k->erase(led_points_k->begin()+i);
			led_points_ps->erase(led_points_ps->begin()+i);
		}
	}
  Matrix4f tf_k2ps;
  pcl::registration::TransformationEstimationSVD<ColorPoint, ColorPoint> estimation_svd;
  estimation_svd.estimateRigidTransformation(*led_points_k, *led_points_ps, tf_k2ps);
  pcl::transformPointCloud(*led_points_k.get(), *led_points_k.get(), tf_k2ps);

  cout << "Verify that the alignment is correct by pressing 'v'. Or press 'q' to quit." << endl;

	util::drawAxes(toBulletTransform(Affine3f(tf_k2ps)),0.10*METERS, scene.env);

	PlotSpheres::Ptr plot_spheres_k(new PlotSpheres());
	PlotSpheres::Ptr plot_spheres_ps(new PlotSpheres());
	scene.env->add(plot_spheres_k);
	scene.env->add(plot_spheres_ps);
	while (ros::ok() && verifyingAlignment) {
		plotPointCloud(led_points_k, plot_spheres_k, 1,0,0);   // red points: common leds as clicked by the user
		plotPointCloud(led_points_ps, plot_spheres_ps, 0,1,0); // green points: common leds as seen by phasespace
		marker_common_points->plot();
		marker_kinect_points->plot();
		scene.draw();

		ros::spinOnce();
	}

	Affine3f tf_ps2k = ((Affine3f) tf_k2ps).inverse();
	vector<Vector3f> kinect_led_positions;
	for(int ind=0; ind<LocalConfig::kinectLedIds.size(); ind++) {
		kinect_led_positions.push_back( tf_ps2k * marker_kinect_points->getPosition(ind) );
		kinect_led_positions[ind] /= METERS;
		cout << "Led ID " << LocalConfig::kinectLedIds[ind] << "\t" << kinect_led_positions[ind].transpose() << endl;
	}
	savePhasespaceRigid(LocalConfig::filename, LocalConfig::kinectLedIds, kinect_led_positions);

	return 0;
}
