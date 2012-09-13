#include "simulation/recording.h"
#include <ros/common.h>
#include <sensor_msgs/Image.h>
#include "tracking/utils_tracking.h"
#include <cv_bridge/cv_bridge.h>
#include <boost/thread.hpp>

struct DemoRecorder {
public:
  typedef boost::shared_ptr<DemoRecorder> Ptr;
  ros::Subscriber m_imageSub;
  sensor_msgs::Image m_lastImg;
  ScreenRecorder m_sr;
  ConsecutiveImageWriter m_cir;
  osgViewer::Viewer& m_viewer;

  void imageCB(const sensor_msgs::Image& img) {
    m_lastImg = img;
  }

  void frameLoop(float hz) {
    ros::Rate rate(hz);
    while (ros::ok()) {
      rate.sleep();
      frame();
    }
  }

  DemoRecorder(ros::NodeHandle nh, std::string imageTopic, osgViewer::Viewer& viewer) :
  m_sr(viewer), m_cir("/tmp/camera_images"), m_viewer(viewer) {
    m_imageSub = nh.subscribe(imageTopic, 1, &DemoRecorder::imageCB, this);
  }

  void frame() {
    if (m_lastImg.width==0) return;
    cv::Mat mat = cv_bridge::toCvCopy(m_lastImg)->image;
    m_cir.write(mat);
    m_sr.snapshot();
  }

};
