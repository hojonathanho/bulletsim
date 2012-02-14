#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include "comm_pcl.h"
#include "utils_pcl.h"
#include "signal.h"

bool wantsExit = false;
void setWantsExit(int) {
  wantsExit = true;
}



void pressEnterToContinue() {
  std::cout << "Press ENTER to continue... " << flush;
  std::cin.ignore( std::numeric_limits <std::streamsize> ::max(), '\n' );
}



CloudGrabber::CloudGrabber(string topic, int downsample)
  : 
  m_topic(topic),
  m_pub(topic, "pcd"),
  m_downsample(downsample),
  m_cbCount(0),
  m_enabled(true)
{}

void CloudGrabber::cloud_cb(const ConstColorCloudPtr& cloud) {
  if (!m_enabled || wantsExit) {
    cout << "cloub_cb: not writing" << endl;
    return;
  }
  if (m_cbCount % m_downsample == 0) m_pub.send(CloudMessage(boost::const_pointer_cast< pcl::PointCloud<pcl::PointXYZRGB> >(cloud)));
    // not nice to use a pointer cast but i don't know a way around it since callback signature is const but class member isn't
  m_cbCount++;       
}


void CloudGrabber::run ()  {
  pcl::Grabber* interface = new pcl::OpenNIGrabber();
  boost::function<void (const ConstColorCloudPtr&)> f = boost::bind (&CloudGrabber::cloud_cb, this, _1);
  interface->registerCallback(f);
  interface->start ();
  signal(SIGINT, &setWantsExit);
  loop();
  interface->stop ();
}

void CloudGrabber::loop() {
  while (!wantsExit) {
    m_enabled = !getThrottled(m_topic);
    sleep (.001);
  }
}

PausingCloudGrabber::PausingCloudGrabber(string topic) : CloudGrabber(topic, 0) {
  m_enabled = false;
}



void PausingCloudGrabber::loop() {
  while (!wantsExit) {
    m_enabled = true;
    pressEnterToContinue();
  }
}

void PausingCloudGrabber::cloud_cb(const ConstColorCloudPtr& cloud) {
  if (!m_enabled || wantsExit) {
    return;
  }
  else {
    m_pub.send(CloudMessage(boost::const_pointer_cast< pcl::PointCloud<pcl::PointXYZRGB> >(cloud))); 
    m_enabled=false;
  }
}


void CloudMessage::writeDataTo(path p) const {
  pcl::io::savePCDFileBinary(p.string().c_str(), *m_data);
}
void CloudMessage::readDataFrom(path p) {
  m_data = readPCD(p.string());
}
