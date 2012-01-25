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



CloudGrabber::CloudGrabber(string topic, bool doPause, int downsample)
  : 
  m_pub(topic, "pcd"),
  m_doPause(doPause),
  m_downsample(downsample),
  m_cbCount(0)
{}

void CloudGrabber::cloud_cb_ (const ConstColorCloudPtr& cloud) {
  if (wantsExit) return;
  if (m_doPause) {
    pressEnterToContinue();
  }

  if (m_doPause || m_cbCount % m_downsample == 0) {
    m_pub.send(CloudMessage(boost::const_pointer_cast< pcl::PointCloud<pcl::PointXYZRGB> >(cloud)));
    cout << "writing pcd file" << endl;
  }
  m_cbCount++;       
}


void CloudGrabber::run ()  {
  pcl::Grabber* interface = new pcl::OpenNIGrabber();

  boost::function<void (const ConstColorCloudPtr&)> f = boost::bind (&CloudGrabber::cloud_cb_, this, _1);

  interface->registerCallback(f);
  interface->start ();

  signal(SIGINT, &setWantsExit);

  while (!wantsExit) {
    sleep (1);
  }
  interface->stop ();
}

void CloudMessage::writeDataTo(path p) const {
  pcl::io::savePCDFileBinary(p.string().c_str(), *m_data);
}
void CloudMessage::readDataFrom(path p) {
  m_data = readPCD(p.string());
}
