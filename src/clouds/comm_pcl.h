#pragma once

#include <iostream>
#include <string>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>

#include "comm/comm2.h"
#include "utils_pcl.h"

using namespace std;

struct CloudMessage : Message {
  ColorCloudPtr m_data;
  CloudMessage() : Message(), m_data(new pcl::PointCloud<pcl::PointXYZRGB>) {}
  CloudMessage(ColorCloudPtr cloud) : Message(), m_data(cloud) {}
  CloudMessage(ColorCloudPtr cloud, Value info) : Message(info), m_data(cloud) {}
  void writeDataTo(path) const;
  void readDataFrom(path);
};


class CloudGrabber {
public:
  bool m_doPause;
  int m_downsample;
  int m_cbCount;
  FilePublisher m_pub;

  CloudGrabber(string topic, bool doPause, int downsample);

  void run();
  void cloud_cb_(const ConstColorCloudPtr& cloud);
};
