#pragma once

#include <iostream>
#include <string>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>

#include "comm/comm.h"
#include "utils_pcl.h"

using namespace std;

struct CloudMessage : Message {
  ColorCloudPtr m_data;
  CloudMessage() : Message(), m_data(new pcl::PointCloud<pcl::PointXYZRGBA>) {}
  CloudMessage(ColorCloudPtr cloud) : Message(), m_data(cloud) {}
  CloudMessage(ColorCloudPtr cloud, Value info) : Message(info), m_data(cloud) {}
  void writeDataTo(path) const;
  void readDataFrom(path);
};


class CloudGrabber {
public:
  bool m_enabled;
  int m_downsample;
  int m_cbCount;
  FilePublisher m_pub;
  string m_topic;

  CloudGrabber(string topic, int downsample);

  void run();
  virtual void loop();
  virtual void cloud_cb(const ConstColorCloudPtr& cloud);
  virtual void processCloud(ColorCloudPtr cloud);
};

class PausingCloudGrabber : public CloudGrabber {
public:
  PausingCloudGrabber(string topic);
  void loop();
  void cloud_cb(const ConstColorCloudPtr& cloud);
};
