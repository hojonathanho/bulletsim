#pragma once

#include <iostream>
#include <string>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>

#include "comm/comm.h"
using namespace comm;
#include "utils_pcl.h"

using namespace std;

struct CloudMessage : Message {
  ColorCloudPtr m_data;
  CloudMessage() : Message(), m_data(new pcl::PointCloud<ColorPoint>) {}
  CloudMessage(ColorCloudPtr cloud) : Message(), m_data(cloud) {}
  CloudMessage(ColorCloudPtr cloud, Json::Value info) : Message(info), m_data(cloud) {}
  void writeDataTo(fs::path) const;
  void readDataFrom(fs::path);
};

//class PCCSubscriber : public Subscriber {
//public:
//  PCCSubscriber(string filename);
//  bool recv(Message& message, bool enableWait=true);
//  bool skip();
//};

//class CloudGrabber {
//public:
//  bool m_enabled;
//  int m_downsample;
//  int m_cbCount;
//  FilePublisher m_pub;
//  string m_topic;
//
//  CloudGrabber(string topic, int downsample);
//
//  void run();
//  virtual void loop();
//  virtual void cloud_cb(const ConstColorCloudPtr& cloud);
//  virtual void processCloud(ColorCloudPtr cloud);
//};
//
//class PausingCloudGrabber : public CloudGrabber {
//public:
//  PausingCloudGrabber(string topic);
//  void loop();
//  void cloud_cb(const ConstColorCloudPtr& cloud);
//};
