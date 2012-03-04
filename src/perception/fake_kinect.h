#pragma once

#include "clouds/utils_pcl.h"
#include "simulation/environment.h"
#include "comm/comm.h"

class FakeKinect {

public:
  osg::ref_ptr<osg::Camera> m_cam;
  osgViewer::Viewer m_viewer;
  osg::ref_ptr<KinectCallback> m_cb;
  FilePublisher m_pub;
  FakeKinect(OSGInstance::Ptr, Affine3f worldFromCam);
  void sendMessage();

};
