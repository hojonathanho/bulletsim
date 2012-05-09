#pragma once

#include "clouds/utils_pcl.h"
#include "simulation/environment.h"
#include "comm/comm.h"
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <osgViewer/Viewer>

class KinectCallback : public osg::Camera::DrawCallback {
    
public:

  int width;
  int height;

  bool m_done;

  osg::ref_ptr<osg::Image> depthbufferimg;
  osg::ref_ptr<osg::Image> colorbufferimg;

  ColorCloudPtr m_cloud;

  KinectCallback( osg::Camera* camera ) ;
  void operator () ( osg::RenderInfo& info ) const;    
};


class FakeKinect {

public:
  osg::ref_ptr<osg::Camera> m_cam;
  osgViewer::Viewer m_viewer;
  osg::ref_ptr<KinectCallback> m_cb;
  boost::shared_ptr<FilePublisher> m_pub;
  FakeKinect(OSGInstance::Ptr, const Eigen::Affine3f &worldFromCam, bool usePub=true);
  void setWorldFromCam(const Eigen::Affine3f &worldFromCam);
  void sendMessage();
  snapshot(ColorCloudPtr &cloud, ColorCloudPtr normals);
};
