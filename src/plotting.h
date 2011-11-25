#pragma once
#include "environment.h"
#include <iostream>
#include <vector>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/StateSet>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <btBulletDynamicsCommon.h>


class PlotPoints : public EnvironmentObject {

  osg::ref_ptr<osg::Geometry> m_geom;
  osg::ref_ptr<osg::Geode> m_geode;
public:
  typedef boost::shared_ptr<PlotPoints> Ptr;

  PlotPoints();

  void init(){
    getEnvironment()->osg->root->addChild(m_geode.get());
  }
  void prePhysics(){}// no physics
  void preDraw(){} // no transforms needed
  void destroy(){} // all ref_ptrs
  void setPoints(const osg::ref_ptr<osg::Vec3Array>& osgPts, const osg::ref_ptr<osg::Vec4Array>& osgCols);
  void setPoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
  void setPoints(const std::vector<btVector3>& pts);
};



