#pragma once
#include "environment.h"
#include <iostream>
#include <vector>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/StateSet>
#include <btBulletDynamicsCommon.h>

#if BUILD_PERCEPTION
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#endif //BUILD_PERCEPTION


class PlotObject : public EnvironmentObject {
protected:
  osg::Vec4 m_defaultColor;
  osg::ref_ptr<osg::Geometry> m_geom;
  osg::ref_ptr<osg::Geode> m_geode;
  osg::ref_ptr<osg::StateSet> m_stateset;
public:
  typedef boost::shared_ptr<PlotObject> Ptr;

  EnvironmentObject::Ptr copy() { return Ptr(new PlotObject(*this)); }

  void init(){
    getEnvironment()->osg->root->addChild(m_geode.get());
  }
  void prePhysics(){}// no physics
  void preDraw(){};//{ m_geode->setStateSet(m_stateset);}
  void destroy(){} 
  void setDefaultColor(float r, float g, float b, float a);
};


class PlotPoints : public PlotObject {
public:
  typedef boost::shared_ptr<PlotPoints> Ptr;
  PlotPoints(float size=5);
  void setPoints(const osg::ref_ptr<osg::Vec3Array>& osgPts, const osg::ref_ptr<osg::Vec4Array>& osgCols);
  void setPoints(const std::vector<btVector3>& pts);
#ifdef BUILD_PERCEPTION
  void setPoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
#endif
};

class PlotLines : public PlotObject {

public:
  typedef boost::shared_ptr<PlotLines> Ptr;
  PlotLines(float width=5);
  void setPoints(const std::vector<btVector3>& pts1, const std::vector<btVector3>& pts2);
  void setPoints(const osg::ref_ptr<osg::Vec3Array>& pts, const osg::ref_ptr<osg::Vec4Array>& cols);
};


