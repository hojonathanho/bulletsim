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


class PlotObject : public EnvironmentObject {
protected:
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
};


class PlotPoints : public PlotObject {
public:
  typedef boost::shared_ptr<PlotPoints> Ptr;
  PlotPoints(float size=5);
  void setPoints(const osg::ref_ptr<osg::Vec3Array>& osgPts, const osg::ref_ptr<osg::Vec4Array>& osgCols);
  void setPoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
  void setPoints(const std::vector<btVector3>& pts);
};

class PlotLines : public PlotObject {

public:
  typedef boost::shared_ptr<PlotLines> Ptr;
  PlotLines(float width=5);
  void setPoints(const std::vector<btVector3>& pts1, const std::vector<btVector3>& pts2);
  void setPoints(const osg::ref_ptr<osg::Vec3Array>& pts, const osg::ref_ptr<osg::Vec4Array>& cols);
};


