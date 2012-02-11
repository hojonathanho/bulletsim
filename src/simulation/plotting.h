#pragma once
#include "environment.h"
#include <iostream>
#include <vector>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/StateSet>
#include <btBulletDynamicsCommon.h>


class PlotObject : public EnvironmentObject {
protected:
  osg::Vec4 m_defaultColor;
  osg::ref_ptr<osg::Geometry> m_geom;
  osg::ref_ptr<osg::Geode> m_geode;
  osg::ref_ptr<osg::StateSet> m_stateset;
public:
  typedef boost::shared_ptr<PlotObject> Ptr;

  EnvironmentObject::Ptr copy(Fork &f) const { return Ptr(new PlotObject(*this)); }

  void init(){
    getEnvironment()->osg->root->addChild(m_geode.get());
  }
  void prePhysics(){}// no physics
  void preDraw(){};
  void destroy(){} 
  void setDefaultColor(float r, float g, float b, float a);
  void forceTransparency(float a);
};

class PlotPoints : public PlotObject {
  osg::ref_ptr<osg::Vec4Array> colors;
public:
  typedef boost::shared_ptr<PlotPoints> Ptr;
  PlotPoints(float size=5);
  void setPoints(const osg::ref_ptr<osg::Vec3Array>& osgPts, const osg::ref_ptr<osg::Vec4Array>& osgCols);
  void setPoints(const osg::ref_ptr<osg::Vec3Array>& osgPts);
  void setPoints(const std::vector<btVector3>& pts, const std::vector<btVector4>& cols);
  void setPoints(const std::vector<btVector3>& pts);
  void forceTransparency(float a);
};

class PlotLines : public PlotObject {

public:
  typedef boost::shared_ptr<PlotLines> Ptr;
  PlotLines(float width=5);
  void setPoints(const std::vector<btVector3>& pts, const std::vector<btVector4>& cols);
  void setPoints(const std::vector<btVector3>& pts);
  void setPoints(const osg::ref_ptr<osg::Vec3Array>& pts, const osg::ref_ptr<osg::Vec4Array>& cols);
  void setPoints(const osg::ref_ptr<osg::Vec3Array>& pts);
  void forceTransparency(float a);
};


