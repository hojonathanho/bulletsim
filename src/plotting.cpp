#include "plotting.h"
#include "util.h"
#include <osg/PointSprite>
#include <osg/Point>
#include <osg/LineWidth>
#include <osg/Geometry>
#include <osg/StateSet>
#include <boost/foreach.hpp>

using namespace std;
using namespace util;

// based on galaxy example in osg docs

void PlotObject::setDefaultColor(float r, float g, float b, float a) {
  m_defaultColor = osg::Vec4(r,g,b,a);
}


PlotPoints::PlotPoints(float size) {
  m_geode = new osg::Geode();
  m_geom = new osg::Geometry();
  m_geom->setDataVariance(osg::Object::DYNAMIC);
  m_geode->addDrawable(m_geom);
  setDefaultColor(1,1,1,1);

  osg::ref_ptr<osg::StateSet> m_stateset = new osg::StateSet();
  osg::Point *point = new osg::Point();

  //  m_stateset->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
  m_stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

  point->setSize(size);
  m_stateset->setAttribute(point);
  m_geode->setStateSet(m_stateset);
}

void PlotPoints::setPoints(const osg::ref_ptr<osg::Vec3Array>& osgPts, const osg::ref_ptr<osg::Vec4Array>& osgCols) {
  int nPts = osgPts->getNumElements();
  m_geom->setVertexArray(osgPts);
  m_geom->setColorArray(osgCols);
  m_geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
  m_geom->getPrimitiveSetList().clear();
  m_geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,nPts));
}

void PlotPoints::setPoints(const osg::ref_ptr<osg::Vec3Array>& osgPts) {
  osg::ref_ptr<osg::Vec4Array> osgCols = new osg::Vec4Array(osgPts->size());
  BOOST_FOREACH(osg::Vec4& col, *osgCols) col = m_defaultColor;
  setPoints(osgPts, osgCols);
}

#if BUILD_PERCEPTION
inline bool pointIsFinite(const pcl::PointXYZRGB& pt) {
  return isfinite(pt.x) && isfinite(pt.y) && isfinite(pt.z);
}

void PlotPoints::setPoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
  osg::ref_ptr<osg::Vec3Array> osgPts = new osg::Vec3Array();
  osg::ref_ptr<osg::Vec4Array>  osgCols = new osg::Vec4Array();
  osgPts->reserve(cloud->size());
  osgCols->reserve(cloud->size());
  BOOST_FOREACH(const pcl::PointXYZRGB& pt, cloud->points){
    if (pointIsFinite(pt)) {
      osgPts->push_back(osg::Vec3(pt.x,pt.y,pt.z));
      osgCols->push_back(osg::Vec4(pt.r/255.,pt.g/255.,pt.b/255.,1));
    }
  }
  setPoints(osgPts,osgCols);
}
#endif //BUILD_PERCEPTION

void PlotPoints::setPoints(const vector<btVector3>& pts, const vector<btVector4>& cols) {
  setPoints(toVec3Array(pts), toVec4Array(cols));
}
void PlotPoints::setPoints(const vector<btVector3>& pts) {
  setPoints(toVec3Array(pts));
}

PlotLines::PlotLines(float width) {
  setDefaultColor(1,1,1,1);

  m_geode = new osg::Geode();
  m_geom = new osg::Geometry();
  m_geom->setDataVariance(osg::Object::DYNAMIC);
  m_geode->addDrawable(m_geom);

  osg::ref_ptr<osg::StateSet> m_stateset = new osg::StateSet();
  osg::LineWidth *linewidth = new osg::LineWidth();
  linewidth->setWidth(width);
  //m_stateset->setAttributeAndModes(linewidth,osg::StateAttribute::ON);
  m_stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
  m_stateset->setAttribute(linewidth);
  m_geode->setStateSet(m_stateset);

}

void PlotLines::setPoints(const vector<btVector3>& pts, const vector<btVector4>& cols) {
  setPoints(toVec3Array(pts),  toVec4Array(cols));
}

void PlotLines::setPoints(const vector<btVector3>& pts) {
  osg::ref_ptr<osg::Vec4Array> osgCols = new osg::Vec4Array(pts.size());
  BOOST_FOREACH(osg::Vec4& col, *osgCols) col = m_defaultColor;
  setPoints(toVec3Array(pts),  osgCols);
}


void PlotLines::setPoints(const osg::ref_ptr<osg::Vec3Array>& osgPts, const osg::ref_ptr<osg::Vec4Array>& osgCols) {
  int nPts = osgPts->getNumElements();
  m_geom->setColorArray(osgCols);
  m_geom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE);
  m_geom->setVertexArray(osgPts);
  m_geom->getPrimitiveSetList().clear();
  m_geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,nPts));
}

void PlotLines::setPoints(const osg::ref_ptr<osg::Vec3Array>& osgPts) {
  osg::ref_ptr<osg::Vec4Array> osgCols = new osg::Vec4Array(osgPts->size());
  BOOST_FOREACH(osg::Vec4& col, *osgCols) col = m_defaultColor;
  setPoints(osgPts, osgCols);
}

