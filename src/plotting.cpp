#include "plotting.h"
#include <osg/PointSprite>
#include <osg/Point>
#include <osg/LineWidth>
#include <osg/Geometry>
#include <osg/StateSet>

using namespace std;


// based on galaxy example in osg docs


PlotPoints::PlotPoints(float size) {
  m_geode = new osg::Geode();
  m_geom = new osg::Geometry();
  m_geode->addDrawable(m_geom);

  osg::ref_ptr<osg::StateSet> m_stateset = new osg::StateSet();
  osg::Point *point = new osg::Point();
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

void PlotPoints::setPoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
  osg::ref_ptr<osg::Vec3Array> osgPts = new osg::Vec3Array();
  osg::ref_ptr<osg::Vec4Array>  osgCols = new osg::Vec4Array();
  for (int i=0; i < cloud->size(); i++) {
    pcl::PointXYZRGB pt = cloud->at(i);
    osgPts->push_back(osg::Vec3(pt.x,pt.y,pt.z));
    osgCols->push_back(osg::Vec4(pt.r/255.,pt.g/255.,pt.b/255.,1));


  }
  setPoints(osgPts,osgCols);

}
void PlotPoints::setPoints(const vector<btVector3>& pts) {
  osg::ref_ptr<osg::Vec3Array> osgPts(new osg::Vec3Array());
  osg::ref_ptr<osg::Vec4Array>  osgCols(new osg::Vec4Array());
  for (int i = 0; i < pts.size(); i++) {
    btVector3 pt = pts[i];
    osgPts->push_back(osg::Vec3(pt.getX(),pt.getY(),pt.getZ()));
    osgCols->push_back(osg::Vec4(1,0,0,1));
  }
  setPoints(osgPts,osgCols);
}

PlotLines::PlotLines(float width) {
  m_geode = new osg::Geode();
  m_geom = new osg::Geometry();
  m_geode->addDrawable(m_geom);

  osg::ref_ptr<osg::StateSet> m_stateset = new osg::StateSet();
  osg::LineWidth *linewidth = new osg::LineWidth();
  linewidth->setWidth(width);
  //m_stateset->setAttributeAndModes(linewidth,osg::StateAttribute::ON);
  m_stateset->setAttribute(linewidth);
  m_geode->setStateSet(m_stateset);

}

void PlotLines::setPoints(const vector<btVector3>& pts1, const vector<btVector3>& pts2) {
  assert(pts1.size()==pts2.size());
  osg::ref_ptr<osg::Vec3Array> osgPts(new osg::Vec3Array());
  osg::ref_ptr<osg::Vec4Array> osgCols(new osg::Vec4Array());
  for (int i=0; i<pts1.size(); i++) {
    btVector3 pt1 = pts1[i];
    btVector3 pt2 = pts2[i];
    osgPts->push_back(osg::Vec3(pt1.getX(),pt1.getY(),pt1.getZ()));
    osgPts->push_back(osg::Vec3(pt2.getX(),pt2.getY(),pt2.getZ()));
    osgCols->push_back(osg::Vec4(0,0,1,1));
  }
  setPoints(osgPts,osgCols);
}

void PlotLines::setPoints(const osg::ref_ptr<osg::Vec3Array>& osgPts, const osg::ref_ptr<osg::Vec4Array>& osgCols) {
  int nPts = osgPts->getNumElements();
  m_geom->setColorArray(osgCols);
  m_geom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE);
  m_geom->setVertexArray(osgPts);
  m_geom->getPrimitiveSetList().clear();
  m_geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,nPts));
}

