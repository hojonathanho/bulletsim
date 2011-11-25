// from osg galaxy example

#include <iostream>
#include <vector>
#include "simplescene.h"
#include <osg/PointSprite>
#include <osg/Point>
#include <osg/Geometry>
#include <osg/StateSet>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

using namespace std;

osg::StateSet* makeStateSet(float size)
{
  osg::StateSet *set = new osg::StateSet();
  osg::Point *point = new osg::Point();
  point->setSize(size);
  set->setAttribute(point);
  return set;
}

class PlotPoints : public EnvironmentObject {

  osg::ref_ptr<osg::Geometry> m_geom;
  osg::ref_ptr<osg::Geode> m_geode;
public:
  typedef boost::shared_ptr<PlotPoints> Ptr;

  PlotPoints() {
    m_geode = new osg::Geode();
    m_geom = new osg::Geometry();
    m_geode->addDrawable(m_geom);
    m_geode->setStateSet(makeStateSet(1.f));
  }

  void init() {
    getEnvironment()->osg->root->addChild(m_geode);
  }

  void prePhysics() {} // no physics
  void preDraw() {} // no transforms needed
  void destroy() {} // all ref_ptrs
  void setPoints(const osg::ref_ptr<osg::Vec3Array>& osgPts, const osg::ref_ptr<osg::Vec4Array>& osgCols) {
    int nPts = osgPts->getNumElements();
    m_geom->setVertexArray(osgPts);
    m_geom->setColorArray(osgCols);
    m_geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    m_geom->getPrimitiveSetList().clear();
    m_geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,nPts));

  }

  void setPoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
    osg::ref_ptr<osg::Vec3Array> osgPts = new osg::Vec3Array();
    osg::ref_ptr<osg::Vec4Array>  osgCols = new osg::Vec4Array();
    for (int i=0; i < cloud->size(); i++) {
      pcl::PointXYZRGB pt = cloud->at(i);
      osgPts->push_back(osg::Vec3(pt.x,pt.y,pt.z));
      //see http://docs.pointclouds.org/trunk/structpcl_1_1_r_g_b.html
      int rgb = pt.rgb;
      uint8_t r = (rgb >> 16) & 0x0000ff;
      uint8_t g = (rgb >> 8)  & 0x0000ff;
      uint8_t b = (rgb)     & 0x0000ff;
      //osgCols->push_back(osg::Vec4(r/255.,g/255.,b/255.,1));
      osgCols->push_back(osg::Vec4(pt.r/255.,pt.g/255.,pt.b/255.,1));
      if (i%100 == 0) cout << pt.r/255. <<  " " << pt.g/255. << " " << pt.b/255. << endl;
      //osgCols->push_back(osg::Vec4(1,0,0,1));

    }
    setPoints(osgPts,osgCols);

    }
  void setPoints(const vector<btVector3>& pts) {
    osg::ref_ptr<osg::Vec3Array> osgPts(new osg::Vec3Array());
    osg::ref_ptr<osg::Vec4Array>  osgCols(new osg::Vec4Array());
    for (int i = 0; i < pts.size(); i++) {
      btVector3 pt = pts[i];
      osgPts->push_back(osg::Vec3(pt.getX(),pt.getY(),pt.getZ()));
      osgCols->push_back(osg::Vec4(1,0,0,1));
    }
    setPoints(osgPts,osgCols);
  }

};



/*
int main() {
  Scene s = Scene(false,false,false);


  /*
  PlotPoints::Ptr pc(new PlotPoints());
  vector<btVector3> pts;
  pts.push_back(btVector3(0,0,0));
  pts.push_back(btVector3(1,0,0));
  pts.push_back(btVector3(0,1,0));
  pts.push_back(btVector3(1,1,0));
  pts.push_back(btVector3(0,0,1));
  pts.push_back(btVector3(1,0,1));
  pts.push_back(btVector3(0,1,1));
  pts.push_back(btVector3(1,1,1));
  
  pc->setPoints(pts);*/


  const string pcdfile = "/home/joschu/Data/pink_rope/0003.pcd";
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcdfile, *cloud) == -1) {
    PCL_ERROR(("couldn't read file " + pcdfile + "\n").c_str());
    return -1;
  }
  PlotPoints::Ptr pc(new PlotPoints());
  pc->setPoints(cloud);


  s.env->add(pc);

  s.viewerLoop();
}
*/
