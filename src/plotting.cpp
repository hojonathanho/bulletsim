
#include "plotting.h"
#include <osg/PointSprite>
#include <osg/Point>
#include <osg/Geometry>
#include <osg/StateSet>

using namespace std;


// based on galaxy example in osg docs

osg::StateSet* makeStateSet(float size)
{
  osg::StateSet *set = new osg::StateSet();
  osg::Point *point = new osg::Point();
  point->setSize(size);
  set->setAttribute(point);
  return set;
}


PlotPoints::PlotPoints() {
  m_geode = new osg::Geode();
  m_geom = new osg::Geometry();
  m_geode->addDrawable(m_geom);
}


void PlotPoints::setPoints(const osg::ref_ptr<osg::Vec3Array>& osgPts, const osg::ref_ptr<osg::Vec4Array>& osgCols) {
  int nPts = osgPts->getNumElements();
  m_geom->setVertexArray(osgPts);
  m_geom->setColorArray(osgCols);
  m_geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
  m_geom->getPrimitiveSetList().clear();
  m_geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,nPts));
  m_geode->setStateSet(makeStateSet(2.f));


}

void PlotPoints::setPoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
  osg::ref_ptr<osg::Vec3Array> osgPts = new osg::Vec3Array();
  osg::ref_ptr<osg::Vec4Array>  osgCols = new osg::Vec4Array();
  for (int i=0; i < cloud->size(); i++) {
    pcl::PointXYZRGB pt = cloud->at(i);
    osgPts->push_back(osg::Vec3(pt.x,pt.y,pt.z));
    //see http://docs.pointclouds.org/trunk/structpcl_1_1_r_g_b.html
    osgCols->push_back(osg::Vec4(pt.r/255.,pt.g/255.,pt.b/255.,1));
    if (i%10000 == 0) cout << pt.r/255. <<  " " << pt.g/255. << " " << pt.b/255. << endl;
    //osgCols->push_back(osg::Vec4(1,0,0,1));

  }
  setPoints(osgPts,osgCols);

}
void PlotPoints:: setPoints(const vector<btVector3>& pts) {
  osg::ref_ptr<osg::Vec3Array> osgPts(new osg::Vec3Array());
  osg::ref_ptr<osg::Vec4Array>  osgCols(new osg::Vec4Array());
  for (int i = 0; i < pts.size(); i++) {
    btVector3 pt = pts[i];
    osgPts->push_back(osg::Vec3(pt.getX(),pt.getY(),pt.getZ()));
    osgCols->push_back(osg::Vec4(1,0,0,1));
  }
  setPoints(osgPts,osgCols);
}




/*
  int main() {
  Scene s = Scene(false,false,false);


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
