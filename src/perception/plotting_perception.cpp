#include "plotting_perception.h"
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

inline bool pointIsFinite(const pcl::PointXYZRGB& pt) {
  return isfinite(pt.x) && isfinite(pt.y) && isfinite(pt.z);
}

void PointCloudPlot::setPoints1(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
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
  PlotPoints::setPoints(osgPts,osgCols);
}
