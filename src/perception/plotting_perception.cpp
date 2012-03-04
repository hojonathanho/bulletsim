#include "plotting_perception.h"
#include "utils_perception.h"
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include "simulation/util.h"
using namespace Eigen;

inline bool pointIsFinite(const pcl::PointXYZRGBA& pt) {
  return isfinite(pt.x) && isfinite(pt.y) && isfinite(pt.z);
}

void PointCloudPlot::setPoints1(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud, float alpha) {
  osg::ref_ptr<osg::Vec3Array> osgPts = new osg::Vec3Array();
  osg::ref_ptr<osg::Vec4Array>  osgCols = new osg::Vec4Array();
  osgPts->reserve(cloud->size());
  osgCols->reserve(cloud->size());
  BOOST_FOREACH(const pcl::PointXYZRGBA& pt, cloud->points){
    if (pointIsFinite(pt)) {
      osgPts->push_back(osg::Vec3(pt.x,pt.y,pt.z));
      osgCols->push_back(osg::Vec4(pt.r/255.,pt.g/255.,pt.b/255.,alpha));
    }
  }
  PlotPoints::setPoints(osgPts,osgCols);
}

void drawCorrLines(PlotLines::Ptr lines, const vector<btVector3>& aPts, const vector<btVector3>& bPts, const SparseArray& corr) {
  vector<btVector3> linePoints;
  for (int iA=0; iA < aPts.size(); iA++) {
    BOOST_FOREACH(const IndVal& iv, corr[iA]) {
      linePoints.push_back(aPts[iA]);
      linePoints.push_back(bPts[iv.ind]);
    }
  }
  lines->setPoints(linePoints);
}

void plotNodesAsSpheres(btSoftBody* psb, const VectorXf& pVis, const Eigen::VectorXf& sigs, PlotSpheres::Ptr spheres) {
  int nPts = pVis.rows();
  using namespace osg;
  ref_ptr<Vec3Array> centers = new Vec3Array();
  ref_ptr<Vec4Array> colors = new Vec4Array();
  //vector<float> sizes = toVec(sigs.array().sqrt());
  vector<float> sizes = toVec(sigs.array().sqrt());
  for (int i=0; i<nPts; i++) {
    const btVector3& v = psb->m_nodes[i].m_x;
    float p = pVis[i];
    centers->push_back(Vec3f(v.x(), v.y(), v.z()));
    colors->push_back(Vec4f(p,p,p,.25));
  }
  spheres->plot(centers, colors, sizes);
}

void plotNodesAsSpheres(const vector<btVector3>& nodes, const VectorXf& pVis, const Eigen::VectorXf& sigs, PlotSpheres::Ptr spheres) {
  int nPts = pVis.rows();
  using namespace osg;
  ref_ptr<Vec3Array> centers = new Vec3Array();
  ref_ptr<Vec4Array> colors = new Vec4Array();
  //vector<float> sizes = toVec(sigs.array().sqrt());
  vector<float> sizes = toVec(sigs.array().sqrt()/2);
  for (int i=0; i<nPts; i++) {
    float p = pVis[i];
    centers->push_back(Vec3f(nodes[i].x(), nodes[i].y(), nodes[i].z()));
    colors->push_back(Vec4f(p,p,p,.25));
  }
  spheres->plot(centers, colors, sizes);
}


void plotObs(const vector<btVector3>& obsPts, const Eigen::VectorXf& inlierFrac, PointCloudPlot::Ptr plot) {
  osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
  for (int i=0; i < inlierFrac.size(); i++) {
    float p = inlierFrac(i);
    colors->push_back(osg::Vec4f(0,p,0,.5));
  }
  plot->setPoints(util::toVec3Array(obsPts), colors);
}
