#include "plotting_tracking.h"
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include "simulation/util.h"
#include "utils/conversions.h"
using namespace std;
using namespace Eigen;

inline bool pointIsFinite(const ColorPoint& pt) {
  return isfinite(pt.x) && isfinite(pt.y) && isfinite(pt.z);
}

void PointCloudPlot::setPoints1(const pcl::PointCloud<ColorPoint>::Ptr& cloud, float alpha) {
  osg::ref_ptr<osg::Vec3Array> osgPts = new osg::Vec3Array();
  osg::ref_ptr<osg::Vec4Array>  osgCols = new osg::Vec4Array();
  osgPts->reserve(cloud->size());
  osgCols->reserve(cloud->size());
  BOOST_FOREACH(const ColorPoint& pt, cloud->points){
    if (pointIsFinite(pt)) {
      osgPts->push_back(osg::Vec3(pt.x,pt.y,pt.z));
      osgCols->push_back(osg::Vec4(pt.r/255.,pt.g/255.,pt.b/255.,alpha));
    }
  }
  PlotPoints::setPoints(osgPts,osgCols);
}

void drawCorrLines(PlotLines::Ptr lines, const vector<btVector3>& aPts, const vector<btVector3>& bPts, const SparseMatrixf& corr, int aPtInd) {
  vector<btVector3> linePoints;
  vector<btVector4> lineColors;
  for (int iRow=0; iRow<corr.rows(); ++iRow)
    for (SparseMatrixf::InnerIterator it(corr,iRow); it; ++it) {
      if (aPtInd<0 || aPtInd==it.row()) {
				linePoints.push_back(aPts[it.row()]);
				linePoints.push_back(bPts[it.col()]);
				lineColors.push_back(btVector4(it.value(),it.value(),0,1));
      }
  }
//	btVector3 cam_pos(-1.68829, -0.149108, 8.07206);
//  for(int i=0; i<aPts.size(); i++) {
//		btVector3 point_to_cam = (cam_pos-aPts[i]).normalized();
//  	linePoints.push_back(aPts[i]+0.01*point_to_cam);
//  	linePoints.push_back(cam_pos);
//  }
  lines->setPoints(linePoints, lineColors);
}

// draw all the lines between aPts and Pts that have a corr>threshold.
// if aPtInd is in the range of aPts, then draw only the lines that comes from aPts[aPtInd]
void drawCorrLines(PlotLines::Ptr lines, const vector<btVector3>& aPts, const vector<btVector3>& bPts, const Eigen::MatrixXf& corr, float threshold, int aPtInd) {
  vector<btVector3> linePoints;
  vector<btVector4> lineColors;
  float max_corr = corr.maxCoeff(); // color lines by corr, where corr has been mapped [threshold,max_corr] -> [0,1]
  for (int i=0; i<corr.rows(); ++i)
    for (int j=0; j<corr.cols(); ++j)
    	if (corr(i,j) > threshold) {
    		if (aPtInd<0 || aPtInd>=corr.rows() || aPtInd==i) {
					linePoints.push_back(aPts[i]);
					linePoints.push_back(bPts[j]);
					float color_factor = (corr(i,j)-threshold)/(max_corr-threshold); //basically, it ranges from 0 to 1
					lineColors.push_back(btVector4(color_factor, color_factor,0,1));
				}
    	}
  lines->setPoints(linePoints, lineColors);
}

void plotNodesAsSpheres(btSoftBody* psb, const VectorXf& pVis, const Eigen::VectorXf& stdev, PlotSpheres::Ptr spheres) {
  int nPts = pVis.rows();
  using namespace osg;
  ref_ptr<Vec3Array> centers = new Vec3Array();
  ref_ptr<Vec4Array> colors = new Vec4Array();
  //vector<float> sizes = toVec(stdev.array().sqrt());
  vector<float> sizes = toVec(stdev/4.0);
  for (int i=0; i<nPts; i++) {
    const btVector3& v = psb->m_nodes[i].m_x;
    float p = pVis[i];
    centers->push_back(Vec3f(v.x(), v.y(), v.z()));
    colors->push_back(Vec4f(p,p,p,.25));
  }
  spheres->plot(centers, colors, sizes);
}

void plotNodesAsSpheres(const vector<btVector3>& nodes, const VectorXf& pVis, const Eigen::VectorXf& stdev, PlotSpheres::Ptr spheres) {
  int nPts = pVis.rows();
  using namespace osg;
  ref_ptr<Vec3Array> centers = new Vec3Array();
  ref_ptr<Vec4Array> colors = new Vec4Array();
  //vector<float> sizes = toVec(stdev.array().sqrt());
  vector<float> sizes = toVec(stdev/4.0);
  for (int i=0; i<nPts; i++) {
    float p = pVis[i];
    centers->push_back(Vec3f(nodes[i].x(), nodes[i].y(), nodes[i].z()));
    colors->push_back(Vec4f(p,p,p,.25));
  }
  spheres->plot(centers, colors, sizes);
}

void plotNodesAsSpheres(const Eigen::MatrixXf nodes, const Eigen::VectorXf& pVis, const Eigen::MatrixXf& stdev, PlotSpheres::Ptr spheres) {
	assert(nodes.rows() == pVis.rows());
	assert(nodes.rows() == stdev.rows());
	assert(nodes.cols() == stdev.cols());
	MatrixXf centers = nodes.leftCols(3);
	MatrixXf colors(nodes.rows(), 4);
	if (nodes.cols() >= 6)
		//colors << nodes.middleCols(3,3).rowwise().reverse(), 0.25*VectorXf::Ones(nodes.rows());
		colors << nodes.middleCols(3,3).rowwise().reverse(), pVis;
		//colors << nodes.middleCols(3,3).rowwise().reverse(), nodes.col(6);
	else
		colors = Vector4f(1,1,1,1).transpose().replicate(nodes.rows(), 1);
	VectorXf sizes = (stdev.leftCols(3)/4.0).rowwise().mean();
	spheres->plot(util::toVec3Array(centers), util::toVec4Array(colors), toVec(sizes));
}

void plotNodesAsSpheres(const Eigen::MatrixXf centers, const Eigen::MatrixXf colors, const Eigen::VectorXf& pVis, const Eigen::MatrixXf& stdev, PlotSpheres::Ptr spheres) {
	assert(centers.rows() == colors.rows());
	MatrixXf nodes(centers.rows(), centers.cols() + colors.cols());
	nodes.leftCols(centers.cols()) = centers;
	nodes.rightCols(colors.cols()) = colors;
	plotNodesAsSpheres(nodes, pVis, stdev, spheres);
}

void plotObs(const vector<btVector3>& obsPts, const Eigen::VectorXf& inlierFrac, PointCloudPlot::Ptr plot) {
  osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
  for (int i=0; i < inlierFrac.size(); i++) {
    //float p = 1.0;
  	float p = inlierFrac(i);
    colors->push_back(osg::Vec4f(0,p,0,.5));
  }
  plot->setPoints(util::toVec3Array(obsPts), colors);
}

void plotObsBorder(const Eigen::MatrixXf cloud, PointCloudPlot::Ptr plot) {
	osg::ref_ptr<osg::Vec3Array> centers = new osg::Vec3Array;
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	for (int i=0; i < cloud.rows(); i++) {
		if (cloud(i,6)) {
			centers->push_back(osg::Vec3f(cloud(i,0), cloud(i,1), cloud(i,2)));
			colors->push_back(osg::Vec4f(cloud(i,5), cloud(i,4), cloud(i,3), 0.5));
		}
	}
	plot->setPoints(centers, colors);
}

void plotObs(const Eigen::MatrixXf cloud, PointCloudPlot::Ptr plot) {
	MatrixXf centers = cloud.leftCols(3);
	MatrixXf colors(cloud.rows(), 4);
	if (cloud.cols() >= 6)
		colors << cloud.middleCols(3,3).rowwise().reverse(), 0.5*VectorXf::Ones(cloud.rows());
	else
		colors = Vector4f(1,1,1,1).transpose().replicate(cloud.rows(), 1);
	plot->setPoints(util::toVec3Array(centers), util::toVec4Array(colors));
}

void plotObs(const Eigen::MatrixXf centers, const Eigen::MatrixXf colors, PointCloudPlot::Ptr plot) {
	assert(centers.rows() == colors.rows());
	MatrixXf nodes(centers.rows(), centers.cols() + colors.cols());
	nodes.leftCols(centers.cols()) = centers;
	nodes.rightCols(colors.cols()) = colors;
	plotObs(nodes, plot);
}
