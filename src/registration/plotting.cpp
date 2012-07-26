//#include "plotting.h"
//#include "util.h"
//#include <osg/PointSprite>
//#include <osg/Point>
//#include <osg/LineWidth>
//#include <osg/Geometry>
//#include <osg/StateSet>
//#include <osg/BlendFunc>
//#include <osg/ShapeDrawable>
//#include <boost/foreach.hpp>
//
//using namespace std;
//using namespace util;
//
//// based on galaxy example in osg docs
//
//void PlotObject::setDefaultColor(float r, float g, float b, float a) {
//  m_defaultColor = osg::Vec4(r,g,b,a);
//}
//
/*
 */
//
#include "plotting.h"
#include <boost/foreach.hpp>
#include <osg/Point>
#include <osg/BlendFunc>

PlotPoints::PlotPoints(float size) :
	m_geom(new osg::Geometry()), m_defaultColor(1, 1, 1, 1) {

	m_geom->setDataVariance(osg::Object::DYNAMIC);
	addDrawable(m_geom);

	osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet();
	osg::Point *point = new osg::Point();

	//  m_stateset->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
	stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

	osg::ref_ptr<osg::BlendFunc> blendFunc = new osg::BlendFunc;
	blendFunc->setFunction(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	stateset->setAttributeAndModes(blendFunc);
	stateset->setMode(GL_BLEND, osg::StateAttribute::ON);

	point->setSize(size);
	stateset->setAttribute(point);

	setStateSet(stateset);
}

void PlotPoints::setPoints(const ColorCloudPtr& cloud) {
	osg::ref_ptr<osg::Vec3Array> osgPts = new osg::Vec3Array();
	osgPts->reserve(cloud->size());
	osg::ref_ptr<osg::Vec4Array> osgCols = new osg::Vec4Array();
	osgCols->reserve(cloud->size());
	BOOST_FOREACH(const ColorPoint& pt, cloud->points)
{	if (std::isfinite(pt.x)) {
		osgPts->push_back(osg::Vec3(pt.x,pt.y,pt.z));
		osgCols->push_back(osg::Vec4(pt.r/255.,pt.g/255.,pt.b/255.,1));
	}

	//
}
}
void PlotPoints::setPoints(const osg::ref_ptr<osg::Vec3Array>& osgPts,
		const osg::ref_ptr<osg::Vec4Array>& osgCols) {
	int nPts = osgPts->getNumElements();
	m_geom->setVertexArray(osgPts);
	m_geom->setColorArray(osgCols);
	m_geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	m_geom->getPrimitiveSetList().clear();
	m_geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0,
			nPts));
}

void PlotPoints::clear() {
	m_geom->getPrimitiveSetList().clear();
	osg::ref_ptr<osg::Vec3Array> osgPts = new osg::Vec3Array;
	osg::ref_ptr<osg::Vec4Array> osgCols = new osg::Vec4Array;
	m_geom->setVertexArray(osgPts);
	m_geom->setColorArray(osgCols);
}

//
//void PlotPoints::setPoints(const osg::ref_ptr<osg::Vec3Array>& osgPts) {
//  osg::ref_ptr<osg::Vec4Array> osgCols = new osg::Vec4Array(osgPts->size());
//  BOOST_FOREACH(osg::Vec4& col, *osgCols) col = m_defaultColor;
//  setPoints(osgPts, osgCols);
//}
//
//void PlotPoints::forceTransparency(float a) {
//  if (!m_geom->getColorArray()) return;
//  osg::Vec4Array &colors = (osg::Vec4Array&) *m_geom->getColorArray();
//  for (int i = 0; i < colors.size(); ++i) {
//    osg::Vec4 c = colors[i];
//    colors[i] = osg::Vec4(c.r(), c.g(), c.b(), a);
//  }
//}
//
//void PlotPoints::setPoints(const vector<btVector3>& pts, const vector<btVector4>& cols) {
//  setPoints(toVec3Array(pts), toVec4Array(cols));
//}
//void PlotPoints::setPoints(const vector<btVector3>& pts) {
//  setPoints(toVec3Array(pts));
//}
//
//PlotLines::PlotLines(float width) {
//  setDefaultColor(1,1,1,1);
//
//  m_geode = new osg::Geode();
//  m_geom = new osg::Geometry();
//  m_geom->setDataVariance(osg::Object::DYNAMIC);
//  m_geode->addDrawable(m_geom);
//
//  osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet();
//  osg::LineWidth *linewidth = new osg::LineWidth();
//  linewidth->setWidth(width);
//  stateset->setAttributeAndModes(linewidth,osg::StateAttribute::ON);
//  stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
//  stateset->setAttribute(linewidth);
//
//  osg::ref_ptr<osg::BlendFunc> blendFunc = new osg::BlendFunc;
//  blendFunc->setFunction(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//  stateset->setAttributeAndModes(blendFunc);
//  stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
//
//  m_geode->setStateSet(stateset);
//}
//
//void PlotLines::setPoints(const vector<btVector3>& pts, const vector<btVector4>& cols) {
//  setPoints(toVec3Array(pts),  toVec4Array(cols));
//}
//
//void PlotLines::setPoints(const vector<btVector3>& pts) {
//  osg::ref_ptr<osg::Vec4Array> osgCols = new osg::Vec4Array(pts.size());
//  BOOST_FOREACH(osg::Vec4& col, *osgCols) col = m_defaultColor;
//  setPoints(toVec3Array(pts),  osgCols);
//}
//
//
//void PlotLines::setPoints(const osg::ref_ptr<osg::Vec3Array>& osgPts, const osg::ref_ptr<osg::Vec4Array>& osgCols) {
//  int nPts = osgPts->getNumElements();
//  m_geom->setColorArray(osgCols);
//  m_geom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE);
//  m_geom->setVertexArray(osgPts);
//  m_geom->getPrimitiveSetList().clear();
//  m_geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,nPts));
//}
//
//void PlotLines::setPoints(const osg::ref_ptr<osg::Vec3Array>& osgPts) {
//  osg::ref_ptr<osg::Vec4Array> osgCols = new osg::Vec4Array(osgPts->size());
//  BOOST_FOREACH(osg::Vec4& col, *osgCols) col = m_defaultColor;
//  setPoints(osgPts, osgCols);
//}
//
//void PlotLines::forceTransparency(float a) {
//  if (!m_geom->getColorArray()) return;
//  osg::Vec4Array &colors = (osg::Vec4Array&) *m_geom->getColorArray();
//  for (int i = 0; i < colors.size(); ++i) {
//    osg::Vec4 c = colors[i];
//    colors[i] = osg::Vec4(c.r(), c.g(), c.b(), a);
//  }
//}
//
//PlotSpheres::PlotSpheres() {
//  m_geode = new osg::Geode();
//  m_nDrawables = 0;
//
//  osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet();
//  //stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
//  osg::ref_ptr<osg::BlendFunc> blendFunc = new osg::BlendFunc;
//  blendFunc->setFunction(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//  stateset->setAttributeAndModes(blendFunc);
//  stateset->setMode(GL_BLEND, osg::StateAttribute::ON);
//
//  m_geode->setStateSet(stateset);
//
//};
//
//void PlotSpheres::clear() {
//  m_geode->removeDrawables(0,m_nDrawables);
//}
//
//void PlotSpheres::plot(const osg::ref_ptr<osg::Vec3Array>& centers, const osg::ref_ptr<osg::Vec4Array>& cols, const vector<float>& radii) {
//  m_geode->removeDrawables(0,m_nDrawables);
//  m_nDrawables = centers->size();
//  for (int i=0; i < centers->size(); i++) {
//    osg::TessellationHints* hints = new osg::TessellationHints;
//    hints->setDetailRatio(0.25f);
//    osg::Sphere* sphere = new osg::Sphere( centers->at(i), radii.at(i));
//    osg::ShapeDrawable* sphereDrawable = new osg::ShapeDrawable(sphere,hints);
//    sphereDrawable->setColor(cols->at(i));
//    m_geode->addDrawable(sphereDrawable);
//  }
//}
//
//PlotBoxes::PlotBoxes() {
//  m_geode = new osg::Geode();
//  osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet();
//  osg::ref_ptr<osg::BlendFunc> blendFunc = new osg::BlendFunc;
//  blendFunc->setFunction(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//  stateset->setAttributeAndModes(blendFunc);
//  stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
//  m_geode->setStateSet(stateset);
//}
//
//void PlotBoxes::clear() {
//  while (m_geode->removeDrawables(0, 1)) ;
//}
//
//void PlotBoxes::addBox(const osg::Vec3 &center, float lenx, float leny, float lenz, const osg::Vec4 &color) {
//  osg::Box *box = new osg::Box(center, lenx, leny, lenz);
//  osg::ShapeDrawable *boxDrawable = new osg::ShapeDrawable(box);
//  boxDrawable->setColor(color);
//  m_geode->addDrawable(boxDrawable);
//}
//
//void PlotAxes::setup(const btTransform &tf, float size) {
//  btMatrix3x3 mat(tf.getRotation());
//  osg::Vec3f origin = util::toOSGVector(tf.getOrigin());
//  osg::Vec3f x = util::toOSGVector(mat.getColumn(0));
//  osg::Vec3f y = util::toOSGVector(mat.getColumn(1));
//  osg::Vec3f z = util::toOSGVector(mat.getColumn(2));
//  setup(origin, x, y, z, size);
//}
//
//PlotAxes::PlotAxes(osg::Vec3f origin, osg::Vec3f x, osg::Vec3f y, osg::Vec3f z, float size) {
//  setup(origin, x, y, z, size);
//
//
//  osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet();
//  osg::ref_ptr<osg::BlendFunc> blendFunc = new osg::BlendFunc;
//  blendFunc->setFunction(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//  stateset->setAttributeAndModes(blendFunc);
//  stateset->setMode(GL_BLEND, osg::StateAttribute::ON);
//
//}
//
//void PlotAxes::setup(osg::Vec3f origin, osg::Vec3f x, osg::Vec3f y, osg::Vec3f z, float size) {
//
//  osg::ref_ptr<osg::Vec4Array> cols = new osg::Vec4Array();
//  osg::ref_ptr<osg::Vec3Array> pts = new osg::Vec3Array();
//
//  pts->push_back(origin);
//  pts->push_back(origin+x*(size/x.length()));
//  pts->push_back(origin);
//  pts->push_back(origin+y*(size/y.length()));
//  pts->push_back(origin);
//  pts->push_back(origin+z*(size/z.length()));
//
//  cols->push_back(osg::Vec4f(1,0,0,1));
//  cols->push_back(osg::Vec4f(0,1,0,1));
//  cols->push_back(osg::Vec4f(0,0,1,1));
//
//  PlotLines::setPoints(pts, cols);
//
//  osg::ref_ptr<osg::Vec3Array> endpts = new osg::Vec3Array();
//  endpts->push_back(origin+x*(size/x.length()));
//  endpts->push_back(origin+y*(size/y.length()));
//  endpts->push_back(origin+z*(size/z.length()));
//  vector<float> radii(3,.1*size);
//  m_ends->plot(endpts, cols, radii);
//}
//
//
//PlotCurve::PlotCurve(float width) : osg::Geode(), m_defaultColor(1,0,0,1) {
//
//  m_geom = new osg::Geometry();
//  addDrawable(m_geom);
//
//  osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet();
//  osg::LineWidth *linewidth = new osg::LineWidth();
//  linewidth->setWidth(width);
//  stateset->setAttributeAndModes(linewidth,osg::StateAttribute::ON);
//  stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
//  stateset->setAttribute(linewidth);
//
//  osg::ref_ptr<osg::BlendFunc> blendFunc = new osg::BlendFunc;
//  blendFunc->setFunction(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//  stateset->setAttributeAndModes(blendFunc);
//  stateset->setMode(GL_BLEND, osg::StateAttribute::ON);
//
//  setStateSet(stateset);
//}
//
//void PlotCurve::setPoints(const vector<btVector3>& pts, const vector<btVector4>& cols) {
//  setPoints(toVec3Array(pts),  toVec4Array(cols));
//}
//
//void PlotCurve::setPoints(const vector<btVector3>& pts) {
//  osg::ref_ptr<osg::Vec4Array> osgCols = new osg::Vec4Array(pts.size());
//  BOOST_FOREACH(osg::Vec4& col, *osgCols) col = m_defaultColor;
//  setPoints(toVec3Array(pts),  osgCols);
//}
//
//void PlotCurve::setPoints(const osg::ref_ptr<osg::Vec3Array>& osgPts, const osg::ref_ptr<osg::Vec4Array>& osgCols) {
//  int nPts = osgPts->getNumElements();
//  m_geom->setColorArray(osgCols);
//  m_geom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE);
//  m_geom->setVertexArray(osgPts);
//  m_geom->getPrimitiveSetList().clear();
//  m_geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP,0,nPts));
//}
//
//
//
//void PointCloudPlot::setPoints1(const pcl::PointCloud<ColorPoint>::Ptr& cloud, float alpha) {
//  osg::ref_ptr<osg::Vec3Array> osgPts = new osg::Vec3Array();
//  osg::ref_ptr<osg::Vec4Array>  osgCols = new osg::Vec4Array();
//  osgPts->reserve(cloud->size());
//  osgCols->reserve(cloud->size());
//  BOOST_FOREACH(const ColorPoint& pt, cloud->points){
//    if (pointIsFinite(pt)) {
//      osgPts->push_back(osg::Vec3(pt.x,pt.y,pt.z));
//      osgCols->push_back(osg::Vec4(pt.r/255.,pt.g/255.,pt.b/255.,alpha));
//    }
//  }
//  PlotPoints::setPoints(osgPts,osgCols);
//}
//
//void drawCorrLines(PlotLines::Ptr lines, const vector<btVector3>& aPts, const vector<btVector3>& bPts, const SparseMatrixf& corr) {
//  vector<btVector3> linePoints;
//  for (int iRow=0; iRow<corr.rows(); ++iRow)
//    for (SparseMatrixf::InnerIterator it(corr,iRow); it; ++it) {
//      linePoints.push_back(aPts[it.row()]);
//      linePoints.push_back(bPts[it.col()]);
//  }
//  lines->setPoints(linePoints);
//}
//
//void plotNodesAsSpheres(btSoftBody* psb, const VectorXf& pVis, const Eigen::VectorXf& sigs, PlotSpheres::Ptr spheres) {
//  int nPts = pVis.rows();
//  using namespace osg;
//  ref_ptr<Vec3Array> centers = new Vec3Array();
//  ref_ptr<Vec4Array> colors = new Vec4Array();
//  //vector<float> sizes = toVec(sigs.array().sqrt());
//  vector<float> sizes = toVec(sigs.array().sqrt());
//  for (int i=0; i<nPts; i++) {
//    const btVector3& v = psb->m_nodes[i].m_x;
//    float p = pVis[i];
//    centers->push_back(Vec3f(v.x(), v.y(), v.z()));
//    colors->push_back(Vec4f(p,p,p,.25));
//  }
//  spheres->plot(centers, colors, sizes);
//}
//
//void plotNodesAsSpheres(const vector<btVector3>& nodes, const VectorXf& pVis, const Eigen::VectorXf& sigs, PlotSpheres::Ptr spheres) {
//  int nPts = pVis.rows();
//  using namespace osg;
//  ref_ptr<Vec3Array> centers = new Vec3Array();
//  ref_ptr<Vec4Array> colors = new Vec4Array();
//  //vector<float> sizes = toVec(sigs.array().sqrt());
//  vector<float> sizes = toVec(sigs.array().sqrt()/2);
//  for (int i=0; i<nPts; i++) {
//    float p = pVis[i];
//    centers->push_back(Vec3f(nodes[i].x(), nodes[i].y(), nodes[i].z()));
//    colors->push_back(Vec4f(p,p,p,.25));
//  }
//  spheres->plot(centers, colors, sizes);
//}
//
//
//void plotObs(const vector<btVector3>& obsPts, const Eigen::VectorXf& inlierFrac, PointCloudPlot::Ptr plot) {
//  osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
//  for (int i=0; i < inlierFrac.size(); i++) {
//    float p = inlierFrac(i);
//    colors->push_back(osg::Vec4f(0,p,0,.5));
//  }
//  plot->setPoints(util::toVec3Array(obsPts), colors);
//}
