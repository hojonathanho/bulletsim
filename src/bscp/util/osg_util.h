#ifndef osg_util_h
#define osg_util_h

#include <osg/Group> 
#include <osg/Geode>
#include <osg/StateSet>
#include <osg/Math>
#include <osg/Geometry>
#include <osg/MatrixTransform>
#include <osg/Depth>
#include <osg/BlendFunc>

#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
inline void init_transparency_group(osg::Group *group) {

  // Bunch of code for initialization. I don't really know what's going on
  osg::StateSet* stateSet= new osg::StateSet();
  stateSet->setMode( GL_BLEND, osg::StateAttribute::ON );
  stateSet->setRenderingHint( osg::StateSet::TRANSPARENT_BIN );
  stateSet->setMode( GL_DEPTH_TEST, osg::StateAttribute::ON );
  osg::Depth* depth = new osg::Depth;
  depth->setWriteMask( true );
  stateSet->setAttributeAndModes( depth, osg::StateAttribute::ON );
  stateSet->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
  osg::BlendFunc *func = new osg::BlendFunc();
  func->setFunction(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  stateSet->setAttributeAndModes(func);

  group->setStateSet(stateSet);

}

inline osg::MatrixTransform *drawEllipsoid(const Vector3d &mean, const Matrix3d &cov, const Vector4d & color) {

  LLT<Matrix3d> lltofCov(cov);
  Matrix3d L = lltofCov.matrixL();
  Matrix4d t = Matrix4d::Zero();
  t.block(0,0,3,3) = L;
  t.block(0,3,3,1) = mean;
  t(3,3) = 1.0;
  
  osg::Matrix osg_t;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) { 
      osg_t(j,i) = t(i,j); // for some stupid reason 
    }
  }
  osg::MatrixTransform *mt = new osg::MatrixTransform(osg_t);
  osg::Vec4 osg_color(color(0), color(1), color(2), color(3)); 

  osg::Geode *geode = new osg::Geode;
  osg::Sphere *sphere = new osg::Sphere();
  osg::ShapeDrawable *sphere_drawable = new osg::ShapeDrawable(sphere);
  sphere_drawable->setColor(osg_color);
  geode->addDrawable(sphere_drawable);
  mt->addChild(geode);

  return mt;

}

inline osg::MatrixTransform *drawEllipsoid2D(const Vector2d &mean, const double z_offset, const Matrix2d &cov, const Vector4d &color) {
  Vector3d new_mean = Vector3d::Zero();
  new_mean.segment(0,2) = mean;
  new_mean(2) = z_offset; 
  Matrix3d new_cov = Matrix3d::Zero();
  new_cov.block(0,0,2,2) = cov;
  new_cov(2,2) = 1e-7;
  return drawEllipsoid(new_mean, new_cov, color);
}




#endif
