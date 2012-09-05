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
#include <osg/ShapeDrawable>
#include <osg/CullFace>
#include <osg/Camera>

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
  stateSet->setAttributeAndModes(new osg::CullFace(osg::CullFace::FRONT));
  stateSet->setAttributeAndModes(new osg::CullFace(osg::CullFace::BACK));
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

// Given a Camera, create a wireframe representation of its
// view frustum. Create a default representation if camera==NULL.
osg::Node*
makeFrustumFromCamera( osg::Camera* camera, double size= 0.1 )
{
    // Projection and ModelView matrices
    osg::Matrixd proj;
    osg::Matrixd mv;
    if (camera)
    {
        proj = camera->getProjectionMatrix();
        mv = camera->getViewMatrix();
    }
    else
    {
        // Create some kind of reasonable default Projection matrix.
        proj.makePerspective( 30., 1., 1., 10. );
        // leave mv as identity
    }

    // Get near and far from the Projection matrix.
//    const double near = 0.1* proj(3,2) / (proj(2,2)-1.0);
    const double near = size;
    const double far = size;
    //const double far =  0.1*proj(3,2) / (1.0+proj(2,2));

    // Get the sides of the near plane.
    const double nLeft = near * (proj(2,0)-1.0) / proj(0,0);
    const double nRight = near * (1.0+proj(2,0)) / proj(0,0);
    const double nTop = near * (1.0+proj(2,1)) / proj(1,1);
    const double nBottom = near * (proj(2,1)-1.0) / proj(1,1);

    // Get the sides of the far plane.
    const double fLeft = far * (proj(2,0)-1.0) / proj(0,0);
    const double fRight = far * (1.0+proj(2,0)) / proj(0,0);
    const double fTop = far * (1.0+proj(2,1)) / proj(1,1);
    const double fBottom = far * (proj(2,1)-1.0) / proj(1,1);

    // Our vertex array needs only 9 vertices: The origin, and the
    // eight corners of the near and far planes.
    osg::Vec3Array* v = new osg::Vec3Array;
    v->resize( 9 );
    (*v)[0].set( 0., 0., 0. );
    (*v)[1].set( nLeft, nBottom, -near );
    (*v)[2].set( nRight, nBottom, -near );
    (*v)[3].set( nRight, nTop, -near );
    (*v)[4].set( nLeft, nTop, -near );
    (*v)[5].set( fLeft, fBottom, -far );
    (*v)[6].set( fRight, fBottom, -far );
    (*v)[7].set( fRight, fTop, -far );
    (*v)[8].set( fLeft, fTop, -far );

    osg::Geometry* geom = new osg::Geometry;
    geom->setUseDisplayList( false );
    geom->setVertexArray( v );

    osg::Vec4Array* c = new osg::Vec4Array;
    c->push_back( osg::Vec4( 1., 1., 1., 1. ) );
    geom->setColorArray( c );
    geom->setColorBinding( osg::Geometry::BIND_OVERALL );

    GLushort idxLines[8] = {
        0, 5, 0, 6, 0, 7, 0, 8 };
    GLushort idxLoops0[4] = {
        1, 2, 3, 4 };
    GLushort idxLoops1[4] = {
        5, 6, 7, 8 };
    geom->addPrimitiveSet( new osg::DrawElementsUShort( osg::PrimitiveSet::LINES, 8, idxLines ) );
    //geom->addPrimitiveSet( new osg::DrawElementsUShort( osg::PrimitiveSet::LINE_LOOP, 4, idxLoops0 ) );
    geom->addPrimitiveSet( new osg::DrawElementsUShort( osg::PrimitiveSet::LINE_LOOP, 4, idxLoops1 ) );

    osg::Geode* geode = new osg::Geode;
    geode->addDrawable( geom );

    geode->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );


    // Create parent MatrixTransform to transform the view volume by
    // the inverse ModelView matrix.
    osg::MatrixTransform* mt = new osg::MatrixTransform;
    mt->setMatrix( osg::Matrixd::inverse( mv ) );
    mt->addChild( geode );

    return mt;
}






#endif
