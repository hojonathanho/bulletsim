#include "simulation/environment.h"
#include "simulation/config_viewer.h"
#include "clouds/utils_pcl.h"
#include "clouds/comm_pcl.h"
#include <pcl/io/pcd_io.h>
#include <GL/gl.h>
#include <GL/glx.h>
#include <GL/glu.h>
#include <Eigen/Dense>
#include "fake_kinect.h"
#include "utils/my_assert.h"
using namespace Eigen;

void computeRangeData( osg::Camera* camera , osg::ref_ptr<osg::Image> depthbufferimg, MatrixXf& Ximg, MatrixXf& Yimg, MatrixXf& Zimg) {
  // from https://svn.lcsr.jhu.edu/cisst/trunk/saw/components/sawOpenSceneGraph/code/osaOSGCamera.cpp
  // This is used by glutUnProject

  const osg::Viewport* viewport = camera->getViewport();
  size_t width = viewport->width();
  size_t height = viewport->height();

  GLint view[4];
  view[0] = (int)viewport->x();
  view[1] = (int)viewport->y();
  view[2] = width;
  view[3] = height;
  
  // Create a 3xN range data destination matrix.
  // [ x1 ... xN ]
  // [ y1 ... yN ]
  // [ z1 ... zN ]
  // VCT_COL_MAJOR is used because we will write in the following order
  // x1, y1, z1, x2, y2, z2, ..., xN, yN, zZ

  Ximg.resize(height,width);
  Yimg.resize(height,width);
  Zimg.resize(height,width);


  // get the intrinsic parameters of the camera
  double fovy, aspectRatio, Zn, Zf;
  camera->getProjectionMatrixAsPerspective( fovy, aspectRatio, Zn, Zf );
  
  osg::Matrixd modelm = camera->getViewMatrix();
  osg::Matrixd projm = camera->getProjectionMatrix();

  //for( int y=0; y<height; y++ ){
  for( int y=height-1; 0<=y; y-- ){
    for( size_t x=0; x<width; x++ ){
      GLdouble X, Y, Z;
      float* d = (float*)depthbufferimg->data( x, y );
      gluUnProject( x, y, *d, modelm.ptr(), projm.ptr(), view, &X, &Y, &Z );
      //gluUnProject( x, y, *d, &model[0][0], &proj[0][0], view, &X, &Y, &Z );
      // rangedata is 4xN column major
      Ximg(y,x) = X;
      Yimg(y,x) = Y;
      Zimg(y,x) = Z;
    }
  }

}


ColorCloudPtr computePointCloud( osg::Camera* camera, osg::ref_ptr<osg::Image> depthbufferimg, osg::ref_ptr<osg::Image> colorbufferimg) {
  MatrixXf X,Y,Z;
  computeRangeData(camera, depthbufferimg, X, Y, Z);
  int nRows = X.rows();
  int nCols = X.cols();
  ColorCloudPtr out(new ColorCloud(nCols, nRows));

  for (int row=0; row < nRows; row++) {
    for (int col=0; col < nCols; col++) {
      ColorPoint& pt = out->at(col, row);
      pt.x = X(row,col);
      pt.y = Y(row,col);
      pt.z = Z(row,col);
      pt.r = *(colorbufferimg->data(col,row));
      pt.g = *(colorbufferimg->data(col,row));
      pt.b = *(colorbufferimg->data(col,row));
    }
  }

  return out;
}

MatrixXf computeDepthImage( osg::Camera* camera, osg::ref_ptr<osg::Image> depthbufferimg ) {
  // from https://svn.lcsr.jhu.edu/cisst/trunk/saw/components/sawOpenSceneGraph/code/osaOSGCamera.cpp


  const osg::Viewport* viewport = camera->getViewport();
  size_t width = viewport->width();
  size_t height = viewport->height();
  
  // Create a 3xN range data destination matrix.
  // [ x1 ... xN ]
  // [ y1 ... yN ]
  // [ z1 ... zN ]
  // VCT_COL_MAJOR is used because we will write in the following order
  // x1, y1, z1, x2, y2, z2, ..., xN, yN, zZ
  MatrixXf depthimage( width, height);
  float* Z = depthimage.data();
  float* z = (float*)depthbufferimg->data();

  // get the intrinsic parameters of the camera
  double fovy, aspectRatio, Zn, Zf;
  camera->getProjectionMatrixAsPerspective( fovy, aspectRatio, Zn, Zf );

  size_t i=0;
  // convert zbuffer values [0,1] to depth values + flip the image vertically
  for( int r=height-1; 0<=r; r-- ){
    for( int c=0; c<width; c++ ){
      // forgot where I took this equation
      Z[ i++ ] = Zn*Zf / (Zf - z[ r*width + c ]*(Zf-Zn));
    }
  }
  return depthimage;
}

KinectCallback::KinectCallback( osg::Camera* camera ) {
  const osg::Viewport* viewport    = camera->getViewport();
  osg::Viewport::value_type width  = viewport->width();
  osg::Viewport::value_type height = viewport->height();
  cout << "viewport w/h: " << width << " " << height << endl;
  depthbufferimg = new osg::Image;
  depthbufferimg->allocateImage(width, height, 1, GL_DEPTH_COMPONENT, GL_FLOAT);
  colorbufferimg = new osg::Image;
  colorbufferimg->allocateImage(width, height, 1, GL_RGBA, GL_UNSIGNED_BYTE);
  camera->attach(osg::Camera::DEPTH_BUFFER, depthbufferimg.get(), 0, 0);
  camera->attach(osg::Camera::COLOR_BUFFER, colorbufferimg.get(), 0, 0); 
  m_done = false;
}

void KinectCallback::operator () ( osg::RenderInfo& info ) const  {
  if (!m_done) {
    osg::Camera* camera = info.getCurrentCamera();
    double start = timeOfDay();
    ColorCloudPtr cloud = computePointCloud(camera, depthbufferimg.get(), colorbufferimg.get());
    cout << timeOfDay() - start << endl;

    KinectCallback* this2 = const_cast<KinectCallback*>(this);
    this2->m_done = true;
    this2->m_cloud = cloud;

  }
}

#include "simulation/util.h"
//#include <osgGA/TrackballManipulator>
osg::Vec3f toOSGVector(Eigen::Vector3f v) { return osg::Vec3f(v.x(), v.y(), v.z());}

FakeKinect::FakeKinect(OSGInstance::Ptr osg, Affine3f worldFromCam, bool usePub) {
  if (usePub) m_pub.reset(new FilePublisher("kinect", "pcd"));
  m_viewer.setUpViewInWindow(0, 0, 640, 480);
  m_viewer.setSceneData(osg->root.get());
  m_viewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);
  m_cam = m_viewer.getCamera();

  m_cam->setProjectionMatrixAsPerspective(49,640./480., .2*METERS, 10*METERS);

  Vector3f eye, center, up;
  Matrix3f rot;
  rot = worldFromCam.linear();
  eye = worldFromCam.translation();
  center = rot.col(2);
  up = -rot.col(1);
  m_cam->setViewMatrixAsLookAt(toOSGVector(eye), toOSGVector(center+eye), toOSGVector(up));

  m_cb = new KinectCallback(m_cam.get());
  m_cam->setFinalDrawCallback(m_cb);


}

void FakeKinect::sendMessage() {
  if (!m_pub) return;
  m_cb->m_done = false;
  m_viewer.frame();
  ENSURE(m_cb->m_cloud != NULL);
  CloudMessage msg(m_cb->m_cloud);
  m_pub->send(msg);
}

ColorCloudPtr FakeKinect::snapshot() {
  m_cb->m_done = false;
  m_viewer.frame();
  ENSURE(m_cb->m_cloud != NULL);
  return m_cb->m_cloud;
}
