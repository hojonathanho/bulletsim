#include "simulation/simplescene.h"
#include "simulation/config_viewer.h"
#include "robots/pr2.h"
#include <osgDB/WriteFile>
#include "utils/vector_io.h"
#include "clouds/utils_pcl.h"
#include <Eigen/Dense>
#include<GL/gl.h>
#include<GL/glx.h>
#include<GL/glu.h>
#include <ostream>
#include <pcl/io/pcd_io.h>
using namespace Eigen;
using namespace std;
const float CX = 319.5;
const float CY = 119.5;
const float F = 525;

bool done = false;

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
      pcl::PointXYZRGBA& pt = out->at(col, row);
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
  cout << "computing depth image" << endl;
  // from https://svn.lcsr.jhu.edu/cisst/trunk/saw/components/sawOpenSceneGraph/code/osaOSGCamera.cpp
  // This is used by glutUnProject

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


class FinalDrawCallback : public osg::Camera::DrawCallback {
    
public:

  int width;
  int height;

  bool m_done;

  osg::ref_ptr<osg::Image> depthbufferimg;
  osg::ref_ptr<osg::Image> colorbufferimg;

    
  FinalDrawCallback( osg::Camera* camera ) {
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
    done = false;
  }

  void operator () ( osg::RenderInfo& info ) const  {
    if (!m_done) {
      osg::Camera* camera = info.getCurrentCamera();
      FinalDrawCallback* this2 = const_cast<FinalDrawCallback*>(this);



      // MatrixXf rangeData = computeDepthImage(camera, depthbufferimg);
      // ofstream outfile("depth.txt");
      // outfile << rangeData << endl;
      // outfile.close();
      ColorCloudPtr cloud = computePointCloud(camera, depthbufferimg.get(), colorbufferimg.get());
      pcl::io::savePCDFileBinary("q.pcd", *cloud);

      this2->m_done = true;
      cout << "writing depth image" << endl;

    }
  }
    
}; // FinalDrawCallback


int main(int argc, char *argv[]) {
  // first read the configuration from the user

  // and override config values to what we want
  SceneConfig::enableIK = false;
  SceneConfig::enableRobot = true;
  SceneConfig::enableHaptics = false;

  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.addGroup(SceneConfig());
  parser.read(argc, argv);

  parser.read(argc, argv);    
    
  // construct the scene
  Scene scene;
  // manipulate the scene or add more objects, if desired

  PR2Manager pr2m(scene);

  // start the simulation

  osgViewer::Viewer viewer2;
  viewer2.setUpViewInWindow(0, 0, 640, 480);
  osg::ref_ptr<osgGA::TrackballManipulator> manip = new osgGA::TrackballManipulator();
  manip->setHomePosition(util::toOSGVector(ViewerConfig::cameraHomePosition), osg::Vec3(), osg::Z_AXIS);
  viewer2.setCameraManipulator(manip);
  viewer2.setSceneData(scene.osg->root.get());
  // viewer2.realize();


  scene.startViewer();

  osg::ref_ptr<osg::Camera> osgCam = viewer2.getCamera(); 

  // colorImage->allocateImage(viewport->width(), viewport->height(), 1, GL_RGBA, GL_UNSIGNED_BYTE); 
  // zImage->allocateImage(viewport->width(), viewport->height(), 1, GL_DEPTH_COMPONENT , GL_FLOAT); 

  // osgCam->attach(osg::Camera::COLOR_BUFFER, colorImage.get()); 
  // osgCam->attach(osg::Camera::DEPTH_BUFFER, zImage.get(), 0, 0); /* */ 
  osg::ref_ptr<FinalDrawCallback> cb = new FinalDrawCallback(osgCam.get());
  osgCam->setFinalDrawCallback(cb);

  while (true) {
    scene.step(.01);
    viewer2.frame();
  }


  // vector<float> depthData;
  // depthData.assign((float*)zImage->data(), (float*)zImage->data()+800*800);
  // vecToFile("blah.txt", depthData);


}
