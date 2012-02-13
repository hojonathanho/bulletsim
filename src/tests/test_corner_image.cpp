#include "simplescene.h"
#include <osg/Camera>

int main(int charc, char* argv[]) {    
  SceneConfig::enableIK = SceneConfig::enableRobot = SceneConfig::enableHaptics = false;


  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.addGroup(SceneConfig());
  parser.read(argc, argv);
  parser.read(argc, argv);    
    
  // construct the scene
  Scene scene;
  // manipulate the scene or add more objects, if desired


  osg::Geometry* quadGeometry = osg::createTexturedQuadGeometry( osg::Vec3(0.0, 0.0, -1.0), osg::Vec3(screenWidth,0.0,0.0), osg::Vec3(0.0,screenHeight,0.0), texSize.x(),texSize.y() ); 

  Bind texture to the quadGeometry 

    Then use the following camera: 

  osg::Camera* camera = new osg::Camera; 

  camera->setClearMask(GL_DEPTH_BUFFER_BIT); 
  camera->setClearColor( osg::Vec4(0.f, 0.f, 0.f, 1.0) ); 
  camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF_INHERIT_VIEWPOINT); 
  camera->setProjectionMatrixAsOrtho( 0.f, screenWidth, 0.f, screenHeight, 1.0, 500.f ); 
  camera->setViewMatrix(osg::Matrix::identity()); 
  camera->setViewport( 0, 0, _screenDims.x(), _screenDims.y() ); 
  camera->setRenderOrder( osg::Camera::POST_RENDER ); 
  camera->addChild(quadGeometry); 

  root->addChild(camera); 


  // start the simulation
  scene.startViewer();
  scene.startLoop();

  return 0;
}
