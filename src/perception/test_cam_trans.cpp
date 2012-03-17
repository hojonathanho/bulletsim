#include "simulation/simplescene.h"
#include "utils/vector_io.h"
#include "comm/comm.h"
#include "perception/utils_perception.h"
#include "perception/make_bodies.h"
#include "simulation/bullet_io.h"
#include "simulation/plotting.h"
#include <osg/io_utils>

CoordinateTransformer* loadTable(Scene& scene) { // load table from standard location and add it to the scene
  vector< vector<float> > vv = floatMatFromFile(onceFile("table_corners.txt").string());
  vector<btVector3> tableCornersCam = toBulletVectors(vv);
  CoordinateTransformer* CT = new CoordinateTransformer(getCamToWorldFromTable(tableCornersCam));

  vector<btVector3> tableCornersWorld = CT->toWorldFromCamN(tableCornersCam);
  BulletObject::Ptr table = makeTable(tableCornersWorld, .1*METERS);
  table->setColor(0,0,1,.25);
  scene.env->add(table);  
  
  return CT;
  
}    


int main(int argc, char* argv[]) {

  Parser parser;
  parser.addGroup(SceneConfig());
  parser.addGroup(GeneralConfig());
  parser.read(argc, argv);

  Scene scene;
  initComm();
  CoordinateTransformer* CT = loadTable(scene);

  scene.startViewer();

  btTransform tf = CT->worldFromCamUnscaled;
  btMatrix3x3 rotation = tf.getBasis();
  btVector3 translation = tf.getOrigin();

  using namespace util;
  PlotAxes::Ptr axes(new PlotAxes(toOSGVector(translation*METERS), toOSGVector(rotation.getColumn(0)), toOSGVector(rotation.getColumn(1)), toOSGVector(rotation.getColumn(2)), 2));
  scene.env->add(axes);

  osg::ref_ptr<osg::Camera> cam = scene.viewer.getCamera();
  OSGCamParams cp(CT->worldFromCamUnscaled, GeneralConfig::scale);

  //  cam->setProjectionMatrixAsPerspective(49,640/480., .01*METERS, 100*METERS);
  //cam->setViewMatrixAsLookAt(cp.eye, osg::Vec3f(0,0,1), osg::Vec3f(0,1,0));
  //origin, z axis, y axis
  cout << CT->worldFromCamUnscaled << endl;
  scene.manip->setHomePosition(cp.eye, cp.center + cp.eye, cp.up);
  cout << cp.eye << ", " << cp.center << ", " << cp.up << endl;



  scene.startLoop();

}
