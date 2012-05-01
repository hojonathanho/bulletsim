#include "perception/fake_kinect.h"
#include "simulation/simplescene.h"
#include "robots/pr2.h"
#include "utils_perception.h"
#include "perception/robot_geometry.h"
#include "simulation/config_viewer.h"

int main() {

  initComm();

  Scene scene;
  SceneConfig::enableIK = false;

  PR2Manager pr2m(scene);
  KinectTransformer kinectTrans(pr2m.pr2->robot);
  kinectTrans.calibrate(btTransform::getIdentity());
  CoordinateTransformer CT(kinectTrans.getWFC());
  
  FakeKinect fk(scene.env->osg, CT.worldFromCamEigen);

  scene.startViewer();
  scene.step(0);

  while (true) {
    fk.sendMessage();
  }

}
