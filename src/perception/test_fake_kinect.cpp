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
  KinectTrans kinectTrans(pr2m.pr2->robot);
  kinectTrans.calibrate(btTransform(btQuaternion(-0.703407, 0.706030, -0.048280, 0.066401), btVector3(0.348212, -0.047753, 1.611060)));
  CoordinateTransformer CT(kinectTrans.getKinectTrans());
  
  FakeKinect fk(scene.env->osg, CT.worldFromCamEigen);

  scene.startViewer();
  scene.step(0);

  while (true) {
    fk.sendMessage();
  }

}
