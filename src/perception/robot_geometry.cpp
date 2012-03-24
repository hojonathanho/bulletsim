#include <openrave/openrave.h>
#include "btBulletDynamicsCommon.h"
#include "simulation/util.h"
#include "simulation/bullet_io.h"
using namespace OpenRAVE;

// btTransform getKinectToWorld(RobotBasePtr robot) {
//   btTransform kinFromCam(btQuaternion(0.03327,0.00521,0.00204, 0.99943), 
// 			 btVector3(-0.03968,0.10810,0.00415));
//   btTransform worldFromCam = util::toBtTransform(robot->GetLink("wide_stereo_optical_frame")->GetTransform());
//   return worldFromCam * kinFromCam.inverse();
// }

btTransform getKinectToWorld(RobotBasePtr robot) {
  return util::toBtTransform(robot->GetLink("camera_rgb_optical_frame")->GetTransform());
}
