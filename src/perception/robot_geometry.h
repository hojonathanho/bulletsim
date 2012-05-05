#ifndef __ROBOT_GEOMETRY_H__
#define __ROBOT_GEOMETRY_H__

#include <openrave/openrave.h>
#include "simulation/util.h"
#include "utils/config.h"

struct KinectTransformer {

	btTransform headFromKinect;
	OpenRAVE::KinBody::LinkPtr headcamframe;

	btTransform getWFC();
	void calibrate(const btTransform& trans) {
		headFromKinect = trans;
	}
	KinectTransformer(OpenRAVE::RobotBasePtr robot) :
		headFromKinect(btTransform::getIdentity()), headcamframe(
				robot->GetLink("wide_stereo_gazebo_l_stereo_camera_optical_frame")) {
	}
};



#endif // __ROBOT_GEOMETRY_H__
