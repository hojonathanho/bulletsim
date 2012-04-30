#ifndef __ROBOT_GEOMETRY_H__
#define __ROBOT_GEOMETRY_H__

#include <openrave/openrave.h>
#include "simulation/util.h"
#include "utils/config.h"

btTransform getKinectToWorld(OpenRAVE::RobotBasePtr robot);

struct KinectTrans {
    btTransform relativeWorldFromKinect;
    OpenRAVE::KinBody::LinkPtr headplate;

    KinectTrans(OpenRAVE::KinBodyPtr pr2) : headplate(pr2->GetLink("head_plate_frame")) { }
    void calibrate(const btTransform &t) {
        relativeWorldFromKinect = util::toBtTransform(headplate->GetTransform()).inverse() * t;
    }
    btTransform getKinectTrans() const {
        btTransform f(util::toBtTransform(headplate->GetTransform()));
        return f * relativeWorldFromKinect;
    }
};

struct KinectTransformer {

	btTransform headFromKinect;
	OpenRAVE::KinBody::LinkPtr headplate;

	btTransform getWFC();
	void calibrate(const btTransform& trans) {
		headFromKinect = trans;
	}
	KinectTransformer(OpenRAVE::RobotBasePtr robot) :
		headFromKinect(btTransform::getIdentity()), headplate(
				robot->GetLink("wide_stereo_gazebo_l_stereo_camera_optical_frame")) {
	}
};



#endif // __ROBOT_GEOMETRY_H__
