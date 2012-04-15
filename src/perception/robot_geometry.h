#ifndef __ROBOT_GEOMETRY_H__
#define __ROBOT_GEOMETRY_H__

#include <openrave/openrave.h>
#include <simulation/util.h>

btTransform getKinectToWorld(RobotBasePtr robot);

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

#endif // __ROBOT_GEOMETRY_H__
