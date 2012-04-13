#ifndef __FL_GENGRIPPER_H__
#define __FL_GENGRIPPER_H__

#include "simulation/environment.h"
#include "simulation/openravesupport.h"
#include "simulation/fake_gripper.h"
#include "simulation/softbodies.h"

struct GenManip {
    typedef boost::shared_ptr<GenManip> Ptr;
    enum Type {
        TYPE_FAKE = 0,
        TYPE_IK
    };
    const Type type;
    struct {
        RaveRobotObject::Manipulator::Ptr ik;
        TelekineticGripper::Ptr fake;
    } manip;
    GenManip(RaveRobotObject::Manipulator::Ptr ikm) : type(TYPE_IK) { manip.ik = ikm; }
    GenManip(TelekineticGripper::Ptr fakem) : type(TYPE_FAKE) { manip.fake = fakem; }

    RaveRobotObject::Manipulator::Ptr asIKManip() const { return manip.ik; }
    TelekineticGripper::Ptr asFake() const { return manip.fake; }

    // every manip type should have a base manip
    // which can be used, for example, to open/close the gripper
    RaveRobotObject::Manipulator::Ptr baseManip() const;

    btTransform getTransform() const;
    GenManip::Ptr getForked(Fork &f, RaveRobotObject::Ptr forkRobot) const;
    btTransform getLinkTransform(KinBody::LinkPtr link) const;
    BulletObject::Ptr getLinkRigidBody(KinBody::LinkPtr link) const;
};

class GenPR2SoftGripper {
    RaveRobotObject::Ptr robot;
    GenManip::Ptr gmanip;

    bool grabOnlyOnContact;

    KinBody::LinkPtr leftFinger, rightFinger;

    // the target softbody
    BulletSoftObject::Ptr sb;

    // Checks if psb is touching the inside of the gripper fingers
    // If so, attaches anchors to every contact point
    void attach(bool left);
    bool grabbing;
    vector<BulletSoftObject::AnchorHandle> anchors;

public:
    typedef boost::shared_ptr<GenPR2SoftGripper> Ptr;

    GenPR2SoftGripper(RaveRobotObject::Ptr robot_, GenManip::Ptr gmanip_, bool leftGripper);

    void setGrabOnlyOnContact(bool b) { grabOnlyOnContact = b; }

    // Must be called before the action is run!
    void setTarget(BulletSoftObject::Ptr sb_) { sb = sb_; }

    void grab();
    void releaseAllAnchors();
    bool isGrabbing() const { return grabbing; }
};

#endif // __FL_GENGRIPPER_H__
