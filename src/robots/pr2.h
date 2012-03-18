#ifndef __PR2_H__
#define __PR2_H__

#include "simulation/openravesupport.h"
#include "simulation/basicobjects.h"
#include "simulation/simplescene.h"

// Special support for the OpenRAVE PR2 model

#define PR2_GRIPPER_OPEN_VAL 0.54f
#define PR2_GRIPPER_CLOSED_VAL 0.03f

class PR2SoftBodyGripper {
    RaveRobotKinematicObject::Ptr robot;
    OpenRAVE::RobotBase::ManipulatorPtr manip;

    float grabOnlyOnContact;

    KinBody::LinkPtr leftFinger, rightFinger;
    const btTransform origLeftFingerInvTrans, origRightFingerInvTrans;

    // the point right where the fingers meet when the gripper is closed
    // (in the robot's initial pose)
    const btVector3 centerPt;

    // vector normal to the direction that the gripper fingers move in the manipulator frame
    // (on the PR2 this points back into the arm)
    const btVector3 closingNormal;

    // points straight down in the PR2 initial position (manipulator frame)
    const btVector3 toolDirection;

    // the target softbody
    btSoftBody *psb;

    btTransform getManipRot() const {
        btTransform trans(util::toBtTransform(manip->GetTransform(), robot->scale));
        trans.setOrigin(btVector3(0, 0, 0));
        return trans;
    }

    // Returns the direction that the specified finger will move when closing
    // (manipulator frame)
    btVector3 getClosingDirection(bool left) const {
        return (left ? 1 : -1) * toolDirection.cross(closingNormal);
    }

    // Finds some innermost point on the gripper
    btVector3 getInnerPt(bool left) const {
        btTransform trans(robot->getLinkTransform(left ? leftFinger : rightFinger));
        // this assumes that the gripper is symmetric when it is closed
        // we get an innermost point on the gripper by transforming a point
        // on the center of the gripper when it is closed
        const btTransform &origInv = left ? origLeftFingerInvTrans : origRightFingerInvTrans;
        return trans * origInv * centerPt;
        // actually above, we can just cache origInv * centerPt
    }

    // Returns true is pt is on the inner side of the specified finger of the gripper
    bool onInnerSide(const btVector3 &pt, bool left) const {
        // then the innerPt and the closing direction define the plane
        return (getManipRot() * getClosingDirection(left)).dot(pt - getInnerPt(left)) > 0;
    }

    // Checks if psb is touching the inside of the gripper fingers
    // If so, attaches anchors to every contact point
    void attach(bool left);

public:
    typedef boost::shared_ptr<PR2SoftBodyGripper> Ptr;

    PR2SoftBodyGripper(RaveRobotKinematicObject::Ptr robot_, OpenRAVE::RobotBase::ManipulatorPtr manip_, bool leftGripper);

    void setGrabOnlyOnContact(bool b) { grabOnlyOnContact = b; }

    // Must be called before the action is run!
    void setTarget(btSoftBody *psb_) { psb = psb_; }

    void grab();
    void releaseAllAnchors() { psb->m_anchors.clear(); }

    void setForkParams(Environment *env_, BulletInstance::Ptr newBullet_, OSGInstance::Ptr newOSG_);
};

class Scene;
class PR2Manager {
private:
    Scene &scene;

    struct {
        bool moveManip0, moveManip1,
             rotateManip0, rotateManip1,
             startDragging;
        float dx, dy, lastX, lastY;
        int ikSolnNum0, ikSolnNum1;
        float lastHapticReadTime;
    } inputState;

    void loadRobot();
    void initIK();
    void initHaptics();

    float hapticPollRate;
    btTransform leftInitTrans, rightInitTrans;
    SphereObject::Ptr hapTrackerLeft, hapTrackerRight;

    boost::function<void()> lopencb, lclosecb, ropencb, rclosecb;

    void actionWrapper(Action::Ptr a, float dt) {
        a->reset();
        scene.runAction(a, dt);
    }

public:
    typedef boost::shared_ptr<PR2Manager> Ptr;

    RaveRobotKinematicObject::Ptr pr2;
    RaveRobotKinematicObject::Manipulator::Ptr pr2Left, pr2Right;

    PR2Manager(Scene &);
    void registerSceneCallbacks();

    void cycleIKSolution(int manipNum); // manipNum == 0 for left, 1 for right

    void setHapticPollRate(float hz) { hapticPollRate = hz; }

    void setLGripperOpenCb(boost::function<void()> cb) { lopencb = cb; }
    void setLGripperCloseCb(boost::function<void()> cb) { lclosecb = cb; }
    void setRGripperOpenCb(boost::function<void()> cb) { ropencb = cb; }
    void setRGripperCloseCb(boost::function<void()> cb) { rclosecb = cb; }
    // convenience methods for calling actions as callbacks
    void setLGripperOpenCb(Action::Ptr a, float dt) { setLGripperOpenCb(boost::bind(&PR2Manager::actionWrapper, this, a, dt)); }
    void setLGripperCloseCb(Action::Ptr a, float dt) { setLGripperCloseCb(boost::bind(&PR2Manager::actionWrapper, this, a, dt)); }
    void setRGripperOpenCb(Action::Ptr a, float dt) { setRGripperOpenCb(boost::bind(&PR2Manager::actionWrapper, this, a, dt)); }
    void setRGripperCloseCb(Action::Ptr a, float dt) { setRGripperCloseCb(boost::bind(&PR2Manager::actionWrapper, this, a, dt)); }

    void processHapticInput();
    bool processKeyInput(const osgGA::GUIEventAdapter &ea);
    bool processMouseInput(const osgGA::GUIEventAdapter &ea);
};

#endif // __PR2_H__
