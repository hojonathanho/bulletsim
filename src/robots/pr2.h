#ifndef __PR2_H__
#define __PR2_H__

#include "simulation/openravesupport.h"
#include "simulation/basicobjects.h"
#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "simulation/fake_gripper.h"
#include <map>

enum HapticEvent {
  hapticLeft0Up,
  hapticLeft0Down,
  hapticLeft0Hold,
  hapticLeft1Up,
  hapticLeft1Down,
  hapticLeft1Hold,
  hapticLeftBoth,
  hapticRight0Up,
  hapticRight0Down,
  hapticRight0Hold,
  hapticRight1Up,
  hapticRight1Down,
  hapticRight1Hold,
  hapticRightBoth
};

// Special support for the OpenRAVE PR2 model

#define PR2_GRIPPER_OPEN_VAL 0.54f
#define PR2_GRIPPER_CLOSED_VAL 0.03f

class PR2SoftBodyGripper {
    RaveRobotObject::Ptr robot;
    OpenRAVE::RobotBase::ManipulatorPtr manip;

    float grabOnlyOnContact;

    KinBody::LinkPtr leftFinger, rightFinger;

    // vector normal to the direction that the gripper fingers move in the manipulator frame
    // (on the PR2 this points back into the arm)
    const btVector3 closingNormal;

    // points straight down in the PR2 initial position (manipulator frame)
    const btVector3 toolDirection;

    // the target softbody
    BulletSoftObject::Ptr sb;

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
        // we get an innermost point on the gripper by transforming a point
        // on the center of the gripper when it is closed
        return trans * (METERS/20.*btVector3((left ? 1 : -1) * 0.234402, -0.299, 0));
    }

    // Returns true is pt is on the inner side of the specified finger of the gripper
    bool onInnerSide(const btVector3 &pt, bool left) const {
        // then the innerPt and the closing direction define the plane
        return (getManipRot() * getClosingDirection(left)).dot(pt - getInnerPt(left)) > 0;
    }

    bool inGraspRegion(const btVector3 &pt) const;

    // Checks if psb is touching the inside of the gripper fingers
    // If so, attaches anchors to every contact point
    void attach(bool left);

    vector<BulletSoftObject::AnchorHandle> anchors;

public:
    typedef boost::shared_ptr<PR2SoftBodyGripper> Ptr;

    PR2SoftBodyGripper(RaveRobotObject::Ptr robot_, OpenRAVE::RobotBase::ManipulatorPtr manip_, bool leftGripper);

    void setGrabOnlyOnContact(bool b) { grabOnlyOnContact = b; }

    // Must be called before the action is run!
    void setTarget(BulletSoftObject::Ptr sb_) { sb = sb_; }

    void grab();
    void releaseAllAnchors();
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

    map<HapticEvent, boost::function<void()> > hapticEvent2Func;
  
    void actionWrapper(Action::Ptr a, float dt) {
        a->reset();
        scene.runAction(a, dt);
    }

public:
    typedef boost::shared_ptr<PR2Manager> Ptr;

    RaveRobotObject::Ptr pr2;
    RaveRobotObject::Manipulator::Ptr pr2Left, pr2Right;
    SphereObject::Ptr hapTrackerLeft, hapTrackerRight;
    bool lEngaged, rEngaged; // only accept haptic input if engaged
    bool armsDisabled; // hack so I can do demonstrations with fake gripper but still use haptics stuff

    PR2Manager(Scene &);
    void registerSceneCallbacks();

    void cycleIKSolution(int manipNum); // manipNum == 0 for left, 1 for right

    void setHapticPollRate(float hz) { hapticPollRate = hz; }

    void setHapticCb(HapticEvent h, boost::function<void()> cb) {hapticEvent2Func[h] = cb;}
    void setHapticCb(HapticEvent h, Action::Ptr a, float dt) { setHapticCb(h, boost::bind(&PR2Manager::actionWrapper, this, a, dt));}
    void handleButtons(bool left[], bool right[]);
    void toggleLeftEngaged() {lEngaged = !lEngaged;}
    void toggleRightEngaged() {rEngaged = !rEngaged;}

    void processHapticInput();
    bool processKeyInput(const osgGA::GUIEventAdapter &ea);
    bool processMouseInput(const osgGA::GUIEventAdapter &ea);
};

#endif // __PR2_H__
