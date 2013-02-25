#ifndef __PR2_H__
#define __PR2_H__

#include "simulation/openravesupport.h"
#include "simulation/basicobjects.h"
#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "simulation/fake_gripper.h"
#include <map>
#include <queue>

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

    bool grabOnlyOnContact;

    KinBody::LinkPtr leftFinger, rightFinger;

    // vector normal to the direction that the gripper fingers move in the manipulator frame
    // (on the PR2 this points back into the arm)
    const btVector3 closingNormal;

    // points straight down in the PR2 initial position (manipulator frame)
    const btVector3 toolDirection;

    // the target softbody
    BulletSoftObject::Ptr sb;

    btTransform getManipRot() const {
        btTransform trans(util::toBtTransform(manip->GetTransform(), GeneralConfig::scale));
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


/*Class for executing OpenRAVE trajectories on a PR2 in Bulletsim. */
class PR2Controller {

	/** Pointer to the PR2. */
    RaveRobotObject::Ptr pr2;

    /** Reference to the scene in which PR2 is present. */
    Scene &scene;

    /** Controller moves the robot iff, ENABLED is TRUE. */
    bool enabled;

	/** Queue to hold the trajectories to be executed.
	 *  The trajectories present in this queue will be executed sequentially,
	 *  in order, back-to-back immediately.
	 *  A trajectory is removed from the queue as soon as its execution is finished.*/
	std::queue<RaveTrajectory::Ptr> trajectories;

	/** Stores the time since the beginning of the trajectory it is executing.*/
	double currentTime;

	/** Simulation time step. */
	float dt;

	/** Main loop which executes the trajectory. Called by the scene at each step.*/
	void execute();

public:
	typedef boost::shared_ptr<PR2Controller> Ptr;


	PR2Controller (Scene &_scene, RaveRobotObject::Ptr _pr2, float _dt = -1) : scene(_scene),trajectories(), enabled(false),
		currentTime(0), dt(_dt), pr2(_pr2) {

		scene.addPreStepCallback(boost::bind(&PR2Controller::execute, this));

		if (dt == -1)
			dt = BulletConfig::dt;
	}

	/** Append the TRAJ to the list of trajectories to be executed.*/
	void appendTrajectory(RaveTrajectory::Ptr traj) {
		trajectories.push(traj);
	}

	/** Return the number of trajectories in the queue.*/
	int getNumTrajectories() {return trajectories.size(); }

	/** Start the controller. */
	void run(){
		if (!trajectories.empty())
			enabled = true;
	}

	/* Halt the controller.
	 * Controller does not move the robot until run/ resume is called. */
	void halt() {
		enabled = false;
	}

	/** Resume the operation of the controller after a halt.*/
	void resume() {run();}

	/** Reset the controller. Halts execution if any in progress.
	 *  Clears the list of trajectories.*/
	void reset(){
		enabled = false;
		while(!trajectories.empty()) trajectories.pop();
	}
};


/* Mirror image of joints (r->l or l->r). */
std::vector<dReal> mirror_arm_joints(const std::vector<dReal> &x);


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

    // arm postures
    const dReal _up[7];
    std::vector<dReal> arm_up;

    const dReal _side[7];
    std::vector<dReal> arm_side;


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

    /** Controller for DOFs of the pr2. */
    PR2Controller::Ptr controller;


    SphereObject::Ptr hapTrackerLeft, hapTrackerRight;
    bool lEngaged, rEngaged; // only accept haptic input if engaged
    bool armsDisabled; // hack so I can do demonstrations with fake gripper but still use haptics stuff

    PR2Manager(Scene &s);
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


    /** Set the joint values of the L('l' : left) or R('r' : right)
     *  arms to the PR2 to the JOINT_VALUES. */
    void setArmJointAngles(const vector<dReal> &joint_values, char lr = 'l');

    /** Set the joint values of both the arms to
     *  left joints to JOINTS_LEFT and right to JOINTS_RIGHT.  */
    void setBothArmsJointAngles(const vector<dReal> &joint_left,
    		const vector<dReal> &joint_right);

    /* Set the arm pose. pose \in {"side", "up"}*/
    void setArmPose(std::string pose="side", char lrb='l');

    /** Sets the torso link VALUE.
     *  Up is value == 1 | Down is value == 0 */
    void setTorso(dReal value);
};




#endif // __PR2_H__
