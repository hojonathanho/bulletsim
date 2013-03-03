#ifndef __RAVENS_H__
#define __RAVENS_H__

#include "simulation/openravesupport.h"
#include "simulation/basicobjects.h"
#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "simulation/fake_gripper.h"
#include <map>
#include <queue>

class Scene;

namespace RavensHapticsEventId {
enum RavensHapticEvent {
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
}

/*Class for executing OpenRAVE trajectories on Ravens in Bulletsim. */
class RavensController {

	/** Pointer to the Ravens. */
    RaveRobotObject::Ptr ravens;

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
	typedef boost::shared_ptr<RavensController> Ptr;


	RavensController (Scene &_scene, RaveRobotObject::Ptr _ravens, float _dt = -1) : scene(_scene),trajectories(), enabled(false),
		currentTime(0), dt(_dt), ravens(_ravens) {

		scene.addPreStepCallback(boost::bind(&RavensController::execute, this));

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
std::vector<dReal> mirror_ravens_joints(const std::vector<dReal> &x);


using namespace RavensHapticsEventId;

class Ravens {
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

    // arm postures
    const dReal _home[6];
    std::vector<dReal> arm_home;
    const dReal _side[6];
    std::vector<dReal> arm_side;


    /** for haptic input. */
    btTransform leftInitTrans, rightInitTrans;
    float hapticPollRate;
    map<RavensHapticEvent, boost::function<void()> > hapticEvent2Func;
    void actionWrapper(Action::Ptr a, float dt) {
        a->reset();
        scene.runAction(a, dt);
    }
    void initHaptics();


public:
    typedef boost::shared_ptr<Ravens> Ptr;

    RaveRobotObject::Ptr                         ravens;
    RaveRobotObject::Manipulator::Ptr    manipL, manipR;
    /** Controller for DOFs of the ravens. */
    RavensController::Ptr                    controller;

    SphereObject::Ptr hapTrackerLeft, hapTrackerRight;
    bool lEngaged, rEngaged; // only accept haptic input if engaged


    Ravens(Scene &s);
    void registerSceneCallbacks();
    void cycleIKSolution(int manipNum); // manipNum == 0 for left, 1 for right
    bool processKeyInput(const osgGA::GUIEventAdapter &ea);
    bool processMouseInput(const osgGA::GUIEventAdapter &ea);


    /** Functions for Haptic input; using phantoms. */
    void setHapticPollRate(float hz) { hapticPollRate = hz; }
    void setHapticCb(RavensHapticEvent h, boost::function<void()> cb) {hapticEvent2Func[h] = cb;}
    void setHapticCb(RavensHapticEvent h, Action::Ptr a, float dt) { setHapticCb(h, boost::bind(&Ravens::actionWrapper, this, a, dt));}
    void handleButtons(bool left[], bool right[]);
    void toggleLeftEngaged() {lEngaged = !lEngaged;}
    void toggleRightEngaged() {rEngaged = !rEngaged;}
    void processHapticInput();


    /** Set the transform of the base of the robot.*/
    void applyTransform(OpenRAVE::Transform T);

    /** Get/Set the joint values of the L('l' : left) or R('r' : right)
     *  arms of RavensII to the JOINT_VALUES. */
    void setArmJointAngles(const vector<dReal> &joint_values, char lr = 'l');
    const vector<dReal> & getArmJointAngles(char lr = 'l');

    /** Set the joint values of both the arms to
     *  left joints to JOINTS_LEFT and right to JOINTS_RIGHT.  */
    void setBothArmsJointAngles(const vector<dReal> &joint_left,
    		const vector<dReal> &joint_right);

    /* Set the arm pose. pose \in {"side", "up"}*/
    void setArmPose(std::string pose="side", char lrb='l');
};
#endif // __RAVENS_H__
