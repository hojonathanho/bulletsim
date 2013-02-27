#include "ravens.h"
#include "simulation/environment.h"
#include "thread_socket_interface.h"


Ravens::Ravens(Scene &s) : scene(s), inputState(),
		_home({-0.153327, -2.29152, -0.0686984, 2.11724, -0.760324, -1.11199}),
		_side({1.832, -0.332, 1.011, -1.437, 1.1 , -2.106}) {

	arm_home.assign(_home, _home+6);
	arm_side.assign(_side, _side+6);
	loadRobot();
	initIK();
	controller.reset(new RavensController(scene, ravens));
	registerSceneCallbacks();
}


void Ravens::registerSceneCallbacks() {
    Scene::Callback mousecb = boost::bind(&Ravens::processMouseInput, this, _1);
    scene.addCallback(osgGA::GUIEventAdapter::PUSH, mousecb);
    scene.addCallback(osgGA::GUIEventAdapter::DRAG, mousecb);

    Scene::Callback keycb = boost::bind(&Ravens::processKeyInput, this, _1);
    scene.addCallback(osgGA::GUIEventAdapter::KEYDOWN, keycb);
    scene.addCallback(osgGA::GUIEventAdapter::KEYUP, keycb);
}


void Ravens::loadRobot() {
  if (!SceneConfig::enableRobot) return;

  OpenRAVE::RobotBasePtr maybeRobot = scene.rave->env->GetRobot("ravens");

  if (maybeRobot) { // if the robot is already loaded into openrave, use it
    ravens.reset(new RaveRobotObject(scene.rave, maybeRobot));
    scene.env->add(ravens);
  } else {
    static const char ROBOT_MODEL_FILE[] = "/home/ankush/sandbox/bulletsim/src/tests/ravens/models/ravens.xml";
    ravens.reset(new RaveRobotObject(scene.rave, ROBOT_MODEL_FILE));
    scene.env->add(ravens);
  }
  ravens->setColor(0.26,0.274,0.294,1.0);
}


void Ravens::initIK() {
    if (!SceneConfig::enableIK || !SceneConfig::enableRobot) return;
    if (!ravens)    return;
    manipL   = ravens->createManipulator("l_arm", SceneConfig::useFakeGrabber);
    manipR   = ravens->createManipulator("r_arm", SceneConfig::useFakeGrabber);
    leftInitTrans  = manipL->getTransform();
    rightInitTrans = manipR->getTransform();
}



bool Ravens::processKeyInput(const osgGA::GUIEventAdapter &ea) {
    switch (ea.getEventType()) {
    case osgGA::GUIEventAdapter::KEYDOWN:
        switch (ea.getKey()) {
        case '1':
            inputState.moveManip0   = true; break;
        case '2':
            inputState.moveManip1   = true; break;
        case 'q':
            inputState.rotateManip0 = true; break;
        case 'w':
            inputState.rotateManip1 = true; break;
        case '!':
            cycleIKSolution(0); break;
        case '@':
            cycleIKSolution(1); break;
        }
        break;
    case osgGA::GUIEventAdapter::KEYUP:
        switch (ea.getKey()) {
        case '1':
            inputState.moveManip0   = false; break;
        case '2':
            inputState.moveManip1   = false; break;
        case 'q':
            inputState.rotateManip0 = false; break;
        case 'w':
            inputState.rotateManip1 = false; break;
        }
        break;
    }
    return false;
}


void Ravens::applyTransform(OpenRAVE::Transform T) {
	{
		EnvironmentMutex::scoped_lock lock(scene.rave->env->GetMutex());
		OpenRAVE::Transform t = ravens->robot->GetTransform();
		ravens->robot->SetTransform(T*t);
		scene.rave->env->UpdatePublishedBodies();
	}
	ravens->updateBullet();
}


void Ravens::cycleIKSolution(int manipNum) {
    BOOST_ASSERT(manipNum == 0 || manipNum == 1);
    RaveRobotObject::Manipulator::Ptr manip =  manipNum == 0 ? manipL : manipR;
    int &ikSolnNum = manipNum == 0 ? inputState.ikSolnNum0 : inputState.ikSolnNum1;

    vector<vector<dReal> > solns;
    if (!manip->solveAllIK(manip->getTransform(), solns)) return;
    if (ikSolnNum >= solns.size()) ikSolnNum = 0;
    cout << "arm " << manipNum << ": setting ik solution number " << ikSolnNum << endl;
    ravens->setDOFValues(manip->manip->GetArmIndices(), solns[ikSolnNum]);
    ++ikSolnNum;
}


bool Ravens::processMouseInput(const osgGA::GUIEventAdapter &ea) {

    if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH) {
        inputState.startDragging = true;
    } else if (ea.getEventType() == osgGA::GUIEventAdapter::DRAG) {
        // drag the active manipulator in the plane of view
        if (SceneConfig::enableRobot && SceneConfig::enableIK &&
              (ea.getButtonMask() & ea.LEFT_MOUSE_BUTTON) &&
              (inputState.moveManip0 || inputState.moveManip1 ||
               inputState.rotateManip0 || inputState.rotateManip1)) {
            if (inputState.startDragging) {
                inputState.dx = inputState.dy = 0;
            } else {
                inputState.dx = inputState.lastX - ea.getXnormalized();
                inputState.dy = ea.getYnormalized() - inputState.lastY;
            }
            inputState.lastX = ea.getXnormalized(); inputState.lastY = ea.getYnormalized();
            inputState.startDragging = false;

            // get our current view
            osg::Vec3d osgCenter, osgEye, osgUp;
            scene.manip->getTransformation(osgCenter, osgEye, osgUp);
            btVector3 from(util::toBtVector(osgEye));
            btVector3 to(util::toBtVector(osgCenter));
            btVector3 up(util::toBtVector(osgUp)); up.normalize();

            // compute basis vectors for the plane of view
            // (the plane normal to the ray from the camera to the center of the scene)
            btVector3 normal = (to - from).normalized();
            btVector3 yVec = (up - (up.dot(normal))*normal).normalized(); //FIXME: is this necessary with osg?
            btVector3 xVec = normal.cross(yVec);
            btVector3 dragVec = SceneConfig::mouseDragScale * (inputState.dx*xVec + inputState.dy*yVec);

            RaveRobotObject::Manipulator::Ptr manip;
            if (inputState.moveManip0 || inputState.rotateManip0)
                manip = manipL;
            else
                manip = manipR;

            btTransform origTrans = manip->getTransform();
            btTransform newTrans(origTrans);

            if (inputState.moveManip0 || inputState.moveManip1)
                // if moving the manip, just set the origin appropriately
                newTrans.setOrigin(dragVec + origTrans.getOrigin());
            else if (inputState.rotateManip0 || inputState.rotateManip1) {
                // if we're rotating, the axis is perpendicular to the
                // direction the mouse is dragging
                btVector3 axis   = normal.cross(dragVec);
                btScalar angle   = dragVec.length();
                btQuaternion rot(axis, angle);
                // we must ensure that we never get a bad rotation quaternion
                // due to really small (effectively zero) mouse movements
                // this is the easiest way to do this:
                if (rot.length() > 0.99f && rot.length() < 1.01f)
                    newTrans.setRotation(rot * origTrans.getRotation());
            }
            manip->moveByIK(newTrans, SceneConfig::enableRobotCollision, true);
            return true;
        }
    }
    return false;
}



/** Set the joint values of the L('l' : left) or R('r' : right)
 * of the robot PROBOT to the JOINT_VALUES. */
void Ravens::setArmJointAngles(const vector<dReal> &joint_values, char lr) {
	assert(("Set Joint Angles: Expecting 6 values. Not found.",
			joint_values.size()==6));

	RaveRobotObject::Manipulator::Ptr armManip = (lr=='l') ? manipL : manipR;
	ravens->setDOFValues(armManip->manip->GetArmIndices(), joint_values);
}

const vector<dReal> & Ravens::getArmJointAngles(char lr) {
	RaveRobotObject::Manipulator::Ptr armManip = (lr=='l') ? manipL : manipR;
	return ravens->getDOFValues(armManip->manip->GetArmIndices());
}


/** Set the joint values of both the arms to
 *  left joints to JOINTS_LEFT and right to JOINTS_RIGHT.  */
void Ravens::setBothArmsJointAngles(const vector<dReal> &joint_left, const vector<dReal> &joint_right) {
	setArmJointAngles(joint_left,  'l');
	setArmJointAngles(joint_right, 'r');
}


/* Set the arm pose. pose \in {"side", "up"}*/
void Ravens::setArmPose(std::string pose, char lrb) {
	vector<dReal> * joints;
	joints = (pose=="home")? &arm_home: &arm_side;

	if (lrb == 'l') {
		setArmJointAngles(*joints, 'l');
	} else if (lrb == 'r') {
		vector<dReal> right_joints = mirror_ravens_joints(*joints);
		setArmJointAngles(right_joints, 'r');
	} else {
		vector<dReal> right_joints = mirror_ravens_joints(*joints);
		setBothArmsJointAngles(*joints, right_joints);
	}
}


//mirror image of joints (r->l or l->r)
std::vector<dReal> mirror_ravens_joints(const std::vector<dReal> &x) {
	assert(("Mirror Joints: Expecting 6 values. Not found.", x.size()==6));
	dReal vec[] = {-1*x[0],x[1], x[2],x[3], x[4],x[5]};
	std::vector<dReal> mirrored;
	mirrored.assign(vec,vec+6);
    return mirrored;
}


void RavensController::execute() {
	if (!enabled) return;

	if (trajectories.empty()) {
		enabled = false;
		return;
	}

	vector<dReal> jointVals;

	if (currentTime >= trajectories.front()->duration()) {
		trajectories.front()->sampleJoints(jointVals, trajectories.front()->duration());
		ravens->setDOFValues(trajectories.front()->dofIndices, jointVals);
		trajectories.pop();
		currentTime = 0;
	} else {
		trajectories.front()->sampleJoints(jointVals, currentTime);
		ravens->setDOFValues(trajectories.front()->dofIndices, jointVals);
		currentTime += dt;
	}
}
