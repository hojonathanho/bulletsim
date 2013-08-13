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
	initHaptics();
	controller.reset(new RavensController(scene, ravens));
	registerSceneCallbacks();

	vector<KinBody::LinkPtr> links;
	manipR->manip->GetChildLinks(links);
	float gc = 151./255.;
	ravens->linkMap[links[1]]->setColor(gc,gc,gc,1);
	ravens->linkMap[links[2]]->setColor(gc,gc,gc,1);

	links.clear();
	manipL->manip->GetChildLinks(links);
	ravens->linkMap[links[1]]->setColor(gc,gc,gc,1);
	ravens->linkMap[links[2]]->setColor(gc,gc,gc,1);
	//ravens->linkMap[links[1]]->setColor(0.2,0.34,0.8,1);
	//ravens->linkMap[links[2]]->setColor(1.0,0.64,0.0,1);



	ravens->linkMap[ravens->robot->GetLink("rhandfinger1_sp")]->rigidBody->setCollisionFlags(
			ravens->linkMap[ravens->robot->GetLink("rhandfinger1_sp")]->rigidBody->getCollisionFlags()
			| btRigidBody::CF_NO_CONTACT_RESPONSE);;
	ravens->linkMap[ravens->robot->GetLink("rhandfinger2_sp")]->rigidBody->setCollisionFlags(
				ravens->linkMap[ravens->robot->GetLink("rhandfinger2_sp")]->rigidBody->getCollisionFlags()
				| btRigidBody::CF_NO_CONTACT_RESPONSE);;

	ravens->linkMap[ravens->robot->GetLink("lhandfinger1_sp")]->rigidBody->setCollisionFlags(
				ravens->linkMap[ravens->robot->GetLink("lhandfinger1_sp")]->rigidBody->getCollisionFlags()
				| btRigidBody::CF_NO_CONTACT_RESPONSE);;
	ravens->linkMap[ravens->robot->GetLink("lhandfinger2_sp")]->rigidBody->setCollisionFlags(
				ravens->linkMap[ravens->robot->GetLink("lhandfinger2_sp")]->rigidBody->getCollisionFlags()
				| btRigidBody::CF_NO_CONTACT_RESPONSE);;
}


void Ravens::registerSceneCallbacks() {
    Scene::Callback mousecb = boost::bind(&Ravens::processMouseInput, this, _1);
    scene.addCallback(osgGA::GUIEventAdapter::PUSH, mousecb);
    scene.addCallback(osgGA::GUIEventAdapter::DRAG, mousecb);

    Scene::Callback keycb = boost::bind(&Ravens::processKeyInput, this, _1);
    scene.addCallback(osgGA::GUIEventAdapter::KEYDOWN, keycb);
    scene.addCallback(osgGA::GUIEventAdapter::KEYUP, keycb);

    if (SceneConfig::enableHaptics)
        scene.addPreStepCallback(boost::bind(&Ravens::processHapticInput, this));
}


void Ravens::loadRobot() {
  if (!SceneConfig::enableRobot) return;

  OpenRAVE::RobotBasePtr maybeRobot = scene.rave->env->GetRobot("ravens");

  if (maybeRobot) { // if the robot is already loaded into openrave, use it
    ravens.reset(new RaveRobotObject(scene.rave, maybeRobot));
    scene.env->add(ravens);
  } else {
    static const char ROBOT_MODEL_FILE[] = EXPAND(BULLETSIM_SRC_DIR)"/tests/ravens/models/ravens.xml";
    ravens.reset(new RaveRobotObject(scene.rave, ROBOT_MODEL_FILE));
    scene.env->add(ravens);
  }
  ravens->setColor(0.2,0.2,0.2,1.0);
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
            inputState.moveManip0   = true; break;    hapTrackerLeft->rigidBody->setCollisionFlags(
                    hapTrackerLeft->rigidBody->getCollisionFlags()
                    | btRigidBody::CF_NO_CONTACT_RESPONSE);
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
            else    hapTrackerLeft->rigidBody->setCollisionFlags(
                    hapTrackerLeft->rigidBody->getCollisionFlags()
                    | btRigidBody::CF_NO_CONTACT_RESPONSE);
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
            manip->moveByIK(newTrans, false, false);
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


void Ravens::initHaptics() {

	if (!SceneConfig::enableHaptics) return;


    connectionInit(); // socket connection for haptics

    btTransform rT = btTransform::getIdentity();
    //lT.setOrigin(manipL->getTransform().getOrigin());
    hapTrackerLeft.reset(new SphereObject(0, 0.003*METERS, rT, true));
    hapTrackerLeft->rigidBody->setCollisionFlags(
            hapTrackerLeft->rigidBody->getCollisionFlags()
            | btRigidBody::CF_NO_CONTACT_RESPONSE);
    hapTrackerLeft->setColor(1, 0, 0, 0.8);
    scene.env->add(hapTrackerLeft);

    btTransform lT = btTransform::getIdentity();
    //rT.setOrigin(manipR->getTransform().getOrigin());
    hapTrackerRight.reset(new SphereObject(0, 0.003*METERS, lT, true));
    hapTrackerRight->rigidBody->setCollisionFlags(
            hapTrackerRight->rigidBody->getCollisionFlags()
            | btRigidBody::CF_NO_CONTACT_RESPONSE);
    hapTrackerRight->setColor(0, 1, 0, 0.8);
    scene.env->add(hapTrackerRight);

    setHapticPollRate(10); // default 10 hz


    this->ravens->ignoreCollisionWith(hapTrackerLeft->rigidBody.get());
    this->ravens->ignoreCollisionWith(hapTrackerRight->rigidBody.get());

    lEngaged = false;
    rEngaged = false;
    setHapticCb(hapticRight0Down,  boost::bind(&Ravens::toggleRightEngaged, this));
    setHapticCb(hapticLeft0Down, boost::bind(&Ravens::toggleLeftEngaged, this));

    setHapticCb(hapticRight1Down,  boost::bind(&Ravens::runRightGripperAction, this));
    setHapticCb(hapticLeft1Down,   boost::bind(&Ravens::runLeftGripperAction, this));

}


void Ravens::runLeftGripperAction () {
	scene.callGripperAction('l');
}

void Ravens::runRightGripperAction () {
	scene.callGripperAction('r');
}


void Ravens::processHapticInput() {
    if (!SceneConfig::enableHaptics)
        return;

    // throttle
    float currTime = scene.viewer.getFrameStamp()->getSimulationTime();
    if (currTime - inputState.lastHapticReadTime < 1./hapticPollRate)
        return;
    inputState.lastHapticReadTime = currTime;

    // read the haptic controllers
    btTransform trans0, trans1;
    bool buttons0[2], buttons1[2];
    if (!util::getHapticInput2(trans0, buttons0, trans1, buttons1)) {
        cout << "failed to read haptic input" << endl;
        return;
    }

    // adjust the transforms
    static const btVector3 ZERO_OFFSET  = btVector3(0, 31.1, 75);
    static const btScalar  HAPTIC_SCALE = 1. / 500 * METERS;
    //static const btVector3 HAPTIC_OFFSET_L = METERS*(btVector3( -0.10, 0, 0.10) + util::toBtVector(this->ravens->robot->GetLink("base_plate")->GetTransform().trans));
    //static const btVector3 HAPTIC_OFFSET_R = METERS*(btVector3( -0.10, 0, 0.10) + util::toBtVector(this->ravens->robot->GetLink("base_plate")->GetTransform().trans));
    static const btVector3 HAPTIC_OFFSET_L = METERS*(btVector3( -0.02, -0.1, 0.10) + util::toBtVector(this->ravens->robot->GetLink("base_plate")->GetTransform().trans));
    static const btVector3 HAPTIC_OFFSET_R = METERS*(btVector3(  0.02, -0.1, 0.10) + util::toBtVector(this->ravens->robot->GetLink("base_plate")->GetTransform().trans));

    static const btTransform CORRECTION(btMatrix3x3( -1, 0, 0,
    		0, 0, 1,
    		0, 1, 0));

    static const btTransform ROT(btMatrix3x3( 0,  1, 0,
    										 -1,  0, 0,
    										  0,  0, 1));
    //CORRECTION = ROT*CORRECTION;

    btVector3 translation0 = HAPTIC_SCALE * (trans0.getOrigin() + ZERO_OFFSET);
    btVector3 translation1 = HAPTIC_SCALE * (trans1.getOrigin() + ZERO_OFFSET);
    trans0.setOrigin(CORRECTION*translation0 + HAPTIC_OFFSET_L);
    trans1.setOrigin(CORRECTION*translation1 + HAPTIC_OFFSET_R);
    trans0.setBasis(CORRECTION.getBasis()*trans0.getBasis());
    trans1.setBasis(CORRECTION.getBasis()*trans1.getBasis());

    //set the transforms
    hapTrackerLeft->motionState->setKinematicPos (trans0);
    hapTrackerRight->motionState->setKinematicPos(trans1);
    handleButtons(buttons0, buttons1);
    if (lEngaged) manipL->moveByIK(trans0, false, false);
    if (rEngaged) manipR->moveByIK(trans1, false, false);
}


void Ravens::handleButtons(bool left[], bool right[]) {

  static bool lastLeft[2] = { left[0], left[1] };
  static bool lastRight[2] = { right[0], right[1] };

  vector<RavensHapticEvent> events;
  if (left[0] && !lastLeft[0]) events.push_back(hapticLeft0Down);
  if (!left[0] && lastLeft[0]) events.push_back(hapticLeft0Up);
  if (left[0]) events.push_back(hapticLeft0Hold);
  if (left[1] && !lastLeft[1]) events.push_back(hapticLeft1Down);
  if (!left[1] && lastLeft[1]) events.push_back(hapticLeft1Up);
  if (left[1]) events.push_back(hapticLeft1Hold);
  if (right[0] && !lastRight[0]) events.push_back(hapticRight0Down);
  if (!right[0] && lastRight[0]) events.push_back(hapticRight0Up);
  if (right[0]) events.push_back(hapticRight0Hold);
  if (right[1] && !lastRight[1]) events.push_back(hapticRight1Down);
  if (!right[1] && lastRight[1]) events.push_back(hapticRight1Up);
  if (right[1]) events.push_back(hapticRight1Hold);
  if (left[0] && left[1] && !(lastLeft[0] && lastLeft[1])) events.push_back(hapticLeftBoth);
  if (right[0] && right[1] && !(lastRight[0] && lastRight[1])) events.push_back(hapticRightBoth);

  lastLeft[0]  = left[0];
  lastLeft[1]  = left[1];
  lastRight[0] = right[0];
  lastRight[1] = right[1];

  BOOST_FOREACH(RavensHapticEvent evt, events) if (hapticEvent2Func.find(evt) != hapticEvent2Func.end()) hapticEvent2Func[evt]();

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












