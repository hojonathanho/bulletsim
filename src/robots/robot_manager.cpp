#include "robot_manager.h"
#include "simulation/simplescene.h"
#include "utils/logging.h"

RobotManager::RobotManager(Scene &s) : scene(s), inputState() {
    loadRobot();
    initIK();
    registerSceneCallbacks();
}

static void drive(RaveRobotObject* robot, float dx, float dy, float da) {
  LOG_DEBUG_FMT("driving %.2f %.2f %.2f", dx, dy, da);
  btTransform tf = robot->getTransform();
  tf = tf * btTransform(btQuaternion(0,0,da), btVector3(dx*METERS, dy*METERS, 0));
  robot->setTransform(tf);
}

static void adjustGrip(RaveRobotObject::Manipulator* manip, float dx) {
  float x = manip->getGripperAngle();
  manip->setGripperAngle(x+dx);
}


void RobotManager::registerSceneCallbacks() {
    Callback mousecb = boost::bind(&RobotManager::processMouseInput, this, _1);
    scene.addCallback(osgGA::GUIEventAdapter::PUSH, mousecb);
    scene.addCallback(osgGA::GUIEventAdapter::DRAG, mousecb);
    scene.addCallback(osgGA::GUIEventAdapter::SCROLL, mousecb);

    Callback keycb = boost::bind(&RobotManager::processKeyInput, this, _1);
    scene.addCallback(osgGA::GUIEventAdapter::KEYDOWN, keycb);
    scene.addCallback(osgGA::GUIEventAdapter::KEYUP, keycb);


    scene.addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Left, boost::bind(drive, bot.get(), 0,.05, 0));
    scene.addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Right, boost::bind(drive, bot.get(), 0,-.05, 0));
    scene.addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Up, boost::bind(drive, bot.get(), .05,0,  0));
    scene.addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Down, boost::bind(drive, bot.get(), -.05,0, 0));
    scene.addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Leftbracket, boost::bind(drive, bot.get(), 0,0, .05));
    scene.addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Rightbracket, boost::bind(drive, bot.get(), 0,0, -.05));

}

void RobotManager::loadRobot() {
  vector<RaveRobotObject::Ptr> robots = getRobots(scene.env, scene.rave);
  ENSURE(robots.size()==1);
  bot = robots[0];
}

void RobotManager::initIK() {
  if (!SceneConfig::enableIK || !SceneConfig::enableRobot)
    return;
  if (!bot) {
    //LOG(warning) << "cannot initialize IK since the Robot model is not yet loaded";
    return;
  }
  BOOST_FOREACH(RobotBase::ManipulatorPtr manip, bot->robot->GetManipulators()) {
    string name = manip->GetName();
    if (name == "arm" || name == "Arm") {
      botLeft = botRight =  bot->createManipulator(name);
    }
    if (name == "leftarm") {
      botLeft  =  bot->createManipulator(name);
    }
    if (name == "rightarm") {
      botRight =  bot->createManipulator(name);
    }

  }
}




bool RobotManager::processKeyInput(const osgGA::GUIEventAdapter &ea) {
    switch (ea.getEventType()) {
    case osgGA::GUIEventAdapter::KEYDOWN:
        switch (ea.getKey()) {
        case '1':
            inputState.moveManip0 = true; break;
        case '2':
            inputState.moveManip1 = true; break;
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
            inputState.moveManip0 = false; break;
        case '2':
            inputState.moveManip1 = false; break;
        case 'q':
            inputState.rotateManip0 = false; break;
        case 'w':
            inputState.rotateManip1 = false; break;
        }
        break;
    }
    return false;
}

void RobotManager::cycleIKSolution(int manipNum) {
    BOOST_ASSERT(manipNum == 0 || manipNum == 1);
    RaveRobotObject::Manipulator::Ptr manip =
        manipNum == 0 ? botLeft : botRight;
    int &ikSolnNum = manipNum == 0 ? inputState.ikSolnNum0 : inputState.ikSolnNum1;

    vector<vector<dReal> > solns;
    if (!manip->solveAllIK(manip->getTransform(), solns)) return;
    if (ikSolnNum >= solns.size()) ikSolnNum = 0;
    cout << "arm " << manipNum << ": setting ik solution number " << ikSolnNum << endl;
    bot->setDOFValues(manip->manip->GetArmIndices(), solns[ikSolnNum]);
    ++ikSolnNum;
}

bool RobotManager::processMouseInput(const osgGA::GUIEventAdapter &ea) {
    if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH) {
        inputState.startDragging = true;
    }
    else if (ea.getEventType() == osgGA::GUIEventAdapter::DRAG) {
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
                manip = botLeft;
            else
                manip = botRight;

            btTransform origTrans = manip->getTransform();
            btTransform newTrans(origTrans);

            if (inputState.moveManip0 || inputState.moveManip1)
                // if moving the manip, just set the origin appropriately
                newTrans.setOrigin(dragVec + origTrans.getOrigin());
            else if (inputState.rotateManip0 || inputState.rotateManip1) {
                // if we're rotating, the axis is perpendicular to the
                // direction the mouse is dragging
                btVector3 axis = normal.cross(dragVec);
                btScalar angle = dragVec.length();
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
    else if (ea.getEventType() == osgGA::GUIEventAdapter::SCROLL) {
      float scroll=0;
      if (ea.getScrollingMotion()==ea.SCROLL_UP) scroll=.01;
      else if (ea.getScrollingMotion()==ea.SCROLL_DOWN) scroll=-.01;
      RaveRobotObject::Manipulator* manip;
      if (inputState.moveManip0 || inputState.rotateManip0)
          manip = botLeft.get();
      else
          manip = botRight.get();
      adjustGrip(manip, scroll);
      return true;
    }
//  // drag the active manipulator in the plane of view
//      if (SceneConfig::enableRobot && SceneConfig::enableIK &&
//          (ea.getButtonMask() & ea.LEFT_MOUSE_BUTTON) &&
//          (inputState.moveManip0 || inputState.moveManip1 ||
//          inputState.rotateManip0 || inputState.rotateManip1)) {
//        if (inputState.startDragging) {
//          inputState.dx = inputState.dy = 0;
//        }
//        else {
//          inputState.dx = inputState.lastX - ea.getXnormalized();
//          inputState.dy = ea.getYnormalized() - inputState.lastY;
//        }
//        inputState.lastX = ea.getXnormalized(); inputState.lastY = ea.getYnormalized();
//        inputState.startDragging = false;

    return false;
}
