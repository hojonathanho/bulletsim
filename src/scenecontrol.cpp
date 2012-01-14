#include "scenecontrol.h"
#include "util.h"

#include <iostream>
using namespace std;

SceneControl::SceneControl() : handler(this), scene(&handler) {
  if (SceneConfig::enableRobot) {
    rave.reset(new RaveInstance());
    btTransform trans(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 5));
    robot.reset(new RaveRobotKinematicObject(rave, "robots/ravenII_2arm.robot.xml", trans, GeneralConfig::scale));
    scene.env->add(robot);
    robot->ignoreCollisionWith(scene.ground->rigidBody.get()); // the robot's always touching the ground anyway
  }

  if (SceneConfig::enableIK && SceneConfig::enableRobot) {
    robotLeft = robot->createManipulator("leftarm", SceneConfig::useFakeGrabber);
    robotRight = robot->createManipulator("rightarm", SceneConfig::useFakeGrabber);
    if (SceneConfig::useFakeGrabber) {
      scene.env->add(robotLeft->grabber);
      scene.env->add(robotRight->grabber);
    }
  }
}

void SceneControl::processHaptics() {
    if (!SceneConfig::enableRobot)
        return;

    // read the haptic controllers
    btTransform trans0, trans1;
    bool buttons0[2], buttons1[2];
    static bool lastButton[2] = { false, false };
    if (!util::getHapticInput(trans0, buttons0, trans1, buttons1))
        return;

    robotLeft->moveByIK(trans0, SceneConfig::enableRobotCollision, true);
    if (buttons0[0] && !lastButton[0]) {
        if (SceneConfig::useFakeGrabber)
            robotLeft->grabber->grabNearestObjectAhead();
        else
            cout << "not implemented" << endl;
    }
    else if (!buttons0[0] && lastButton[0]) {
        if (SceneConfig::useFakeGrabber)
            robotLeft->grabber->releaseConstraint();
        else
            cout << "not implemented" << endl;
    }
    lastButton[0] = buttons0[0];

    robotRight->moveByIK(trans0, SceneConfig::enableRobotCollision, true);
    if (buttons1[0] && !lastButton[1]) {
        if (SceneConfig::useFakeGrabber)
            robotRight->grabber->grabNearestObjectAhead();
        else
            cout << "not implemented" << endl;
    }
    else if (!buttons1[0] && lastButton[1]) {
        if (SceneConfig::useFakeGrabber)
            robotRight->grabber->releaseConstraint();
        else
            cout << "not implemented" << endl;
    }
    lastButton[1] = buttons1[0];
}

void SceneEventHandler::getTransformation( osg::Vec3d& eye, osg::Vec3d& center, osg::Vec3d& up ) const
  {
    center = _center;
    eye = _center + _rotation * osg::Vec3d( 0., 0., _distance );
    up = _rotation * osg::Vec3d( 0., 1., 0. );
  }

bool SceneEventHandler::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
    switch (ea.getEventType()) {
    case osgGA::GUIEventAdapter::KEYDOWN:
        switch (ea.getKey()) {
        case 'd':
            state.debugDraw = !state.debugDraw;
            controller->scene.dbgDraw->setEnabled(state.debugDraw);
            break;
        case 'p':
            controller->scene.toggleIdle();
            break;
        case '1':
            state.moveManip0 = true; break;
        case '2':
            state.moveManip1 = true; break;
        case 'q':
            state.rotateManip0 = true; break;
        case 'w':
	    state.rotateManip1 = true; break;
      }
      break;

    case osgGA::GUIEventAdapter::KEYUP:
        switch (ea.getKey()) {
        case '1':
            state.moveManip0 = false; break;
        case '2':
            state.moveManip1 = false; break;
        case 'q':
            state.rotateManip0 = false; break;
        case 'w':
            state.rotateManip1 = false; break;
        }
        break;

    case osgGA::GUIEventAdapter::PUSH:
        state.startDragging = true;
        return osgGA::TrackballManipulator::handle(ea, aa);

    case osgGA::GUIEventAdapter::DRAG:
        // drag the active manipulator in the plane of view
        if (SceneConfig::enableRobot && SceneConfig::enableIK &&
              (ea.getButtonMask() & ea.LEFT_MOUSE_BUTTON) &&
              (state.moveManip0 || state.moveManip1 ||
               state.rotateManip0 || state.rotateManip1)) {
            if (state.startDragging) {
                dx = dy = 0;
            } else {
                dx = lastX - ea.getXnormalized();
                dy = ea.getYnormalized() - lastY;
            }
            lastX = ea.getXnormalized(); lastY = ea.getYnormalized();
            state.startDragging = false;
  
            // get our current view
            osg::Vec3d osgCenter, osgEye, osgUp;
            getTransformation(osgCenter, osgEye, osgUp);
            btVector3 from(util::toBtVector(osgEye));
            btVector3 to(util::toBtVector(osgCenter));
            btVector3 up(util::toBtVector(osgUp)); up.normalize();
  
            // compute basis vectors for the plane of view
            // (the plane normal to the ray from the camera to the center of the scene)
            btVector3 normal = (to - from).normalized();
            btVector3 yVec = (up - (up.dot(normal))*normal).normalized(); //FIXME: is this necessary with osg?
            btVector3 xVec = normal.cross(yVec);
            btVector3 dragVec = SceneConfig::mouseDragScale * (dx*xVec + dy*yVec);

            RaveRobotKinematicObject::Manipulator::Ptr manip;
            if (state.moveManip0 || state.rotateManip0)
	        manip = controller->robotLeft;
            else
                manip = controller->robotRight;

            btTransform origTrans = manip->getTransform();
            btTransform newTrans(origTrans);

            if (state.moveManip0 || state.moveManip1)
                // if moving the manip, just set the origin appropriately
                newTrans.setOrigin(dragVec + origTrans.getOrigin());
            else if (state.rotateManip0 || state.rotateManip1) {
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
        } else {
            // if not dragging, we want the camera to move
            return osgGA::TrackballManipulator::handle(ea, aa);
        }
        break;

    default:
        return osgGA::TrackballManipulator::handle(ea, aa);
    }
    // this event handler doesn't actually change the camera, so return false
    // to let other handlers deal with this event too
    return false;
}
