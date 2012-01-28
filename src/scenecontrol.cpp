#include "scenecontrol.h"
#include "util.h"

#include <iostream>
using namespace std;

SceneControl::SceneControl() {
  if (SceneConfig::enableRobot) {
    rave.reset(new RaveInstance());
    btTransform trans(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 5));
    robot.reset(new RaveRobotKinematicObject(rave, "robots/pr2-beta-sim.robot.xml", trans, GeneralConfig::scale));
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
