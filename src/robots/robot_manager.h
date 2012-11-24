#pragma once

#include "simulation/openravesupport.h"

class Scene;

class RobotManager {
private:
    Scene &scene;

    struct {
        bool moveManip0, moveManip1,
             rotateManip0, rotateManip1,
             startDragging;
        float dx, dy, lastX, lastY;
        int ikSolnNum0, ikSolnNum1;
    } inputState;


    void loadRobot();
    void initIK();


public:
    typedef boost::shared_ptr<RobotManager> Ptr;

    RaveRobotObject::Ptr bot;
    RaveRobotObject::Manipulator::Ptr botLeft, botRight;

    RobotManager(Scene &s);
    void registerSceneCallbacks();

    void cycleIKSolution(int manipNum); // manipNum == 0 for left, 1 for right

    bool processKeyInput(const osgGA::GUIEventAdapter &ea);
    bool processMouseInput(const osgGA::GUIEventAdapter &ea);
};
