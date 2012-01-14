#pragma once
#include "basescene.h"
#include "openravesupport.h"
#include "sceneconfig.h"

struct SceneControl {
  typedef boost::shared_ptr<SceneControl> Ptr;
  SceneEventHandler handler;
  BaseScene scene;
  RaveInstance::Ptr rave;
  RaveRobotKinematicObject::Ptr robot;
  RaveRobotKinematicObject::Manipulator::Ptr robotLeft, robotRight;
  SceneControl();

private:
  void processHaptics();
};
