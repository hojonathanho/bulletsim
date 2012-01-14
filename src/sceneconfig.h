#pragma once

struct SceneConfig : Config {
  static bool enableIK;
  static bool enableHaptics;
  static bool enableRobot;
  static bool enableRobotCollision;
  static bool useFakeGrabber;
  static float mouseDragScale;
  SceneConfig() : Config() {
    params.push_back(new Parameter<bool>("enableIK", &enableIK, "enable OpenRAVE IK for the PR2"));
    params.push_back(new Parameter<bool>("enableHaptics", &enableHaptics, "enable haptics for the PR2"));
    params.push_back(new Parameter<bool>("enableRobot", &enableRobot, "enable the PR2"));
    params.push_back(new Parameter<bool>("enableRobotCollision", &enableRobotCollision, "collision detection between robot and environment during user manipulation"));
    params.push_back(new Parameter<bool>("useFakeGrabber", &useFakeGrabber, "use a fake grabber; pass false for realistic grasping"));
    params.push_back(new Parameter<float>("mouseDragScale", &mouseDragScale, "scaling factor for mouse control for IK"));    
  }
};
