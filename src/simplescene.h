#pragma once
#include <osgViewer/Viewer>
#include <osgbCollision/GLDebugDrawer.h>
#include <osgGA/TrackballManipulator>
#include "environment.h"
#include "basicobjects.h"
#include "openravesupport.h"
#include "config.h"
#include "plotting.h"

class Scene;

class EventHandler : public osgGA::TrackballManipulator {
private:
  Scene *scene;
  float lastX, lastY, dx, dy;
protected:
  void getTransformation( osg::Vec3d& eye, osg::Vec3d& center, osg::Vec3d& up ) const;
public:
  EventHandler(Scene *scene_) : scene(scene_), state() {}
  struct {
    bool debugDraw,
         moveManip0, moveManip1,
         rotateManip0, rotateManip1,
         startDragging,
         idling;
  } state;
  bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa);
};

struct Scene {
  typedef boost::shared_ptr<Scene> Ptr;

  OSGInstance::Ptr osg;
  BulletInstance::Ptr bullet;
  RaveInstance::Ptr rave;
  Environment::Ptr env;
  boost::shared_ptr<osgbCollision::GLDebugDrawer> dbgDraw;
  osgViewer::Viewer viewer;
  osg::ref_ptr<EventHandler> manip;

  PlotPoints::Ptr plotPoints;
  PlotLines::Ptr plotLines;

  PlaneStaticObject::Ptr ground;
  RaveRobotKinematicObject::Ptr pr2;
  RaveRobotKinematicObject::Manipulator::Ptr pr2Left, pr2Right;

  Scene();

  void showWindow(bool showWindow, bool realtime);

  bool drawingOn, syncTime;
  void setDrawing(bool b) { drawingOn = b; }
  void setSyncTime(bool b) { syncTime = b; }
  // Starts the viewer. Must be called before any step/draw/viewerLoop call
  // and after adding objects to the environment
  void startViewer();

  // TODO: remove all dt params and use CFG.bullet.dt instead

  // Steps physics and updates the display (if displayOn is true)
  // If syncTime is true, then these will block until the time interval passes on the system clock
  virtual void step(float dt, int maxsteps, float internaldt);
  virtual void step(float dt);
  virtual void stepFor(float dt, float time);

  // Blocks the caller and just runs the viewer for the specified
  // time interval. If either syncTime or displayOn is false, this does nothing.
  void idleFor(float time);

  struct {
      float currTime, prevTime;
      bool looping, paused;
  } loopState;
  // Starts a viewer loop and blocks the caller.
  void startLoop(); // runs with a variable-rate dt that depends on the system speed
  void startFixedTimestepLoop(float dt); // runs with constant dt
  void stopLoop();
  // If the display is on, this will pause the simulation
  // while allowing the user to interact with the viewer.
  void idle(bool b);
  void toggleIdle();

  void runAction(Action &a, float dt);
  void runAction(Action::Ptr a, float dt) { runAction(*a.get(), dt); }

protected:
  void processHaptics();

  // Does debug drawing, updates the viewer window, and
  // processes OSG events
  virtual void draw();
};

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
