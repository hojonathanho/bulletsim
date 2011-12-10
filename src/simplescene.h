#pragma once
#include <osgViewer/Viewer>
#include <osgbCollision/GLDebugDrawer.h>
#include <osgGA/TrackballManipulator>
#include "environment.h"
#include "basicobjects.h"
#include "openravesupport.h"

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
    bool debugDraw, moveGrabber0, moveGrabber1, startDragging, idling;
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

  PlaneStaticObject::Ptr ground;
  RaveRobotKinematicObject::Ptr pr2;
  RaveRobotKinematicObject::Manipulator::Ptr pr2Left, pr2Right;

  Scene();

  void showWindow(bool showWindow, bool realtime);

  bool drawingOn, syncTime;
  void setDrawing(bool b) { drawingOn = b; }
  void setTimeSync(bool b) { syncTime = b; }
  // Starts the viewer. Must be called before any step/draw/viewerLoop call
  // and after adding objects to the environment
  void startViewer();

  // Steps physics and updates the display (if displayOn is true)
  // If syncTime is true, then these will block until the time interval passes on the system clock
  void preStep();
  void step(float dt, int maxsteps, float internaldt);
  void step(float dt);
  void stepFor(float dt, float time);

  // Blocks the caller and just runs the viewer for the specified
  // time interval. If either syncTime or displayOn is false, this does nothing.
  void idleFor(float time);

  struct {
      float currTime, prevTime;
      bool looping, paused;
  } loopState;
  // Starts a viewer loop and blocks the caller.
  void startLoop();
  void stopLoop();
  // If the display is on, this will pause the simulation
  // while allowing the user to interact with the viewer.
  void idle(bool b);
  void toggleIdle();

  void runAction(Action &a, float dt);
  void runAction(Action::Ptr a, float dt) { runAction(*a.get(), dt); }

private:
  void processHaptics();

  // Does debug drawing, updates the viewer window, and
  // processes OSG events
  void draw();
};
