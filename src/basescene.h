#pragma once
#include <osgViewer/Viewer>
#include <osgbCollision/GLDebugDrawer.h>
#include <osgGA/TrackballManipulator>
#include "environment.h"
#include "basicobjects.h"
#include "config.h"
#include "plotting.h"
#include "sceneeventhandler.h"

struct BaseScene {
  typedef boost::shared_ptr<BaseScene> Ptr;

  OSGInstance::Ptr osg;
  BulletInstance::Ptr bullet;
  Environment::Ptr env;
  boost::shared_ptr<osgbCollision::GLDebugDrawer> dbgDraw;
  osgViewer::Viewer viewer;
  osg::ref_ptr<SceneEventHandler> manip;

  PlotPoints::Ptr plotPoints;
  PlotLines::Ptr plotLines;

  PlaneStaticObject::Ptr ground;

  BaseScene(osg::ref_ptr<SceneEventHandler> manip_);

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
  void startLoop(); // runs with a variable-rate dt that depends on the system speed
  void startFixedTimestepLoop(float dt); // runs with constant dt
  void stopLoop();
  // If the display is on, this will pause the simulation
  // while allowing the user to interact with the viewer.
  void idle(bool b);
  void toggleIdle();

  void runAction(Action &a, float dt);
  void runAction(Action::Ptr a, float dt) { runAction(*a.get(), dt); }

private:
  // Does debug drawing, updates the viewer window, and
  // processes OSG events
  void draw();
};
