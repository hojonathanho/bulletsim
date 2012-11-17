#pragma once
#include <osgViewer/Viewer>
#include <osgbCollision/GLDebugDrawer.h>
#include <osgGA/TrackballManipulator>
#include "environment.h"
#include "basicobjects.h"
#include "openravesupport.h"
#include "plotting.h"
#include "utils/config.h"

class Scene;

class EventHandler : public osgGA::TrackballManipulator {
private:
  Scene &scene;
public:
  EventHandler(Scene &scene_) : scene(scene_) {}
  bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa);

  // for osg 2.8 - 3.x compatibility issues
  void getTransformation(osg::Vec3d &eye, osg::Vec3d &center, osg::Vec3d &up) const;
};

struct Scene {
  typedef boost::shared_ptr<Scene> Ptr;

  OSGInstance::Ptr osg;
  BulletInstance::Ptr bullet;
  RaveInstance::Ptr rave;

  Environment::Ptr env;
  std::set<Fork::Ptr> forks;

  boost::shared_ptr<osgbCollision::GLDebugDrawer> dbgDraw;
  osgViewer::Viewer viewer;
  osg::ref_ptr<EventHandler> manip;


  BoxObject::Ptr ground;

  // callbacks should return true if the default TrackballManipulator::handle behavior
  // should be suppressed. if all callbacks return false, then it won't be suppressed
  typedef boost::function<bool(const osgGA::GUIEventAdapter &)> Callback;
  typedef multimap<osgGA::GUIEventAdapter::EventType, Callback> CallbackMap;
  CallbackMap callbacks;
  void addCallback(osgGA::GUIEventAdapter::EventType t, Callback cb) { callbacks.insert(make_pair(t, cb)); }
  typedef multimap<int, Callback> KeyCallbackMap;
  KeyCallbackMap keyCallbacks;
  multimap<int, std::string> keyCallbackDescs;
  void addKeyCallback(int c, Callback cb, std::string desc="");

  typedef boost::function<void(void)> VoidCallback;
  void addVoidCallback(osgGA::GUIEventAdapter::EventType t, VoidCallback cb);
  void addVoidKeyCallback(int c, VoidCallback cb, std::string desc="");
    struct VoidCallbackWrapper {
        VoidCallback fn;
        VoidCallbackWrapper(VoidCallback fn_) : fn(fn_) { }
        bool operator()() { fn(); return false; }
    };

  vector<VoidCallback> prestepCallbacks;
  void addPreStepCallback(VoidCallback cb);

  vector<VoidCallback> predrawCallbacks;
  void addPreDrawCallback(VoidCallback cb);


  Scene();

  void showWindow(bool showWindow, bool realtime);

  bool drawingOn, syncTime;
  void setDrawing(bool b) { drawingOn = b; }
  void setSyncTime(bool b) { syncTime = b; }

  // If you register a Fork, then stepping the scene will also
  // step it in parallel with the main environment
  void registerFork(Fork::Ptr f) { forks.insert(f); }
  void unregisterFork(Fork::Ptr f) { forks.erase(f); }

  // Starts the viewer. Must be called before any step/draw/viewerLoop call
  // and after adding objects to the environment
  void startViewer();

  void toggleDebugDraw();
  void help();

  // TODO: remove all dt params and use CFG.bullet.dt instead

  // Steps physics and updates the display (if displayOn is true)
  // If syncTime is true, then these will block until the time interval passes on the system clock
  virtual void step(float dt, int maxsteps, float internaldt);
  virtual void step(float dt);
  void stepFor(float dt, float time);

  // Blocks the caller and just runs the viewer for the specified
  // time interval. If either syncTime or displayOn is false, this does nothing.
  void idleFor(float time);

  struct {
      float currTime, prevTime;
      bool looping, paused, debugDraw;
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

  virtual void draw();

protected:
  void processHaptics();

  // Does debug drawing, updates the viewer window, and
  // processes OSG events
};

struct SceneConfig : Config {
  static bool enableIK;
  static bool enableHaptics;
  static bool enableRobot;
  static bool enableRobotCollision;
  static bool useFakeGrabber;
  static bool startIdle;
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
