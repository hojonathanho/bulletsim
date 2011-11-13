#include <osgViewer/Viewer>
#include <osgbCollision/GLDebugDrawer.h>
#include <osgGA/TrackballManipulator>
#include "environment.h"
#include "basicobjects.h"
#include "openravesupport.h"
#include "util.h"



class EventHandler;

class Scene {
  typedef boost::shared_ptr<Scene> Ptr;

public:
  OSGInstance::Ptr osg;
  BulletInstance::Ptr bullet;
  Environment::Ptr env;
  RaveInstance::Ptr rave;
  PlaneStaticObject::Ptr ground;
  RaveRobotKinematicObject::Ptr pr2;
  osgbCollision::GLDebugDrawer *dbgDraw;
  osgViewer::Viewer viewer;

  osg::ref_ptr<EventHandler> manip;


  Scene();
  void step(float);
  osg::ref_ptr<EventHandler> createEventHandler();
};



class EventHandler : public osgGA::TrackballManipulator{
private:
  Scene *scene;
  float lastX, lastY, dx, dy;
protected:
  void getTransformation( osg::Vec3d& eye, osg::Vec3d& center, osg::Vec3d& up ) const;
public:
  EventHandler(Scene *scene_);
  struct {
    bool debugDraw, moveGrabber0, moveGrabber1, startDragging;
  } state;
  bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa);

};
