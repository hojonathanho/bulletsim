#pragma once

struct SceneControl;

class SceneEventHandler : public osgGA::TrackballManipulator {
private:
  SceneControl *controller;
  float lastX, lastY, dx, dy;
protected:
  void getTransformation( osg::Vec3d& eye, osg::Vec3d& center, osg::Vec3d& up ) const;
public:
  SceneEventHandler(SceneControl *controller_) : controller(controller_), state() {}
  struct {
    bool debugDraw,
         moveManip0, moveManip1,
         rotateManip0, rotateManip1,
         startDragging,
         idling;
  } state;
  bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa);
};
