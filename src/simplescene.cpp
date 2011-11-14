#include "simplescene.h"


  Scene::Scene() {
    osg.reset(new OSGInstance());
    bullet.reset(new BulletInstance());
    bullet->dynamicsWorld->setGravity(btVector3(0., 0., -9.8));
    rave.reset(new RaveInstance());
    env.reset(new Environment(bullet, osg));

    boost::shared_ptr<btMotionState> ms;
    ms.reset(new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0))));
    ground.reset(new PlaneStaticObject(btVector3(0., 0., 1.), 0., ms));
    env->add(ground);

    btTransform trans(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0));
    pr2.reset(new RaveRobotKinematicObject(rave, "robots/pr2-beta-sim.robot.xml", trans));
    env->add(pr2);


    dbgDraw = new osgbCollision::GLDebugDrawer();
    dbgDraw->setDebugMode(btIDebugDraw::DBG_MAX_DEBUG_DRAW_MODE /*btIDebugDraw::DBG_DrawWireframe*/);
    bullet->dynamicsWorld->setDebugDrawer(dbgDraw);


    viewer.setUpViewInWindow(30, 30, 800, 800);
    manip = createEventHandler();
    manip->state.debugDraw = true;
    manip->setHomePosition(osg::Vec3(5, 0, 5), osg::Vec3(), osg::Z_AXIS);
    viewer.setCameraManipulator(manip);
    viewer.setSceneData(osg->root.get());

    viewer.realize();

  }

void Scene::step(float dt, int maxsteps, float internaldt) {

    if (!osg->root->containsNode(dbgDraw->getSceneGraph()))
      osg->root->addChild(dbgDraw->getSceneGraph());



    env->step(dt, maxsteps, internaldt);

    if (manip->state.debugDraw) {
      dbgDraw->BeginDraw();
      bullet->dynamicsWorld->debugDrawWorld();
      dbgDraw->EndDraw();
    }

    viewer.frame();
  }

osg::ref_ptr<EventHandler> Scene::createEventHandler() { return osg::ref_ptr<EventHandler>(new EventHandler(this)); }


void EventHandler::getTransformation( osg::Vec3d& eye, osg::Vec3d& center, osg::Vec3d& up ) const
  {
    center = _center;
    eye = _center + _rotation * osg::Vec3d( 0., 0., _distance );
    up = _rotation * osg::Vec3d( 0., 1., 0. );
  }


  //the default TrackballManipulator has weird keybindings, so we set them here
  // virtual bool performMovementLeftMouseButton(double dt, double dx, double dy) {
  //     return osgGA::TrackballManipulator::performMovementMiddleMouseButton(dt, dx, dy);
  // }

  // virtual bool performMovementMiddleMouseButton(double dt, double dx, double dy) {
  //     return false;
  // }

  // virtual bool performMovementRightMouseButton(double dt, double dx, double dy) {
  //     return osgGA::TrackballManipulator::performMovementLeftMouseButton(dt, dx, dy);
  // }
        
EventHandler::EventHandler(Scene *scene_) : scene(scene_), state() {}

bool EventHandler::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
    switch (ea.getEventType()) {

    case osgGA::GUIEventAdapter::PUSH:
      state.startDragging = true;
      return osgGA::TrackballManipulator::handle(ea, aa);

    case osgGA::GUIEventAdapter::DRAG:
      // drag the active grabber in the plane of view
      if ((ea.getButtonMask() & ea.LEFT_MOUSE_BUTTON) &&
	  (state.moveGrabber0 || state.moveGrabber1)) {

	dx = lastX - ea.getXnormalized();
	dy = ea.getYnormalized() - lastY;
	lastX = ea.getXnormalized(); lastY = ea.getYnormalized();

	if (state.startDragging) { dx = dy = 0; }
	state.startDragging = false;

	// get our current view
	osg::Vec3d osgCenter, osgEye, osgUp;
	getTransformation(osgCenter, osgEye, osgUp);
	btVector3 from(util::toBtVector(osgEye));
	btVector3 to(util::toBtVector(osgCenter));
	btVector3 up(util::toBtVector(osgUp)); up.normalize();

	// compute basis vectors for the plane of view
	// (the plane normal to the ray from the camera to the center of the scene)
	btVector3 normal = to - from; normal.normalize();
	up = (up.dot(-normal))*normal + up; up.normalize(); //FIXME: is this necessary with osg?
	btVector3 xDisplacement = normal.cross(up) * dx;
	btVector3 yDisplacement = up * dy;

	// now set the position of the grabber
	int g = state.moveGrabber0 ? 0 : 1;

      } else {
	// if not dragging, we want the camera to move
	return osgGA::TrackballManipulator::handle(ea, aa);
      }

      break;

    default:
      return osgGA::TrackballManipulator::handle(ea, aa);
    }

    // this event handler doesn't actually change the camera, so return false
    // to let other handlers deal with this event too
    return false;
}




// int main() {


//   Scene s =     Scene();

//   for (int i=0; i<10000; i++)
//     s.step();
// }
