#include "simplescene.h"
#include "mouse_picking.h"
#include "config_bullet.h"
#include "config_viewer.h"
#include "util.h"

#include <iostream>
using namespace std;

Scene::Scene() {
    osg.reset(new OSGInstance());
    bullet.reset(new BulletInstance());

    if (SceneConfig::enableRobot)
        rave.reset(new RaveInstance());

    env.reset(new Environment(bullet, osg));

    // populate the scene with some basic objects
    //ground.reset(new PlaneStaticObject(btVector3(0., 0., 1.), 0., btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0))));
    ground.reset(new BoxObject(0, btVector3(5*METERS, 5*METERS, 0.01*METERS), btTransform(btQuaternion(0,0,0,1), btVector3(0,0,-0.01*METERS))));
    ground->collisionShape->setMargin(0.001*METERS);
    ground->rigidBody->setFriction(1.0);
    env->add(ground);

    // default callbacks
    addVoidKeyCallback('p', boost::bind(&Scene::toggleIdle, this));
    addVoidKeyCallback('d', boost::bind(&Scene::toggleDebugDraw, this));

    addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Escape, boost::bind(exit, 0));
    viewer.addEventHandler(new PickingMouseHandler(*this));
}

void Scene::startViewer() {
    drawingOn = syncTime = true;
    loopState.looping = loopState.paused = loopState.debugDraw = false;

    dbgDraw.reset(new osgbCollision::GLDebugDrawer());
    dbgDraw->setDebugMode(btIDebugDraw::DBG_MAX_DEBUG_DRAW_MODE /*btIDebugDraw::DBG_DrawWireframe*/);
    dbgDraw->setEnabled(false);
    bullet->dynamicsWorld->setDebugDrawer(dbgDraw.get());
    osg->root->addChild(dbgDraw->getSceneGraph());

    viewer.setUpViewInWindow(0, 0, ViewerConfig::windowWidth, ViewerConfig::windowHeight);
    manip = new EventHandler(*this);
    manip->setHomePosition(util::toOSGVector(ViewerConfig::cameraHomePosition)*METERS, osg::Vec3(), osg::Z_AXIS);
    viewer.setCameraManipulator(manip);
    viewer.setSceneData(osg->root.get());
    viewer.realize();
    step(0);
}

osgViewer::View* Scene::startView() {
    drawingOn = syncTime = true;
    loopState.looping = loopState.paused = loopState.debugDraw = false;

    dbgDraw.reset(new osgbCollision::GLDebugDrawer());
    dbgDraw->setDebugMode(btIDebugDraw::DBG_MAX_DEBUG_DRAW_MODE /*btIDebugDraw::DBG_DrawWireframe*/);
    dbgDraw->setEnabled(false);
    bullet->dynamicsWorld->setDebugDrawer(dbgDraw.get());
    osg->root->addChild(dbgDraw->getSceneGraph());

    osgViewer::View *view = new osgViewer::View;

    view->setUpViewInWindow(0, 0, ViewerConfig::windowWidth, ViewerConfig::windowHeight);
    manip = new EventHandler(*this);
    manip->setHomePosition(util::toOSGVector(ViewerConfig::cameraHomePosition)*METERS, osg::Vec3(), osg::Z_AXIS);
    view->setCameraManipulator(manip);
    view->setSceneData(osg->root.get());
    //viewer.realize();
    step(0);
    return view;
}

void Scene::toggleDebugDraw() {
    loopState.debugDraw = !loopState.debugDraw;
    dbgDraw->setEnabled(loopState.debugDraw);
}

void Scene::step(float dt, int maxsteps, float internaldt) {
    static float startTime=viewer.getFrameStamp()->getSimulationTime(), endTime;

    if (syncTime && drawingOn)
        endTime = viewer.getFrameStamp()->getSimulationTime();

    // run pre-step callbacks
    for (int i = 0; i < prestepCallbacks.size(); ++i)
        prestepCallbacks[i]();

    env->step(dt, maxsteps, internaldt);
    for (std::set<Fork::Ptr>::iterator i = forks.begin(); i != forks.end(); ++i)
        (*i)->env->step(dt, maxsteps, internaldt);

    // run pre-draw callbacks
    for (int i = 0; i < predrawCallbacks.size(); ++i)
        predrawCallbacks[i]();

    draw();

    if (syncTime && drawingOn) {
        float timeLeft = dt - (endTime - startTime);
        idleFor(timeLeft);
        startTime = endTime + timeLeft;
    }
}

void Scene::step(float dt) {
    step(dt, BulletConfig::maxSubSteps, BulletConfig::internalTimeStep);
}

// Steps for a time interval
void Scene::stepFor(float dt, float time) {
    while (time > 0) {
        step(dt);
        time -= dt;
    }
}

// Idles for a time interval. Physics will not run.
void Scene::idleFor(float time) {
    if (!drawingOn || !syncTime || time <= 0.f)
        return;
    float endTime = time + viewer.getFrameStamp()->getSimulationTime();
    while (viewer.getFrameStamp()->getSimulationTime() < endTime && !viewer.done()) {
      draw();
      sleep(1/60.);
    }
}

void Scene::draw() {
    if (!drawingOn)
        return;
    if (loopState.debugDraw) {
        dbgDraw->BeginDraw();
        bullet->dynamicsWorld->debugDrawWorld();
        dbgDraw->EndDraw();
    }
    viewer.frame();
}

void Scene::startLoop() {
    bool oldSyncTime = syncTime;
    syncTime = false;
    loopState.looping = true;
    loopState.prevTime = loopState.currTime =
        viewer.getFrameStamp()->getSimulationTime();
    while (loopState.looping && drawingOn && !viewer.done()) {
        loopState.currTime = viewer.getFrameStamp()->getSimulationTime();
        step(loopState.currTime - loopState.prevTime);
        loopState.prevTime = loopState.currTime;
    }
    syncTime = oldSyncTime;
}

void Scene::startFixedTimestepLoop(float dt) {
    loopState.looping = true;
    while (loopState.looping && drawingOn && !viewer.done())
        step(dt);
}

void Scene::stopLoop() {
    loopState.looping = false;
}

void Scene::idle(bool b) {
    loopState.paused = b;
    while (loopState.paused && drawingOn && !viewer.done()) {
      draw();
      sleep(1/60.);
    }
    loopState.prevTime = loopState.currTime = viewer.getFrameStamp()->getSimulationTime();
}

void Scene::toggleIdle() {
    idle(!loopState.paused);
}

void Scene::runAction(Action &a, float dt) {
    while (!a.done()) {
        a.step(dt);
        step(dt);
    }
}

void Scene::addKeyCallback(int c, Callback cb) {
    if (keyCallbacks.count(c) != 0)
        cout << "warning: key " << c << " is bound to multiple callbacks" << endl;
    keyCallbacks.insert(make_pair(c, cb));
}

void Scene::addVoidCallback(osgGA::GUIEventAdapter::EventType t, VoidCallback cb) {
    addCallback(t, boost::bind<bool>(VoidCallbackWrapper(cb)));
}
void Scene::addVoidKeyCallback(int c, VoidCallback cb) {
    addKeyCallback(c, boost::bind<bool>(VoidCallbackWrapper(cb)));
}

void Scene::addPreStepCallback(VoidCallback cb) {
    prestepCallbacks.push_back(cb);
}

void Scene::addPreDrawCallback(VoidCallback cb) {
    predrawCallbacks.push_back(cb);
}

void EventHandler::getTransformation(osg::Vec3d &eye, osg::Vec3d &center, osg::Vec3d &up) const {
    center = _center;
    eye = _center + _rotation * osg::Vec3d(0., 0., _distance);
    up = _rotation * osg::Vec3d(0., 1., 0.);
}

bool EventHandler::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
    bool suppressDefault = false;
    osgGA::GUIEventAdapter::EventType t = ea.getEventType();

    // keypress handlers (for convenience)
    if (t == osgGA::GUIEventAdapter::KEYDOWN) {
        pair<Scene::KeyCallbackMap::const_iterator, Scene::KeyCallbackMap::const_iterator> range =
            scene.keyCallbacks.equal_range(ea.getKey());
        for (Scene::KeyCallbackMap::const_iterator i = range.first; i != range.second; ++i)
            suppressDefault |= i->second(ea);
    }

    // general handlers
    pair<Scene::CallbackMap::const_iterator, Scene::CallbackMap::const_iterator> range =
        scene.callbacks.equal_range(t);
    for (Scene::CallbackMap::const_iterator i = range.first; i != range.second; ++i)
        suppressDefault |= i->second(ea);

    if (!suppressDefault)
        return osgGA::TrackballManipulator::handle(ea, aa);

    return false;
}

bool SceneConfig::enableIK = true;
bool SceneConfig::enableHaptics = false;
bool SceneConfig::enableRobot = true;
bool SceneConfig::enableRobotCollision = false;
bool SceneConfig::useFakeGrabber = false;
float SceneConfig::mouseDragScale = 1.;
