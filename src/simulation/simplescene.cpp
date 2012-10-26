#include "simplescene.h"
#include "environment.h"
#include "basicobjects.h"
#include "openravesupport.h"
#include "plotting.h"
#include "mouse_picking.h"
#include "config_bullet.h"
#include "config_viewer.h"
#include "util.h"
#include <iostream>
using namespace std;

Scene::Scene() {
  setup();
}
Scene::Scene(OpenRAVE::EnvironmentBasePtr env) {
  rave.reset(new RaveInstance(env));
  setup();
}

void Scene::setup() {
    osg.reset(new OSGInstance());
    bullet.reset(new BulletInstance());

    if (!rave) rave.reset(new RaveInstance());

    env.reset(new Environment(bullet, osg));

    // populate the scene with some basic objects
    //ground.reset(new PlaneStaticObject(btVector3(0., 0., 1.), 0., btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0))));
    ground.reset(new BoxObject(0, btVector3(5*METERS, 5*METERS, 0.01*METERS), btTransform(btQuaternion(0,0,0,1), btVector3(0,0,-0.01*METERS))));
    ground->collisionShape->setMargin(0.001*METERS);
    ground->rigidBody->setFriction(1.0);
    // in centimeters
		float width = 2*ground->getHalfExtents().x() * 100/METERS;
		float height = 2*ground->getHalfExtents().y() * 100/METERS;
		cv::Mat tex (height, width, CV_8UC3);
		// chessboard of squares of 10 cm
		for (int i=0; i<tex.rows; i++) {
			for (int j=0; j<tex.cols; j++) {
				if ((i/10)%2 != (j/10)%2) tex.at<cv::Vec3b>(i,j) = cv::Vec3b(135,184,222);
				//if ((i/10)%2 != (j/10)%2) tex.at<cv::Vec3b>(i,j) = cv::Vec3b(179,222,245);
				else tex.at<cv::Vec3b>(i,j) = cv::Vec3b(211,211,211);
			}
		}
		ground->setTexture(tex);

    env->add(ground);

    // default callbacks
    addVoidKeyCallback('p', boost::bind(&Scene::toggleIdle, this), "pause simulation");
    addVoidKeyCallback('d', boost::bind(&Scene::toggleDebugDraw, this), "toggle debug draw");
    addVoidKeyCallback('h', boost::bind(&Scene::help, this), "display help info");

    addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Escape, boost::bind(exit, 0), "(escape) exit");
    viewer.addEventHandler(new PickingMouseHandler(*this));

    drawingOn = false; // no drawing until startViewer()
}

void Scene::startViewer() {
    drawingOn = syncTime = true;
    loopState.looping = loopState.paused = loopState.debugDraw = false;

    dbgDraw.reset(new osgbCollision::GLDebugDrawer());
    dbgDraw->setDebugMode(btIDebugDraw::DBG_MAX_DEBUG_DRAW_MODE /*btIDebugDraw::DBG_DrawWireframe*/);
    dbgDraw->setEnabled(false);
    bullet->dynamicsWorld->setDebugDrawer(dbgDraw.get());
    osg->root->addChild(dbgDraw->getSceneGraph());

    {
    osg::ref_ptr<osg::Light> light = new osg::Light;
    light->setLightNum(0);
    light->setPosition(osg::Vec4(-10*METERS,0,10*METERS,1));
    osg::ref_ptr<osg::LightSource> lightSource = new osg::LightSource;
    lightSource->setLight(light.get());
    light->setDiffuse(osg::Vec4(1,.9,.9,1)*.5);
    osg->root->addChild(lightSource.get());
    osg->root->getOrCreateStateSet()->setMode(GL_LIGHT0, osg::StateAttribute::ON);
    }

    {
    osg::ref_ptr<osg::Light> light = new osg::Light;
    light->setLightNum(1);
    light->setPosition(osg::Vec4(10*METERS,0,10*METERS,1));
    osg::ref_ptr<osg::LightSource> lightSource = new osg::LightSource;
    lightSource->setLight(light.get());
    light->setDiffuse(osg::Vec4(.9,.9,1,1)*.5);
    osg->root->addChild(lightSource.get());
    osg->root->getOrCreateStateSet()->setMode(GL_LIGHT1, osg::StateAttribute::ON);
    }

    viewer.setUpViewInWindow(0, 0, ViewerConfig::windowWidth, ViewerConfig::windowHeight);
    manip = new EventHandler(*this);
    manip->setHomePosition(util::toOSGVector(ViewerConfig::cameraHomePosition)*METERS, util::toOSGVector(ViewerConfig::cameraHomeCenter)*METERS, util::toOSGVector(ViewerConfig::cameraHomeUp)*METERS);
    manip->setWheelZoomFactor(ViewerConfig::zoomFactor);
    viewer.setCameraManipulator(manip);
    viewer.setSceneData(osg->root.get());
    viewer.realize();
    step(0);
}

void Scene::toggleDebugDraw() {
    loopState.debugDraw = !loopState.debugDraw;
    dbgDraw->setEnabled(loopState.debugDraw);
}

void Scene::help() {
  printf("key bindings:\n");
  for (multimap<int,string>::iterator it = keyCallbackDescs.begin(); it != keyCallbackDescs.end(); ++it) {
    printf("%c: %s\n", (char)it->first, it->second.c_str());
  }

}

void Scene::step(float dt, int maxsteps, float internaldt) {

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
    loopState.looping = true;
    loopState.prevTime = loopState.currTime = viewer.getFrameStamp()->getSimulationTime();
    while (loopState.looping && drawingOn && !viewer.done()) {
        loopState.currTime = viewer.getFrameStamp()->getSimulationTime();
        step(loopState.currTime - loopState.prevTime);
        loopState.prevTime = loopState.currTime;
    }
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

void Scene::addKeyCallback(int c, Callback cb, std::string desc) {
    if (keyCallbacks.count(c) != 0)
        cout << "warning: key " << c << " is bound to multiple callbacks" << endl;
    keyCallbacks.insert(make_pair(c, cb));
    keyCallbackDescs.insert(make_pair(c, desc));
}

void Scene::addVoidCallback(osgGA::GUIEventAdapter::EventType t, VoidCallback cb) {
    addCallback(t, boost::bind<bool>(VoidCallbackWrapper(cb)));
}
void Scene::addVoidKeyCallback(int c, VoidCallback cb, std::string desc) {
    addKeyCallback(c, boost::bind<bool>(VoidCallbackWrapper(cb)), desc);
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
