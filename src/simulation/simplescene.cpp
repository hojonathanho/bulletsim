#include "simplescene.h"
#include "mouse_picking.h"
#include "config_bullet.h"
#include "config_viewer.h"
#include "util.h"

#include <iostream>
using namespace std;

Scene::Scene()  : userInput(false) {
    osg.reset(new OSGInstance());
    bullet.reset(new BulletInstance());

    if (SceneConfig::enableRobot)
        rave.reset(new RaveInstance());

    env.reset(new Environment(bullet, osg));

    // populate the scene with some basic objects
    //ground.reset(new PlaneStaticObject(btVector3(0., 0., 1.), 0., btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0))));
    ground.reset(new BoxObject(0, btVector3(1*METERS, 1*METERS, 0.01*METERS), btTransform(btQuaternion(0,0,0,1), btVector3(0,0,-0.01*METERS))));
    ground->collisionShape->setMargin(0.001*METERS);
    ground->rigidBody->setFriction(1.0);
    ground->receiveShadow = true;
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

    viewer.setUpViewInWindow(0, 0, ViewerConfig::windowWidth, ViewerConfig::windowHeight);
    manip = new EventHandler(*this);
    manip->setHomePosition(util::toOSGVector(ViewerConfig::cameraHomePosition)*METERS, util::toOSGVector(ViewerConfig::cameraHomeCenter)*METERS, util::toOSGVector(ViewerConfig::cameraHomeUp)*METERS);
    manip->setWheelZoomFactor(ViewerConfig::zoomFactor);
    viewer.setCameraManipulator(manip);
    viewer.setSceneData(osg->root.get());

    //viewer.getCamera()->getView()->setLightingMode(osg::View::NO_LIGHT);

//    osg::Light* light = new osg::Light();
//    osg::LightSource * lightsource = new osg::LightSource();
//    lightsource->setLight(light);
//    osg->root->addChild(lightsource);
//
//    osg::StateSet* stateset = osg->root->getOrCreateStateSet();
//    lightsource->setStateSetModes(*stateset, osg::StateAttribute::ON);
//
//    light->setAmbient(osg::Vec4d(0.0, 0.0, 0.0, 1.0));
//    light->setDiffuse(osg::Vec4d(1.0, 1.0, 1.0, 1.0));
//    light->setSpecular(osg::Vec4d(0.5, 0.2, 0.5, 1.0));
//    light->setPosition(osg::Vec4d(2*METERS, 2*METERS, METERS*3.0, 1.0));


    osg::Light* light1 = new osg::Light();
    osg::LightSource * lightsource1 = new osg::LightSource();
    lightsource1->setLight(light1);
    osg->root->addChild(lightsource1);
    osg::StateSet* stateset1 = osg->root->getOrCreateStateSet();
    lightsource1->setStateSetModes(*stateset1, osg::StateAttribute::ON);
    light1->setAmbient(osg::Vec4d(0.2, 0.2, 0.2, 1.0));
    light1->setDiffuse(osg::Vec4d(1.0, 1.0, 1.0, 1.0));
    light1->setSpecular(osg::Vec4d(0.5, 0.5, 0.5, 1.0));
    light1->setPosition(osg::Vec4d(0*METERS, 0.5*METERS, METERS*.50, 1.0));


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
bool SceneConfig::enableShadows = false;
