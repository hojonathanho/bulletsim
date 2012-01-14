#include "basescene.h"
#include "config_bullet.h"
#include "config_viewer.h"
#include "util.h"
#include <iostream>
using namespace std;

BaseScene::BaseScene(osg::ref_ptr<SceneEventHandler> manip_) : manip(manip_) {
    osg.reset(new OSGInstance());
    bullet.reset(new BulletInstance());
    bullet->setGravity(BulletConfig::gravity);

    env.reset(new Environment(bullet, osg));

    // plots for debugging
    plotPoints.reset(new PlotPoints(GeneralConfig::scale * 0.5));
    env->add(plotPoints);
    plotLines.reset(new PlotLines(GeneralConfig::scale * 0.5));
    env->add(plotLines);

    // populate the scene with some basic objects
    boost::shared_ptr<btDefaultMotionState> ms;
    ms.reset(new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0))));
    ground.reset(new PlaneStaticObject(btVector3(0., 0., 1.), 0., ms));
    env->add(ground);
}

void BaseScene::startViewer() {
    drawingOn = syncTime = true;
    loopState.looping = loopState.paused = false;

    dbgDraw.reset(new osgbCollision::GLDebugDrawer());
    dbgDraw->setDebugMode(btIDebugDraw::DBG_MAX_DEBUG_DRAW_MODE /*btIDebugDraw::DBG_DrawWireframe*/);
    dbgDraw->setEnabled(false);
    bullet->dynamicsWorld->setDebugDrawer(dbgDraw.get());
    osg->root->addChild(dbgDraw->getSceneGraph());

    viewer.setUpViewInWindow(0, 0, ViewerConfig::windowWidth, ViewerConfig::windowHeight);
    manip->setHomePosition(util::toOSGVector(ViewerConfig::cameraHomePosition), osg::Vec3(), osg::Z_AXIS);
    viewer.setCameraManipulator(manip);
    viewer.setSceneData(osg->root.get());
    viewer.realize();
}

void BaseScene::step(float dt, int maxsteps, float internaldt) {
    static float startTime=viewer.getFrameStamp()->getSimulationTime(), endTime;

    if (syncTime && drawingOn)
        endTime = viewer.getFrameStamp()->getSimulationTime();

    env->step(dt, maxsteps, internaldt);
    draw();

    if (syncTime && drawingOn) {
        float timeLeft = dt - (endTime - startTime);
        idleFor(timeLeft);
        startTime = endTime + timeLeft;
    }
}

void BaseScene::step(float dt) {
    step(dt, BulletConfig::maxSubSteps, BulletConfig::internalTimeStep);
}

// Steps for a time interval
void BaseScene::stepFor(float dt, float time) {
    while (time > 0) {
        step(dt);
        time -= dt;
    }
}

// Idles for a time interval. Physics will not run.
void BaseScene::idleFor(float time) {
    if (!drawingOn || !syncTime || time <= 0.f)
        return;
    float endTime = time + viewer.getFrameStamp()->getSimulationTime();
    while (viewer.getFrameStamp()->getSimulationTime() < endTime && !viewer.done())
        draw();
}

void BaseScene::draw() {
    if (!drawingOn)
        return;
    if (manip->state.debugDraw) {
        dbgDraw->BeginDraw();
        bullet->dynamicsWorld->debugDrawWorld();
        dbgDraw->EndDraw();
    }
    viewer.frame();
}

void BaseScene::startLoop() {
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

void BaseScene::startFixedTimestepLoop(float dt) {
    loopState.looping = true;
    while (loopState.looping && drawingOn && !viewer.done())
        step(dt);
}

void BaseScene::stopLoop() {
    loopState.looping = false;
}

void BaseScene::idle(bool b) {
    loopState.paused = b;
    while (loopState.paused && drawingOn && !viewer.done())
        draw();
    loopState.prevTime = loopState.currTime = viewer.getFrameStamp()->getSimulationTime();
}

void BaseScene::toggleIdle() {
    idle(!loopState.paused);
}

void BaseScene::runAction(Action &a, float dt) {
    while (!a.done()) {
        a.step(dt);
        step(dt);
    }
}

