#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>
#include <osgbCollision/GLDebugDrawer.h>
#include "environment.h"
#include "basicobjects.h"
#include "openravesupport.h"

const bool ENABLE_DEBUG_DRAW = false;

void initEnvironment(Environment &env, RaveInstance::Ptr rave) {
    boost::shared_ptr<btMotionState> ms;
    
    ms.reset(new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0))));
    PlaneStaticObject::Ptr ground(new PlaneStaticObject(btVector3(0., 0., 1.), 0., ms));
    env.add(ground);

#if 0
    ms.reset(new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 1.5))));
    CylinderStaticObject::Ptr cyl(new CylinderStaticObject(0.0, 1, 3, ms));
    env.add(cyl);

    ms.reset(new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(-10, 1, 1.5))));
    CylinderStaticObject::Ptr cyl2(new CylinderStaticObject(0.0, 1, 3, ms));
    env.add(cyl2);

    ms.reset(new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 10))));
    SphereObject::Ptr sphere(new SphereObject(1., 1., ms));
    env.add(sphere);
#endif

    btTransform trans(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0));
    RaveRobotKinematicObject::Ptr pr2(new RaveRobotKinematicObject(
        rave, "/home/jonathan/Downloads/pr2-beta-static.zae", trans));
    env.add(pr2);
}

int main() {
    OSGInstance::Ptr osg(new OSGInstance());
    BulletInstance::Ptr bullet(new BulletInstance());
    RaveInstance::Ptr rave(new RaveInstance());

    bullet->dynamicsWorld->setGravity(btVector3(0., 0., -9.8));

    Environment env(bullet, osg);
    initEnvironment(env, rave);

    // start osg
    osgbCollision::GLDebugDrawer *dbgDraw(NULL);
    if (ENABLE_DEBUG_DRAW) {
        dbgDraw = new osgbCollision::GLDebugDrawer();
        dbgDraw->setDebugMode(btIDebugDraw::DBG_DrawWireframe);
        bullet->dynamicsWorld->setDebugDrawer( dbgDraw );
        osg->root->addChild( dbgDraw->getSceneGraph() );
    }
    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow(30, 30, 768, 480);

    osg::ref_ptr<osgGA::TrackballManipulator> tb = new osgGA::TrackballManipulator();
    tb->setHomePosition(osg::Vec3(20, 0, 10), osg::Vec3(), osg::Z_AXIS);
    viewer.setCameraManipulator(tb.get());

    viewer.setSceneData(osg->root.get());

    double currSimTime = viewer.getFrameStamp()->getSimulationTime();
    double prevSimTime = prevSimTime;
    viewer.realize();
    while (!viewer.done()) {
        if (ENABLE_DEBUG_DRAW)
            dbgDraw->BeginDraw();

        currSimTime = viewer.getFrameStamp()->getSimulationTime();
        env.step(currSimTime - prevSimTime);
        prevSimTime = currSimTime;

        if (ENABLE_DEBUG_DRAW) {
            bullet->dynamicsWorld->debugDrawWorld();
            dbgDraw->EndDraw();
        }

        viewer.frame();
    }

    return 0;
}
