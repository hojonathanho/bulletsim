#include <osgViewer/Viewer>
#include <osgbCollision/GLDebugDrawer.h>
#include <osgGA/TrackballManipulator>
#include "environment.h"
#include "basicobjects.h"
#include "openravesupport.h"
#include "thread_socket_interface.h"
#include "util.h"
#include "rope.h"

struct Scene {
    typedef boost::shared_ptr<Scene> Ptr;

    Environment::Ptr env;
    RaveInstance::Ptr rave;

    PlaneStaticObject::Ptr ground;
    SphereObject::Ptr sphere;
    CapsuleObject::Ptr capsule;

    RaveRobotKinematicObject::Ptr pr2;
    RaveRobotKinematicObject::Manipulator::Ptr pr2Left, pr2Right;
    GrabberKinematicObject::Ptr grabber[2];

    CapsuleRope::Ptr rope;

    Scene(Environment::Ptr env_, RaveInstance::Ptr rave_) : env(env_), rave(rave_) {
        boost::shared_ptr<btMotionState> ms;

        ms.reset(new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0))));
        ground.reset(new PlaneStaticObject(btVector3(0., 0., 1.), 0., ms));
        env->add(ground);

    #if 0
        ms.reset(new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 1.5))));
        CylinderStaticObject::Ptr cyl(new CylinderStaticObject(0.0, 1, 3, ms));
        env.add(cyl);

        ms.reset(new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(-10, 1, 1.5))));
        CylinderStaticObject::Ptr cyl2(new CylinderStaticObject(0.0, 1, 3, ms));
        env.add(cyl2);
    #endif

        ms.reset(new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(1, 0, 3))));
        sphere.reset(new SphereObject(1, 0.1, ms));
        env->add(sphere);

        ms.reset(new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(2, 2, 5))));
        capsule.reset(new CapsuleObject(1, 0.1, 0.3, ms));
        env->add(capsule);
        
        btTransform trans(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0));
        pr2.reset(new RaveRobotKinematicObject(rave, "robots/pr2-beta-static.zae", trans));
        env->add(pr2);
        pr2Left = pr2->createManipulator("leftarm");
        pr2Right = pr2->createManipulator("rightarm");
        env->add(pr2Left->grabber);
        env->add(pr2Right->grabber);
        grabber[0] = pr2Left->grabber;
        grabber[1] = pr2Right->grabber;


          int nLinks = 20;
          btAlignedObjectArray<btVector3> ctrlPts;
          for (int i=0; i< nLinks; i++) {
            ctrlPts.push_back(btVector3(.25*i,0,1));
          }
        rope.reset(new CapsuleRope(ctrlPts, 0.5f));
        env->add(rope);
    }

    void processHaptics() {
        // read the haptic controllers
#if 0
        Vector3d start_proxy_pos, end_proxy_pos;
        Matrix3d start_proxy_rot, end_proxy_rot;
        bool start_proxybutton[2], end_proxybutton[2];
        static bool last_button[2] = { false, false };
        if (getDeviceState(start_proxy_pos, start_proxy_rot, start_proxybutton,
                           end_proxy_pos, end_proxy_rot, end_proxybutton)) {
            // unfortunately now we need to convert the eigen data structures
            // to bullet structures, which the rest of the program uses
            btVector3 hap0Pos(start_proxy_pos.z(), start_proxy_pos.x(), start_proxy_pos.y());
            //printf("hap0pos: %f %f %f\n", hap0Pos.x(), hap0Pos.y(), hap0Pos.z());
            btMatrix3x3 hap0Rot = MATRIX_EIGEN_TO_BT(start_proxy_rot);
            btTransform hap0Trans(hap0Rot * HAPTIC_ROTATION, hap0Pos*HAPTIC_TRANS_SCALE + HAPTIC_OFFSET0);
            grabber[0]->getKinematicMotionState().setKinematicPos(hap0Trans);
            if (start_proxybutton[0] && !last_button[0])
                grabber[0]->grabNearestObjectAhead();
            else if (!start_proxybutton[0] && last_button[0])
                grabber[0]->releaseConstraint();
            last_button[0] = start_proxybutton[0];

            btVector3 hap1Pos(end_proxy_pos.z(), end_proxy_pos.x(), end_proxy_pos.y());
            //printf("hap1pos: %f %f %f\n", hap1Pos.x(), hap1Pos.y(), hap1Pos.z());
            btMatrix3x3 hap1Rot = MATRIX_EIGEN_TO_BT(end_proxy_rot);
            btTransform hap1Trans(hap1Rot * HAPTIC_ROTATION, hap1Pos*HAPTIC_TRANS_SCALE + HAPTIC_OFFSET1);
            grabber[1]->getKinematicMotionState().setKinematicPos(hap1Trans);
            if (end_proxybutton[0] && !last_button[1])
                grabber[1]->grabNearestObjectAhead();
            else if (!end_proxybutton[0] && last_button[1])
                grabber[1]->releaseConstraint();
            last_button[1] = end_proxybutton[0];
        }
#endif

        btTransform trans0, trans1;
        bool buttons0[2], buttons1[2];
        static bool lastButton[2] = { false, false };
        if (!util::getHapticInput(trans0, buttons0, trans1, buttons1))
            return;

        pr2Left->moveByIK(trans0);
        if (buttons0[0] && !lastButton[0])
            grabber[0]->grabNearestObjectAhead();
        else if (!buttons0[0] && lastButton[0])
            grabber[0]->releaseConstraint();
        lastButton[0] = buttons0[0];

        pr2Right->moveByIK(trans0);
        if (buttons1[0] && !lastButton[1])
            grabber[1]->grabNearestObjectAhead();
        else if (!buttons1[0] && lastButton[1])
            grabber[1]->releaseConstraint();
        lastButton[1] = buttons1[0];
    }

    class EventHandler : public osgGA::TrackballManipulator {
    private:
        Scene *scene;
        float lastX, lastY, dx, dy;

    protected:



      void getTransformation( osg::Vec3d& eye, osg::Vec3d& center, osg::Vec3d& up ) const
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
        
    public:
        struct {
            bool debugDraw, moveGrabber0, moveGrabber1, startDragging;
        } state;
        EventHandler(Scene *scene_) : scene(scene_), state() {}

        bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
            switch (ea.getEventType()) {
            case osgGA::GUIEventAdapter::KEYDOWN:
                switch (ea.getKey()) {
                case 'd':
                    state.debugDraw = !state.debugDraw; break;
                case '1':
                    state.moveGrabber0 = true; break;
                case '2':
                    state.moveGrabber1 = true; break;
                }
                break;

            case osgGA::GUIEventAdapter::KEYUP:
                switch (ea.getKey()) {
                case '1':
                    state.moveGrabber0 = false; break;
                case '2':
                    state.moveGrabber1 = false; break;
                }
                break;

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
                    btTransform origTrans;
                    scene->grabber[g]->motionState->getWorldTransform(origTrans);
                    btTransform newTrans(origTrans);
                    newTrans.setOrigin(origTrans.getOrigin() + xDisplacement + yDisplacement);
                    if (g == 0) scene->pr2Left->moveByIK(newTrans);
                    else scene->pr2Right->moveByIK(newTrans);

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
    };
    osg::ref_ptr<EventHandler> createEventHandler() { return osg::ref_ptr<EventHandler>(new EventHandler(this)); }
};

int main() {
    OSGInstance::Ptr osg(new OSGInstance());
    BulletInstance::Ptr bullet(new BulletInstance());
    RaveInstance::Ptr rave(new RaveInstance());

    bullet->dynamicsWorld->setGravity(btVector3(0., 0., -9.8));

    Environment::Ptr env(new Environment(bullet, osg));
    Scene::Ptr scene(new Scene(env, rave));

    osgbCollision::GLDebugDrawer *dbgDraw = new osgbCollision::GLDebugDrawer();
    dbgDraw->setDebugMode(btIDebugDraw::DBG_MAX_DEBUG_DRAW_MODE /*btIDebugDraw::DBG_DrawWireframe*/);
    bullet->dynamicsWorld->setDebugDrawer(dbgDraw);

    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow(30, 30, 800, 800);
    osg::ref_ptr<Scene::EventHandler> manip = scene->createEventHandler();
    manip->setHomePosition(osg::Vec3(5, 0, 5), osg::Vec3(), osg::Z_AXIS);
    viewer.setCameraManipulator(manip);

    viewer.setSceneData(osg->root.get());

    connectionInit(); // haptics

    double currSimTime = viewer.getFrameStamp()->getSimulationTime();
    double prevSimTime = prevSimTime;
    viewer.realize();
    while (!viewer.done()) {
        if (manip->state.debugDraw) {
            if (!osg->root->containsNode(dbgDraw->getSceneGraph()))
                osg->root->addChild(dbgDraw->getSceneGraph());
            dbgDraw->BeginDraw();
        } else {
            osg->root->removeChild(dbgDraw->getSceneGraph());
        }

        scene->processHaptics();

        currSimTime = viewer.getFrameStamp()->getSimulationTime();
        env->step(currSimTime - prevSimTime, 200, 1./200.);
        prevSimTime = currSimTime;

        if (manip->state.debugDraw) {
            bullet->dynamicsWorld->debugDrawWorld();
            dbgDraw->EndDraw();
        }

        viewer.frame();
    }

    return 0;
}
