#include <osgViewer/Viewer>
#include <osgbCollision/GLDebugDrawer.h>
#include <osgGA/TrackballManipulator>
#include "environment.h"
#include "basicobjects.h"
#include "openravesupport.h"
#include "thread_socket_interface.h"
#include "util.h"

struct Scene {
    typedef boost::shared_ptr<Scene> Ptr;

    Environment::Ptr env;
    RaveInstance::Ptr rave;

    PlaneStaticObject::Ptr ground;
    SphereObject::Ptr sphere;
    RaveRobotKinematicObject::Ptr pr2;
    GrabberKinematicObject::Ptr grabber[2];

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

        ms.reset(new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 3))));
        sphere.reset(new SphereObject(1, 0.1, ms));
        env->add(sphere);

        btTransform trans(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0));
        pr2.reset(new RaveRobotKinematicObject(rave, "/home/jonathan/Downloads/pr2-beta-static.zae", trans));
        env->add(pr2);

        grabber[0].reset(new GrabberKinematicObject(0.02, 0.05));
        env->add(grabber[0]);
        grabber[0]->getKinematicMotionState().setKinematicPos(btTransform(btQuaternion(0, 0, 0, 1), btVector3(5, 5, 0)));

        grabber[1].reset(new GrabberKinematicObject(0.02, 0.05));
        env->add(grabber[1]);
        grabber[1]->getKinematicMotionState().setKinematicPos(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 1, 2)));
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

        grabber[0]->getKinematicMotionState().setKinematicPos(trans0);
        if (buttons0[0] && !lastButton[0])
            grabber[0]->grabNearestObjectAhead();
        else if (!buttons0[0] && lastButton[0])
            grabber[0]->releaseConstraint();
        lastButton[0] = buttons0[0];

        grabber[1]->getKinematicMotionState().setKinematicPos(trans1);
        if (buttons1[0] && !lastButton[1])
            grabber[1]->grabNearestObjectAhead();
        else if (!buttons1[0] && lastButton[1])
            grabber[1]->releaseConstraint();
        lastButton[1] = buttons1[0];
    }

    class EventHandler : public osgGA::TrackballManipulator {
    private:
        bool debugDraw;

    protected:
        virtual bool performMovementLeftMouseButton(double dt, double dx, double dy) {
            return osgGA::TrackballManipulator::performMovementMiddleMouseButton(dt, dx, dy);
        }

        virtual bool performMovementMiddleMouseButton(double dt, double dx, double dy) {
            return false;
        }

        virtual bool performMovementRightMouseButton(double dt, double dx, double dy) {
            return osgGA::TrackballManipulator::performMovementLeftMouseButton(dt, dx, dy);
        }
        
    public:
        EventHandler() : debugDraw(false) { }

        bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
            switch (ea.getEventType()) {
            case osgGA::GUIEventAdapter::KEYDOWN:
                switch (ea.getKey()) {
                case 'd':
                    debugDraw = !debugDraw;
                    break;
                }
                break;

            default:
                return osgGA::TrackballManipulator::handle(ea, aa);
    /*        case osgGA::GUIEventAdapter::DRAG:
                float dx = lastX - ea.getXnormalized();
                float dy = ea.getYnormalized() - lastY;
                lastX = ea.getXnormalized(); lastY = ea.getYnormalized();

                if (ea.getButtonMask() & ea.RIGHT_MOUSE_BUTTON) {
                    printf("rmb dragging\n");
                    //setByMatrix(getMatrix() * osg::Matrix::rotate(dy/ osg::));
                }
                break;
*/
            }

            // this event handler doesn't actually change the camera, so return false
            // to let other handlers deal with this event too
            return false;
        }

        bool debugDrawOn() const { return debugDraw; }
    };
};

int main() {
    OSGInstance::Ptr osg(new OSGInstance());
    BulletInstance::Ptr bullet(new BulletInstance());
    RaveInstance::Ptr rave(new RaveInstance());

    bullet->dynamicsWorld->setGravity(btVector3(0., 0., -9.8));

    Environment::Ptr env(new Environment(bullet, osg));
    Scene::Ptr scene(new Scene(env, rave));

    // start osg
    osgbCollision::GLDebugDrawer *dbgDraw = new osgbCollision::GLDebugDrawer();
    dbgDraw->setDebugMode(btIDebugDraw::DBG_MAX_DEBUG_DRAW_MODE /*btIDebugDraw::DBG_DrawWireframe*/);
    bullet->dynamicsWorld->setDebugDrawer(dbgDraw);

    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow(30, 30, 800, 800);

    osg::ref_ptr<Scene::EventHandler> manip = new Scene::EventHandler;
    manip->setHomePosition(osg::Vec3(5, 0, 5), osg::Vec3(), osg::Z_AXIS);
    viewer.setCameraManipulator(manip);

    viewer.setSceneData(osg->root.get());

    connectionInit(); // haptics

    double currSimTime = viewer.getFrameStamp()->getSimulationTime();
    double prevSimTime = prevSimTime;
    viewer.realize();
    while (!viewer.done()) {
        if (manip->debugDrawOn()) {
            if (!osg->root->containsNode(dbgDraw->getSceneGraph()))
                osg->root->addChild(dbgDraw->getSceneGraph());
            dbgDraw->BeginDraw();
        } else {
            osg->root->removeChild(dbgDraw->getSceneGraph());
        }

        scene->processHaptics();

        currSimTime = viewer.getFrameStamp()->getSimulationTime();
        env->step(currSimTime - prevSimTime);
        prevSimTime = currSimTime;

        if (manip->debugDrawOn()) {
            bullet->dynamicsWorld->debugDrawWorld();
            dbgDraw->EndDraw();
        }

        viewer.frame();
    }

    return 0;
}
