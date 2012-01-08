#include "simplescene.h"
#include "config.h"
#include "config_bullet.h"
#include "config_viewer.h"
#include "util.h"
#include <openrave/kinbody.h>

class PR2RigidBodyGripperAction : public Action {
    RaveRobotKinematicObject::Manipulator::Ptr manip;
    dReal startVal, endVal;
    vector<int> indices;
    vector<dReal> vals;

    // min/max gripper dof vals
    static const float CLOSED_VAL = 0.03f, OPEN_VAL = 0.54f;

    KinBody::LinkPtr leftFinger, rightFinger;

    // the target rigidbody
    btRigidBody *target;

    boost::shared_ptr<btGeneric6DofConstraint> constraint;

    btScalar avg;
    // look for contacts between the specified finger and target
    // if the applied impulse reaches some threshold, this returns true
    // signifiying that the gripper cannot be closed further without penetrating the target
    bool checkContacts(bool left) {
        btRigidBody * const finger =
            manip->robot->associatedObj(left ? leftFinger : rightFinger)->rigidBody.get();

        const btScalar a = 0.15; // moving average
        const btScalar threshold = 5.; // 5 times average impulse means we stop closing the gripper

        BulletInstance::Ptr bullet = manip->robot->getEnvironment()->bullet;
        for (int i = 0; i < bullet->dispatcher->getNumManifolds(); ++i) {
            btPersistentManifold* contactManifold = bullet->dispatcher->getManifoldByIndexInternal(i);
            btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
            btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());

            if ((obA != finger || obB != target) && (obB != finger || obA != target))
                continue;

            for (int j = 0; j < contactManifold->getNumContacts(); j++) {
                btManifoldPoint& pt = contactManifold->getContactPoint(j);
                btScalar impulse = pt.getAppliedImpulse();
                cout << "curr " << impulse << '\n';
                cout << "average " << avg << '\n';
                if (avg <= 0.0001)
                    avg = impulse;
                if (impulse > threshold*avg)
                    return true;
                avg = a*impulse + (1. - a)*avg;
            }
        }

        return false;
    }

    // attaches the target to the manipulator
    void attach() {
        btRigidBody *rb = manip->robot->associatedObj(manip->manip->GetEndEffector())->rigidBody.get();
        btTransform manipTrans; rb->getMotionState()->getWorldTransform(manipTrans);
        btTransform targetTransInv; target->getMotionState()->getWorldTransform(targetTransInv);
        targetTransInv = targetTransInv.inverse();

        btTransform manipFrame(manipTrans.inverse().getRotation(), btVector3(0, 0, 0));
        btTransform targetFrame(targetTransInv.getRotation(), targetTransInv * manipTrans.getOrigin());
        constraint.reset(new btGeneric6DofConstraint(*rb, *target, manipFrame, targetFrame, false));

        // make the constraint completely rigid
        constraint->setLinearLowerLimit(btVector3(0, 0, 0));
        constraint->setLinearUpperLimit(btVector3(0, 0, 0));
        constraint->setAngularLowerLimit(btVector3(0, 0, 0));
        constraint->setAngularUpperLimit(btVector3(0, 0, 0));

        manip->robot->getEnvironment()->bullet->dynamicsWorld->addConstraint(constraint.get());
    }

    void releaseConstraint() {
        if (!constraint) return;
        manip->robot->getEnvironment()->bullet->dynamicsWorld->removeConstraint(constraint.get());
        constraint.reset();
    }

public:
    typedef boost::shared_ptr<PR2RigidBodyGripperAction> Ptr;
    PR2RigidBodyGripperAction(RaveRobotKinematicObject::Manipulator::Ptr manip_,
                  const string &leftFingerName,
                  const string &rightFingerName,
                  float time) :
            Action(time), manip(manip_), vals(1, 0),
            leftFinger(manip->robot->robot->GetLink(leftFingerName)),
            rightFinger(manip->robot->robot->GetLink(rightFingerName)),
            indices(manip->manip->GetGripperIndices()),
            target(NULL)
    {
        if (indices.size() != 1)
            cout << "WARNING: more than one gripper DOF; just choosing first one" << endl;
        setCloseAction();
    }

    ~PR2RigidBodyGripperAction() {
        releaseConstraint();
    }

    void setEndpoints(dReal start, dReal end) { startVal = start; endVal = end; }
    dReal getCurrDOFVal() const {
        vector<dReal> v;
        manip->robot->robot->GetDOFValues(v);
        return v[indices[0]];
    }
    void setOpenAction() { setEndpoints(getCurrDOFVal(), OPEN_VAL); } // 0.54 is the max joint value for the pr2
    void setCloseAction() { setEndpoints(getCurrDOFVal(), CLOSED_VAL); }
    void toggleAction() {
        if (endVal == CLOSED_VAL)
            setOpenAction();
        else if (endVal == OPEN_VAL)
            setCloseAction();
    }

    // Must be called before the action is run!
    void setTarget(btRigidBody *target_) { target = target_; }

    void reset() {
        Action::reset();
        releaseConstraint();
    }

    void step(float dt) {
        if (done()) return;

        // if opening, no need to check contact
        if (endVal != OPEN_VAL || !target) {
            // if there's a large force on the fingers
            // then we probably can't close any further
            if (checkContacts(true) || checkContacts(false)) {
                attach();
                setDone(true);
                return;
            }
        }

        stepTime(dt);
        float frac = fracElapsed();
        vals[0] = (1.f - frac)*startVal + frac*endVal;
        manip->robot->setDOFValues(indices, vals);
    }
};

class CustomScene;
class CustomKeyHandler : public osgGA::GUIEventHandler {
    CustomScene &scene;
public:
    CustomKeyHandler(CustomScene &scene_) : scene(scene_) { }
    bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&);
};

struct CustomScene : public Scene {
    PR2RigidBodyGripperAction::Ptr leftAction, rightAction;

    void run() {
        viewer.addEventHandler(new CustomKeyHandler(*this));

        const float dt = BulletConfig::dt;
        const float table_height = .5;
        const float table_thickness = .05;

        boost::shared_ptr<btDefaultMotionState> ms(new btDefaultMotionState(
            btTransform(btQuaternion(0, 0, 0, 1),
                        GeneralConfig::scale * btVector3(1, 0, table_height-table_thickness/2))));
        BoxObject::Ptr table(
            new BoxObject(0, GeneralConfig::scale * btVector3(.75,.75,table_thickness/2),ms));
        env->add(table);

        ms.reset(new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1),
                 GeneralConfig::scale * btVector3(0.7, 0.188, table_height + 0.1/2))));
        BoxObject::Ptr box(new BoxObject(1, GeneralConfig::scale * btVector3(.03, .03, .03), ms));
        env->add(box);
        pr2->ignoreCollisionWith(box->rigidBody.get());

        leftAction.reset(new PR2RigidBodyGripperAction(pr2Left, "l_gripper_l_finger_tip_link", "l_gripper_r_finger_tip_link", 1));
        leftAction->setTarget(box->rigidBody.get());
        rightAction.reset(new PR2RigidBodyGripperAction(pr2Right, "r_gripper_l_finger_tip_link", "r_gripper_r_finger_tip_link", 1));
        rightAction->setTarget(box->rigidBody.get());

        // open left gripper and set initial position
        leftAction->setOpenAction();
        runAction(leftAction, dt);
        pr2Left->moveByIK(btTransform(btQuaternion(-0.250283, 0.967325, -0.00955494, 0.0393573),
                    GeneralConfig::scale * btVector3(13.9589/20, 3.6742/20, 10.4952/20)));

        startViewer();
        startFixedTimestepLoop(dt);
    }

    void printInfo() {
        btVector3 v = pr2Left->getTransform().getOrigin();
        btQuaternion q = pr2Left->getTransform().getRotation();
        cout << "left gripper pos: " << v.x() << ' ' << v.y() << ' ' << v.z() << '\n';
        cout << "left gripper rot: " << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
        v = pr2Right->getTransform().getOrigin();
        q = pr2Right->getTransform().getRotation();
        cout << "right gripper pos: " << v.x() << ' ' << v.y() << ' ' << v.z() << '\n';
        cout << "right gripper rot: " << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
    }
};

bool CustomKeyHandler::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&) {
    switch (ea.getEventType()) {
    case osgGA::GUIEventAdapter::KEYDOWN:
        switch (ea.getKey()) {
        case 'a':
            scene.leftAction->reset();
            scene.leftAction->toggleAction();
            scene.runAction(scene.leftAction, BulletConfig::dt);
            break;
        case 's':
            scene.rightAction->reset();
            scene.rightAction->toggleAction();
            scene.runAction(scene.rightAction, BulletConfig::dt);
            break;
        case ' ':
            scene.printInfo();
            break;
        }
        break;
    }
    return false;
}

int main(int argc, char *argv[]) {
  Parser().read(argc,argv);
    GeneralConfig::scale = 20.;
    ViewerConfig::cameraHomePosition = btVector3(100, 0, 100);
    BulletConfig::dt = 0.01;
    BulletConfig::internalTimeStep = 0.01;
    BulletConfig::maxSubSteps = 0;
    SceneConfig::enableRobotCollision = false;

    CustomScene().run();
    return 0;
}
