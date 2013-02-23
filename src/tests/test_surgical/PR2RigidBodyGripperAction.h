#include "simulation/simplescene.h"
#include "simulation/config_bullet.h"
#include "simulation/config_viewer.h"
#include "simulation/util.h"
#include "robots/pr2.h"
#include <openrave/kinbody.h>

class PR2RigidBodyGripperAction : public Action {
    RaveRobotObject::Manipulator::Ptr manip;
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
    PR2RigidBodyGripperAction(RaveRobotObject::Manipulator::Ptr manip_,
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
