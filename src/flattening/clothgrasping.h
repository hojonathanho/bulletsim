#ifndef __CLOTHGRASPING_H__
#define __CLOTHGRASPING_H__

#include "simulation/environment.h"
#include "simulation/openravesupport.h"
#include "robots/pr2.h"

// an action that just runs a given function once
class FunctionAction : public Action {
private:
    boost::function<void(void)> fn;

public:
    FunctionAction(boost::function<void(void)> fn_) : fn(fn_) { }

    void step(float) {
        if (done()) return;
        fn();
        setDone(true);
    }
};

// an action that runs a bunch of given actions in sequence
class ActionChain : public Action {
private:
    vector<Action::Ptr> actions;

public:
    ActionChain() { }
    ActionChain(vector<Action::Ptr> &actions_) : actions(actions_) { }

    ActionChain &append(Action::Ptr a) { actions.push_back(a); return *this; }
    ActionChain &append(Action *a) { actions.push_back(Action::Ptr(a)); return *this; }
    ActionChain &operator<<(Action::Ptr a) { return append(a); }
    ActionChain &operator<<(Action *a) { return append(a); }
    void clear() { actions.clear(); }

    bool done() const {
        if (isDone) return true; // set by setDone()
        for (int i = 0; i < actions.size(); ++i)
            if (!actions[i]->done()) return false;
        return true;
    }

    void setExecTime(float) {
        cout << "warning: doesn't make sense to set the execution time of an action chain" << endl;
    }

    void reset() {
        Action::reset();
        for (int i = 0; i < actions.size(); ++i)
            actions[i]->reset();
    }

    void step(float dt) {
        if (done()) return;

        // execute the next action that hasn't finished yet
        int nextNotDone = 0;
        for ( ; nextNotDone < actions.size(); ++nextNotDone)
            if (!actions[nextNotDone]->done()) break;
        if (nextNotDone >= actions.size()) {
            setDone(true);
            return;
        }

        actions[nextNotDone]->step(dt);
    }
};

class RobotInterpAction : public Action {
    RaveRobotObject::Ptr robot;
    vector<int> indices;
    vector<dReal> startvals, endvals;

public:
    RobotInterpAction(RaveRobotObject::Ptr robot_) : robot(robot_) { }

    void setIndices(const vector<int> &v) {
        indices.assign(v.begin(), v.end());
    }
    void setStartVals(const vector<dReal> &v) {
        startvals.assign(v.begin(), v.end());
    }
    void setEndVals(const vector<dReal> &v) {
        endvals.assign(v.begin(), v.end());
    }

    void step(float dt) {
        if (done()) return;
        stepTime(dt);
        const float a = fracElapsed();
        // interpolate each joint value
        vector<dReal> interpvals(startvals.size());
        for (int i = 0; i < interpvals.size(); ++i)
            interpvals[i] = (1.-a)*startvals[i] + a*endvals[i];
        robot->setDOFValues(indices, interpvals);
    }
};

class ManipIKInterpAction : public RobotInterpAction {
    RaveRobotObject::Ptr robot;
    RaveRobotObject::Manipulator::Ptr manip;

    float normJointVal(dReal f, dReal g) {
        if (g < f)
            return g + 2*M_PI*round((f - g) / 2/M_PI);
        return g - 2*M_PI*round((g - f) / 2/M_PI);
    }

    btTransform targetTrans;
    bool relative;

    void calcEndVals() {
        if (relative)
            targetTrans = targetTrans * manip->getTransform();

        vector<dReal> currvals;
        robot->robot->SetActiveDOFs(manip->manip->GetArmIndices());
        robot->robot->GetActiveDOFValues(currvals);
        setStartVals(currvals);

        vector<dReal> newvals;
        cout << "solving ik for transform: " << targetTrans.getOrigin().x() << ' ' << targetTrans.getOrigin().y() << ' ' << targetTrans.getOrigin().z() << '\t'  
            << targetTrans.getRotation().x() << ' ' << targetTrans.getRotation().y() << ' ' << targetTrans.getRotation().z() << ' ' << targetTrans.getRotation().w() << endl;
        if (!manip->solveIK(targetTrans, newvals)) {
            cout << "could not solve ik" << endl;
            setDone(true);
        } else {
            BOOST_ASSERT(newvals.size() == currvals.size());
            // round new joint vals to nearest multiple of pi
            // that the old vals were, so we don't get
            // unnecessary rotation
            for (int i = 0; i < newvals.size(); ++i)
                newvals[i] = normJointVal(currvals[i], newvals[i]);
            setEndVals(newvals);
        }
    }

public:
    ManipIKInterpAction(RaveRobotObject::Ptr robot_,
                        RaveRobotObject::Manipulator::Ptr manip_) :
        robot(robot_), manip(manip_),
        targetTrans(btTransform::getIdentity()), relative(false),
        RobotInterpAction(robot_) { }

    // sets the manipulator transform
    void setTargetTrans(const btTransform &t) {
        setIndices(manip->manip->GetArmIndices());
        targetTrans = t;
    }

    void step(float dt) {
        if (timeElapsed == 0)
            calcEndVals();
        RobotInterpAction::step(dt);
    }

    // sets the transform of the tip of the fingers
    void setPR2TipTargetTrans(const btTransform &t) {
        static const btTransform MANIP_TO_TIP =
            btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, -0.005)*METERS);
        setTargetTrans(t * MANIP_TO_TIP);
    }

    void setRelativeTrans(const btTransform &t) {
        relative = true;
        setTargetTrans(t);
    }
};

class PR2SoftBodyGripperAction : public Action {
    RaveRobotObject::Ptr robot;
    OpenRAVE::RobotBase::ManipulatorPtr manip;
    PR2SoftBodyGripper sbgripper;

    dReal startVal, endVal;
    vector<int> indices;
    vector<dReal> vals;

    // the target softbody
    btSoftBody *psb;

public:
    typedef boost::shared_ptr<PR2SoftBodyGripperAction> Ptr;
    PR2SoftBodyGripperAction(RaveRobotObject::Ptr robot_, OpenRAVE::RobotBase::ManipulatorPtr manip, bool leftGripper) :
        robot(robot_),
        sbgripper(robot_, manip, leftGripper),
        indices(manip->GetGripperIndices()),
        vals(1, 0)
    {
        sbgripper.setGrabOnlyOnContact(true);
        if (indices.size() != 1)
            cout << "WARNING: more than one gripper DOF; just choosing first one" << endl;
        setCloseAction();
    }

    void setEndpoints(dReal start, dReal end) { startVal = start; endVal = end; }
    dReal getCurrDOFVal() const {
        vector<dReal> v;
        robot->robot->GetDOFValues(v);
        return v[indices[0]];
    }
    void setOpenAction() { setEndpoints(getCurrDOFVal(), PR2_GRIPPER_OPEN_VAL); }
    void setCloseAction() { setEndpoints(getCurrDOFVal(), PR2_GRIPPER_CLOSED_VAL); }
    void toggleAction() {
        if (endVal == PR2_GRIPPER_CLOSED_VAL)
            setOpenAction();
        else if (endVal == PR2_GRIPPER_OPEN_VAL)
            setCloseAction();
    }

    // Must be called before the action is run!
    void setTarget(btSoftBody *psb_) {
        psb = psb_;
        sbgripper.setTarget(psb_);
    }

    void releaseAllAnchors() {
        psb->m_anchors.clear();
    }

    void reset() {
        Action::reset();
        releaseAllAnchors();
    }

    void step(float dt) {
        if (done()) return;
        stepTime(dt);

        float frac = fracElapsed();
        vals[0] = (1.f - frac)*startVal + frac*endVal;
        robot->setDOFValues(indices, vals);

        if (vals[0] == PR2_GRIPPER_CLOSED_VAL)
            sbgripper.grab();
    }
};

class GripperOpenCloseAction : public Action {
    RaveRobotObject::Ptr robot;
    OpenRAVE::RobotBase::ManipulatorPtr manip;

    dReal startVal, endVal;
    vector<int> indices;
    vector<dReal> vals;

    dReal getCurrDOFVal() const {
        vector<dReal> v;
        robot->robot->GetDOFValues(v);
        return v[indices[0]];
    }

public:
    typedef boost::shared_ptr<PR2SoftBodyGripperAction> Ptr;
    GripperOpenCloseAction(RaveRobotObject::Ptr robot_, OpenRAVE::RobotBase::ManipulatorPtr manip, bool open) :
        robot(robot_),
        indices(manip->GetGripperIndices()),
        vals(1, 0)
    {
        if (indices.size() != 1)
            cout << "WARNING: more than one gripper DOF; just choosing first one" << endl;
        endVal = open ? PR2_GRIPPER_OPEN_VAL : PR2_GRIPPER_CLOSED_VAL;
    }

    void step(float dt) {
        if (done()) return;

        if (timeElapsed == 0)
            startVal = getCurrDOFVal();

        stepTime(dt);

        float frac = fracElapsed();
        vals[0] = (1.f - frac)*startVal + frac*endVal;
        robot->setDOFValues(indices, vals);
    }
};

// orients the gripper for grabbing a cloth node
//class PR2OrientGripperAction : public ManipIKInterpAction {
//    PR2OrientGripperAction
//};

static const btQuaternion PR2_GRIPPER_INIT_ROT(0., 0.7071, 0., 0.7071);
static const btQuaternion GRIPPER_TO_VERTICAL_ROT(btVector3(1, 0, 0), M_PI/2);
static const btVector3 PR2_GRIPPER_INIT_ROT_DIR(1, 0, 0);
static const btScalar MOVE_BEHIND_DIST = 0.02;
static const btVector3 OFFSET(0, 0, 0.0/*5*/); // don't sink through table
static const btScalar ANGLE_DOWN_HEIGHT = 0.05;
static const btScalar LOWER_INTO_TABLE = -0.01;
class GraspClothNodeAction : public ActionChain {
    RaveRobotObject::Ptr robot;
    RaveRobotObject::Manipulator::Ptr manip;
    btSoftBody *cloth;
    const int node;
    btVector3 dir;

    // pos = desired manip transform origin
    // dir = direction vector that the manipulator should point to
    btTransform transFromDir(const btVector3 &dir, const btVector3 &pos) {
        btVector3 cross = PR2_GRIPPER_INIT_ROT_DIR.cross(dir);
        btScalar angle = dir.angle(PR2_GRIPPER_INIT_ROT_DIR);
        if (btFuzzyZero(cross.length2()))
            cross = btVector3(1, 0, 0); // arbitrary axis
        btQuaternion q(cross, angle);
        return btTransform(q * GRIPPER_TO_VERTICAL_ROT * PR2_GRIPPER_INIT_ROT, pos);
    }

public:
    GraspClothNodeAction(RaveRobotObject::Ptr robot_, RaveRobotObject::Manipulator::Ptr manip_, btSoftBody *cloth_, int node_, const btVector3 &dir_) :
        robot(robot_), manip(manip_), cloth(cloth_), node(node_), dir(dir_) {

        dir.setZ(0); // only look at vector on the x-y plane
        if (!btFuzzyZero(dir.length2())) {
            dir.normalize();

            GripperOpenCloseAction *openGripper = new GripperOpenCloseAction(robot, manip->manip, true);
            GripperOpenCloseAction *closeGripper = new GripperOpenCloseAction(robot, manip->manip, false);

            ManipIKInterpAction *positionGrasp = new ManipIKInterpAction(robot, manip);
            btVector3 v(cloth->m_nodes[node].m_x);
            // move the gripper a few cm behind node, angled slightly down
            btVector3 dir2 = btVector3(dir.x(), dir.y(), -ANGLE_DOWN_HEIGHT * METERS).normalize();
            positionGrasp->setPR2TipTargetTrans(transFromDir(dir2, v - dir2*MOVE_BEHIND_DIST*METERS + OFFSET*METERS));

            ManipIKInterpAction *lowerIntoTable = new ManipIKInterpAction(robot, manip);
            lowerIntoTable->setRelativeTrans(btTransform(btQuaternion(0,0,0,1), btVector3(0, 0, LOWER_INTO_TABLE*METERS)));

            ManipIKInterpAction *moveForward = new ManipIKInterpAction(robot, manip);
            moveForward->setRelativeTrans(btTransform(btQuaternion(0, 0, 0, 1),
                       dir * MOVE_BEHIND_DIST*2 * METERS));

            PR2SoftBodyGripper sbgripper(robot, manip->manip, true); // TODO: leftGripper flag
            sbgripper.setGrabOnlyOnContact(true);
            sbgripper.setTarget(cloth);
            FunctionAction *setAnchors = new FunctionAction(boost::bind(&PR2SoftBodyGripper::grab, sbgripper));
            *this << openGripper << positionGrasp << lowerIntoTable << moveForward << closeGripper << setAnchors;
        }
    }
};

#endif // __CLOTHGRASPING_H__
