#ifndef __FL_GRASPINGACTIONS_IMPL_H__
#define __FL_GRASPINGACTIONS_IMPL_H__

#include "graspingactions.h"

#include "simulation/environment.h"
#include "simulation/openravesupport.h"
#include "robots/pr2.h"
#include "config_flattening.h"

#undef PR2_GRIPPER_OPEN_VAL
#undef PR2_GRIPPER_CLOSED_VAL
#define PR2_GRIPPER_OPEN_VAL 0.54f
#define PR2_GRIPPER_CLOSED_VAL 0.03f

// does nothing
class EmptyAction : public Action {
public:
    typedef boost::shared_ptr<EmptyAction> Ptr;
    EmptyAction() : Action() { setDone(true); }
    void step(float) {
        setDone(true);
    }
};

// an action that just runs a given function once
class FunctionAction : public Action {
private:
    boost::function<void(void)> fn;

public:
    typedef boost::shared_ptr<FunctionAction> Ptr;

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
    typedef boost::shared_ptr<ActionChain> Ptr;

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

class RobotJointInterpAction : public virtual Action {
    RaveRobotObject::Ptr robot;
    vector<int> indices;
    vector<dReal> startvals, endvals;

public:
    typedef boost::shared_ptr<RobotJointInterpAction> Ptr;

    RobotJointInterpAction(RaveRobotObject::Ptr robot_) : robot(robot_) { }

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

class FakeManipAction : public virtual Action {
    TelekineticGripper::Ptr gripper;
    btTransform start, dest;

public:
    FakeManipAction(TelekineticGripper::Ptr fakeGripper) : gripper(fakeGripper), start(btTransform::getIdentity()), dest(btTransform::getIdentity()) { }
    void setStart(const btTransform &start_) { start = start_; }
    void setDest(const btTransform &dest_) { dest = dest_; }

    void step(float dt) {
        if (done()) return;
        stepTime(dt);
        const float a = fracElapsed();
        btVector3 v = (1-a)*start.getOrigin() + a*dest.getOrigin();
        btQuaternion q = start.getRotation().slerp(dest.getRotation(), a);
        gripper->setTransform(btTransform(q, v));
    }
};

class ManipMoveAction : public virtual Action {
protected:
    enum MoveMode {
        MOVE_TIP_ABSOLUTE,
        MOVE_RELATIVE,
        MOVE_ROT_ONLY,
        MOVE_ORIGIN_ONLY
    } mode;

public:
    typedef boost::shared_ptr<ManipMoveAction> Ptr;

    MoveMode getMode() const { return mode; }

    virtual void setTargetTrans(const btTransform &t) = 0;

    // sets the transform of the tip of the fingers
    virtual void setPR2TipTargetTrans(const btTransform &t) {
        static const btTransform MANIP_TO_TIP =
            btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, -0.005)*METERS);
        setTargetTrans(t * MANIP_TO_TIP);
    }

    // t is an offset transformation (will be applied to the current transform when run)
    virtual void setRelativeTrans(const btTransform &t) {
        mode = MOVE_RELATIVE;
        setTargetTrans(t);
    }

    // sets the rotation only and keeps the current origin
    virtual void setRotOnly(const btQuaternion &rot) {
        mode = MOVE_ROT_ONLY;
        setTargetTrans(btTransform(rot, btVector3(0, 0, 0)));
    }

    // sets the origin only and keeps the current rotation
    virtual void setOriginOnly(const btVector3 &origin) {
        mode = MOVE_ORIGIN_ONLY;
        setTargetTrans(btTransform(btQuaternion::getIdentity(), origin));
    }
};

class ManipIKInterpAction : public RobotJointInterpAction, public ManipMoveAction {
    RaveRobotObject::Ptr robot;
    RaveRobotObject::Manipulator::Ptr manip;

    // rounds new joint vals to nearest multiple of pi (or 2pi)
    // that the old vals were, so we don't get unnecessary rotation
    static float normJointVal(dReal f, dReal g, bool halfRotSym) {
        const float a = halfRotSym ? M_PI : 2*M_PI;
        if (g < f)
            return g + a*round((f - g)/a);
        return g - a*round((g - f)/a);
    }

    btTransform targetTrans;

    void calcEndVals() {
        switch (getMode()) {
        case MOVE_RELATIVE:
            targetTrans = targetTrans * manip->getTransform();
            break;
        case MOVE_ROT_ONLY:
            targetTrans.setOrigin(manip->getTransform().getOrigin());
            break;
        case MOVE_ORIGIN_ONLY:
            targetTrans.setRotation(manip->getTransform().getRotation());
            break;
        }

        vector<dReal> currvals;
        robot->robot->SetActiveDOFs(manip->manip->GetArmIndices());
        robot->robot->GetActiveDOFValues(currvals);
        setStartVals(currvals);

        vector<dReal> newvals;
       // cout << "solving ik for transform: " << targetTrans.getOrigin().x() << ' ' << targetTrans.getOrigin().y() << ' ' << targetTrans.getOrigin().z() << '\t'  
       //     << targetTrans.getRotation().x() << ' ' << targetTrans.getRotation().y() << ' ' << targetTrans.getRotation().z() << ' ' << targetTrans.getRotation().w() << endl;
        if (!manip->solveIK(targetTrans, newvals)) {
            RobotJointInterpAction::setDone(true);
            throw GraspingActionFailed("could not solve ik");
        } else {
            BOOST_ASSERT(newvals.size() == currvals.size());
            for (int i = 0; i < newvals.size(); ++i) {
                // if the joint is the wrist roll, then round to nearest
                // multiple of pi, not 2pi
                int index = manip->manip->GetArmIndices()[i];
                bool halfRotSym = index == 21 || index == 33;
                newvals[i] = normJointVal(currvals[i], newvals[i], halfRotSym);
            }
            setEndVals(newvals);
        }
    }

public:
    typedef boost::shared_ptr<ManipIKInterpAction> Ptr;

    ManipIKInterpAction(RaveRobotObject::Ptr robot_,
                        RaveRobotObject::Manipulator::Ptr manip_) :
        robot(robot_), manip(manip_),
        targetTrans(btTransform::getIdentity()),
        RobotJointInterpAction(robot_) { }

    // sets the manipulator transform
    void setTargetTrans(const btTransform &t) {
        setIndices(manip->manip->GetArmIndices());
        targetTrans = t;
    }

    void step(float dt) {
        if (RobotJointInterpAction::timeElapsed == 0)
            calcEndVals();
        RobotJointInterpAction::step(dt);
    }
};

class FakeManipMoveAction : public FakeManipAction, public ManipMoveAction {
    btTransform targetTrans;
    TelekineticGripper::Ptr fgripper;

    void calcTraj() {
        btTransform currTrans = fgripper->getTransform();
        switch (getMode()) {
        case MOVE_RELATIVE:
            targetTrans = targetTrans * currTrans;
            break;
        case MOVE_ROT_ONLY:
            targetTrans.setOrigin(currTrans.getOrigin());
            break;
        case MOVE_ORIGIN_ONLY:
            targetTrans.setRotation(currTrans.getRotation());
            break;
        }
        setStart(currTrans);
        setDest(targetTrans);
    }

public:
    FakeManipMoveAction(TelekineticGripper::Ptr fgripper_) : fgripper(fgripper_), targetTrans(btTransform::getIdentity()), FakeManipAction(fgripper_) { }
    void setTargetTrans(const btTransform &t) { targetTrans = t; }
    void step(float dt) {
        if (FakeManipAction::timeElapsed == 0)
            calcTraj();
        FakeManipAction::step(dt);
    }
};

class GenPR2SoftGripperAction : public Action {
    RaveRobotObject::Ptr robot;
    OpenRAVE::RobotBase::ManipulatorPtr manip;
    GenPR2SoftGripper::Ptr sbgripper;

    dReal startVal, endVal;
    vector<int> indices;
    vector<dReal> vals;

    // the target softbody
    BulletSoftObject::Ptr sb;

public:
    typedef boost::shared_ptr<GenPR2SoftGripperAction> Ptr;

    GenPR2SoftGripperAction(RaveRobotObject::Ptr robot_, OpenRAVE::RobotBase::ManipulatorPtr manip, GenPR2SoftGripper::Ptr sbgripper_) :
        robot(robot_),
        sbgripper(sbgripper_),
        indices(manip->GetGripperIndices()),
        vals(1, 0)
    {
        if (indices.size() != 1)
            cout << "WARNING: more than one gripper DOF; just choosing first one" << endl;
    }

    void setEndpoints(dReal start, dReal end) { startVal = start; endVal = end; }
    dReal getCurrDOFVal() const {
        vector<dReal> v;
        robot->robot->GetDOFValues(v);
        return v[indices[0]];
    }
    void setOpenAction() { setEndpoints(getCurrDOFVal(), PR2_GRIPPER_OPEN_VAL); }
    void setCloseAction() { setEndpoints(getCurrDOFVal(), PR2_GRIPPER_CLOSED_VAL); }

    // Must be called before the action is run!
    void setTarget(BulletSoftObject::Ptr sb_) {
        sb = sb_;
        sbgripper->setTarget(sb_);
    }

    void step(float dt) {
        if (done()) return;

        if (timeElapsed == 0) {
            if (sbgripper->isGrabbing()) {
                setOpenAction();
                sbgripper->releaseAllAnchors();
            } else {
                setCloseAction();
            }
        }

        stepTime(dt);

        float frac = fracElapsed();
        vals[0] = (1.f - frac)*startVal + frac*endVal;
        robot->setDOFValues(indices, vals);

        if (vals[0] == PR2_GRIPPER_CLOSED_VAL)
            sbgripper->grab();
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
    typedef boost::shared_ptr<GripperOpenCloseAction> Ptr;

    GripperOpenCloseAction(RaveRobotObject::Ptr robot_, OpenRAVE::RobotBase::ManipulatorPtr manip, bool open) :
        robot(robot_),
        indices(manip->GetGripperIndices()),
        vals(1, 0)
    {
        if (indices.size() != 1)
            cout << "WARNING: more than one gripper DOF; just choosing first one" << endl;
        setOpen(open);
    }

    void setOpen(bool open) {
        endVal = open ? PR2_GRIPPER_OPEN_VAL : PR2_GRIPPER_CLOSED_VAL;
    }

    void step(float dt) {
        if (done()) return;

        if (timeElapsed == 0)
            startVal = getCurrDOFVal();

        if (startVal == endVal) {
            setDone(true);
            return;
        }

        stepTime(dt);

        float frac = fracElapsed();
        vals[0] = (1.f - frac)*startVal + frac*endVal;
        robot->setDOFValues(indices, vals);
    }
};

static const btQuaternion PR2_GRIPPER_INIT_ROT(0., 0.7071, 0., 0.7071);
static const btQuaternion GRIPPER_TO_VERTICAL_ROT(btVector3(1, 0, 0), M_PI/2);
static const btQuaternion GRIPPER_DOWN_ROT(btVector3(0, 1, 0), M_PI/2);
static const btVector3 PR2_GRIPPER_INIT_ROT_DIR(1, 0, 0);
static const btScalar MOVE_BEHIND_DIST = 0;//0.02;
static const btScalar MOVE_FORWARD_DIST = 0.01;
static const btVector3 SCOOP_OFFSET(0, 0, 0.02); // don't sink through table
//static const btScalar ANGLE_DOWN_HEIGHT = 0.03;
static const btQuaternion ANGLE_DOWN_ROT(btVector3(0, 1, 0), 45*M_PI/180);
static const btScalar LOWER_INTO_TABLE = -0.01;
static const btScalar LIFT_DIST = 0.05;
class GraspClothNodeAction : public ActionChain {
    RaveRobotObject::Ptr robot;
    GenManip::Ptr gmanip;
    BulletSoftObject::Ptr sb;
    const int node;
    btVector3 dir;
    GenPR2SoftGripper::Ptr sbgripper;

    // pos = desired manip transform origin
    // dir = direction vector that the manipulator should point to
    btTransform transFromDir(const btVector3 &dir, const btVector3 &pos, bool angleDown) {
        btVector3 cross = PR2_GRIPPER_INIT_ROT_DIR.cross(dir);
        btScalar angle = dir.angle(PR2_GRIPPER_INIT_ROT_DIR);
        if (btFuzzyZero(cross.length2()))
            cross = btVector3(0, 0, 1);
        btQuaternion q(btQuaternion(cross, angle)
                * (angleDown ? ANGLE_DOWN_ROT : btQuaternion::getIdentity())
                * GRIPPER_TO_VERTICAL_ROT * PR2_GRIPPER_INIT_ROT);
        return btTransform(q, pos);
    }

    ManipMoveAction::Ptr createMoveAction() {
        switch (gmanip->type) {
        case GenManip::TYPE_FAKE:
            return FakeManipMoveAction::Ptr(new FakeManipMoveAction(gmanip->asFake()));
        case GenManip::TYPE_IK:
            return ManipIKInterpAction::Ptr(new ManipIKInterpAction(robot, gmanip->asIKManip()));
        }
        cout << "error: gen manip type " << gmanip->type << " not recognized by GraspClothNodeAction" << endl;
        return ManipMoveAction::Ptr();
    };

    void init() {
        if (btFuzzyZero(dir.length2())) {
            cout << "WARNING: creating GraspClothNodeAction with dir length zero" << endl;
            return;
        }

        dir.normalize();

        // if dir is a small angle from the table, use special scooping motion
        bool scoop = false;
        btVector3 dirProjTable = dir; dirProjTable.setZ(0); dirProjTable.normalize();
        btScalar angleWithTable = abs(dir.angle(dirProjTable));
        if (angleWithTable < M_PI/4) {
            dir = dirProjTable;
            scoop = true;
        }

        GripperOpenCloseAction::Ptr openGripper(new GripperOpenCloseAction(robot, gmanip->baseManip()->manip, true));
        GripperOpenCloseAction::Ptr closeGripper(new GripperOpenCloseAction(robot, gmanip->baseManip()->manip, false));

        ManipMoveAction::Ptr positionGrasp(createMoveAction());
        btVector3 v(sb->softBody->m_nodes[node].m_x);
        btTransform graspTrans = scoop ? transFromDir(dir, v + dir*0.03*METERS + SCOOP_OFFSET*METERS, true)
                                       : transFromDir(dir, v - dir*MOVE_BEHIND_DIST*METERS, false);
        positionGrasp->setPR2TipTargetTrans(graspTrans);

        ManipMoveAction::Ptr moveAboveNode(createMoveAction());
        btTransform moveAboveTrans(GRIPPER_DOWN_ROT * PR2_GRIPPER_INIT_ROT,
                graspTrans.getOrigin() + btVector3(0, 0, 2*LIFT_DIST*METERS));
        moveAboveNode->setPR2TipTargetTrans(moveAboveTrans);

        ManipMoveAction::Ptr moveForward(createMoveAction());
        moveForward->setRelativeTrans(btTransform(btQuaternion(0, 0, 0, 1),
                   dir * (MOVE_BEHIND_DIST+(scoop?0:MOVE_FORWARD_DIST)) * METERS));
        if (scoop) moveForward->setExecTime(0);

        FunctionAction::Ptr releaseAnchors(new FunctionAction(boost::bind(&GenPR2SoftGripper::releaseAllAnchors, sbgripper)));
        FunctionAction::Ptr setAnchors(new FunctionAction(boost::bind(&GenPR2SoftGripper::grab, sbgripper)));

        // make gripper face down, while keeping rotation
        ManipMoveAction::Ptr orientGripperDown(createMoveAction());
        btVector3 facingDir = btTransform(graspTrans.getRotation() * PR2_GRIPPER_INIT_ROT.inverse(), btVector3(0, 0, 0)) * PR2_GRIPPER_INIT_ROT_DIR;
        btScalar angleFromVert = facingDir.angle(btVector3(0, 0, -1));
        btVector3 cross = facingDir.cross(btVector3(0, 0, -1));
        if (btFuzzyZero(cross.length2())) cross = btVector3(1, 0, 0); // arbitrary axis
        orientGripperDown->setRotOnly(btQuaternion(cross, angleFromVert) * graspTrans.getRotation());

        ManipMoveAction::Ptr moveUp(createMoveAction());
        moveUp->setRelativeTrans(btTransform(btQuaternion(0, 0, 0, 1),
                    btVector3(0, 0, 0.01*METERS)));

        ManipMoveAction::Ptr moveToNeutralHeight(createMoveAction());
        moveToNeutralHeight->setRelativeTrans(btTransform(btQuaternion(0, 0, 0, 1),
                    btVector3(0, 0, LIFT_DIST * METERS)));

        *this << releaseAnchors << openGripper
            << moveAboveNode << positionGrasp
            << moveForward << closeGripper << setAnchors
            << moveUp << orientGripperDown << moveToNeutralHeight;
    }

public:
    typedef boost::shared_ptr<GraspClothNodeAction> Ptr;

#if 0
    // constructor for using real robot
    GraspClothNodeAction(
            RaveRobotObject::Ptr robot_,
            RaveRobotObject::Manipulator::Ptr manip_,
            GenPR2SoftGripper::Ptr sbgripper_,
            BulletSoftObject::Ptr sb_, int node_,
            const btVector3 &dir_) :
        robot(robot_), manip(manip_), sbgripper(sbgripper_), sb(sb_), node(node_), dir(dir_) {
        init();
    }

    // constructor for using fake manipulator
    GraspClothNodeAction(
            RaveRobotObject::Ptr robot_,
            TelekineticGripper::Ptr fakeManip_,
            GenPR2SoftGripper::Ptr sbgripper_,
            BulletSoftObject::Ptr sb_, int node_,
            const btVector3 &dir_) :
        robot(robot_), manip(fakeManip_->m_manip), fakeManip(fakeManip_),
        sbgripper(sbgripper_),
        sb(sb_), node(node_), dir(dir_)
    {
        init();
    }
#endif
    // general constructor with GenManip
    GraspClothNodeAction(
            RaveRobotObject::Ptr robot_,
            GenManip::Ptr genmanip, 
            GenPR2SoftGripper::Ptr sbgripper_,
            BulletSoftObject::Ptr sb_, int node_,
            const btVector3 &dir_) :
        robot(robot_), gmanip(genmanip), sbgripper(sbgripper_), sb(sb_), node(node_), dir(dir_) {
        init();
    }
};

#endif // __FL_GRASPINGACTIONS_IMPL_H__
