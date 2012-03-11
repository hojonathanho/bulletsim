#ifndef __CLOTHGRASPING_H__
#define __CLOTHGRASPING_H__

#include "simulation/environment.h"
#include "simulation/openravesupport.h"
#include "robots/pr2.h"

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
            return g + M_PI*round((f - g) / M_PI);
        return g - M_PI*round((g - f) / M_PI);
    }

public:
    ManipIKInterpAction(RaveRobotObject::Ptr robot_,
                        RaveRobotObject::Manipulator::Ptr manip_) :
        robot(robot_), manip(manip_), RobotInterpAction(robot_) { }

    void setTargetTrans(const btTransform &t) {
        setIndices(manip->manip->GetArmIndices());

        vector<dReal> currvals;
        robot->robot->SetActiveDOFs(manip->manip->GetArmIndices());
        robot->robot->GetActiveDOFValues(currvals);
        setStartVals(currvals);

        vector<dReal> newvals;
        if (!manip->solveIK(t, newvals)) {
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
};

class PR2SoftBodyGripperAction : public Action {
    RaveRobotObject::Ptr robot;
    OpenRAVE::RobotBase::ManipulatorPtr manip;
    PR2SoftBodyGripper sbgripper;

    dReal startVal, endVal;
    vector<int> indices;
    vector<dReal> vals;

    // min/max gripper dof vals
    static const float CLOSED_VAL = 0.03f, OPEN_VAL = 0.54f;

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
    void setOpenAction() { setEndpoints(getCurrDOFVal(), OPEN_VAL); }
    void setCloseAction() { setEndpoints(getCurrDOFVal(), CLOSED_VAL); }
    void toggleAction() {
        if (endVal == CLOSED_VAL)
            setOpenAction();
        else if (endVal == OPEN_VAL)
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

        if (vals[0] == CLOSED_VAL)
            sbgripper.grab();
    }
};

#endif // __CLOTHGRASPING_H__
