#ifndef __CLOTHGRASPING_H__
#define __CLOTHGRASPING_H__

#include "simulation/environment.h"
#include "robots/pr2.h"

class PR2SoftBodyGripperAction : public Action {
    RaveRobotKinematicObject::Ptr robot;
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
    PR2SoftBodyGripperAction(RaveRobotKinematicObject::Ptr robot_, OpenRAVE::RobotBase::ManipulatorPtr manip, bool leftGripper) :
        robot(robot_),
        sbgripper(robot_, manip, leftGripper),
        indices(manip->GetGripperIndices()),
        vals(1, 0)
    {
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
