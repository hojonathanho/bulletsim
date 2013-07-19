#pragma once
#include "RavenGrabMonitor.h"

class CustomScene;

class RavensRigidBodyGripperAction : public Action {

    RaveRobotObject::Manipulator::Ptr manip;
    vector<dReal> startVals;
    dReal endVal;
    vector<int> indices;
    vector<dReal> vals;

    // manages the grabs for the ravens
    RavensGrabMonitor::Ptr grabMonitor;

    CustomScene &s;
    // Gripper has peg
    bool hasPeg;

    string arm;


    // min/max gripper dof vals
    static const float CLOSED_VAL = 0.0f, CLOSED_VAL_PEG = 0.015f, OPEN_VAL = 0.30f;

public:
    typedef boost::shared_ptr<RavensRigidBodyGripperAction> Ptr;
    RavensRigidBodyGripperAction(RaveRobotObject::Manipulator::Ptr manip_,
                           	     const string &leftFingerName,
                           	     const string &rightFingerName,
                           	     btDynamicsWorld* world_,
                           	     float time, CustomScene &_s, char * _arm);

    void setPeg (bool _hasPeg) {hasPeg = _hasPeg;}

    ~RavensRigidBodyGripperAction() {grabMonitor->release();}

    btVector3 getVec (bool left);
    btVector3 getClosingDirection(bool left);
    btTransform getTfm (bool left);

    void setEndpoints(vector<dReal> start, dReal end) { startVals = start; endVal = end; }

    vector<dReal> getCurrDOFVal();

    void setTargets(vector<CompoundObject<BulletObject>::Ptr>& bodies) {grabMonitor->setBodies(bodies);}
    void setOpenAction();
    void setCloseAction();
    void toggleAction();
    void reset() {
        Action::reset();
        grabMonitor->release();
    }

    void grab (bool grabN, float threshold=100);
    void step(float dt);
};
