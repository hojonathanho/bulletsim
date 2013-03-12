#pragma once
#include "RavenGrabMonitor.h"
#include "jointRecorder.h"

class RavensRigidBodyGripperAction : public Action {

    RaveRobotObject::Manipulator::Ptr manip;
    vector<dReal> startVals;
    dReal endVal;
    vector<int> indices;
    vector<dReal> vals;

    // manages the grabs for the ravens
    RavensGrabMonitor::Ptr grabMonitor;

    string arm;
    // jointRecorder to push in "grab" or "release" lines into file
    jointRecorder::Ptr jr;

    // min/max gripper dof vals
    static const float CLOSED_VAL = 0.0f, OPEN_VAL = 0.25f;

public:
    typedef boost::shared_ptr<RavensRigidBodyGripperAction> Ptr;
    RavensRigidBodyGripperAction(RaveRobotObject::Manipulator::Ptr manip_,
                           	     const string &leftFingerName,
                           	     const string &rightFingerName,
                           	     btDynamicsWorld* world_,
                           	     float time, Scene &s, char * _arm, jointRecorder * _jr) :
                           	     Action(time), manip(manip_), vals(2, 0), jr (_jr), arm(_arm) {

    	grabMonitor.reset(new RavensGrabMonitor(manip_, world_, leftFingerName, rightFingerName, s));
    	manip->manip->GetChildDOFIndices(indices);
    	setCloseAction();
    }

    ~RavensRigidBodyGripperAction() {
        grabMonitor->release();
    }

    btVector3 getVec (bool left) {
    	btVector3 z = grabMonitor->getToolDirection();
    	return grabMonitor->getInnerPt(left) + z*0.015*METERS;
    }

    btVector3 getClosingDirection(bool left) {return grabMonitor->getClosingDirection(left);}

    btTransform getTfm (bool left) {
    	return grabMonitor->getInverseFingerTfm(left).inverse();
    }

    void setEndpoints(vector<dReal> start, dReal end) { startVals = start; endVal = end; }

    vector<dReal> getCurrDOFVal() const {
        vector<dReal> v;
        manip->robot->robot->GetDOFValues(v, indices);
        return v;
    }

    void setTargets(vector<BulletObject::Ptr>& bodies) {grabMonitor->setBodies(bodies);}
    void setOpenAction()  {
    	setEndpoints(getCurrDOFVal(), OPEN_VAL);
    	string message = "release ";
    	jr->addMessageToFile(message.append(arm));
    }
    void setCloseAction() { setEndpoints(getCurrDOFVal(), CLOSED_VAL); }
    void toggleAction() {
        if (endVal == CLOSED_VAL)
            setOpenAction();
        else if (endVal == OPEN_VAL)
            setCloseAction();
    }

    void reset() {
        Action::reset();
        grabMonitor->release();
    }

    void grab (float threshold=100) {grabMonitor->grab(threshold);}

    void step(float dt) {
        if (done()) return;

        // if there's a large force on the fingers
        // then we probably can't close any further
        if (endVal != OPEN_VAL) {
        	grab();
    		//jr->addMessageToFile(arm.append(" grab"));
        	if (grabMonitor->getNumGrabbed() > 0) {
                string message = "grab ";
        		jr->addMessageToFile(message.append(arm));
        		setDone(true);
        		return;
        	}
        }

        stepTime(dt);

        float frac = fracElapsed();
        vals[0] = (1.f - frac)*startVals[0] + frac*endVal;
        vals[1] = (1.f - frac)*startVals[1] + frac*-1*endVal;
        manip->robot->setDOFValues(indices, vals);
    }
};
