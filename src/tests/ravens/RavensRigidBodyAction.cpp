#include "RavensRigidBodyAction.h"
#include "CustomScene.h"

RavensRigidBodyGripperAction::RavensRigidBodyGripperAction(RaveRobotObject::Manipulator::Ptr manip_,
		const string &leftFingerName,
		const string &rightFingerName,
		btDynamicsWorld* world_,
		float time, CustomScene &_s, char * _arm, jointRecorder * _jr) :
		s(_s), Action(time), manip(manip_), vals(2, 0),
		jr (_jr), arm(_arm), hasPeg(false) {

	grabMonitor.reset(new RavensGrabMonitor(manip_, world_, arm[0], leftFingerName, rightFingerName, s));
	manip->manip->GetChildDOFIndices(indices);
	setCloseAction();
}


btVector3 RavensRigidBodyGripperAction::getVec (bool left) {
	btVector3 z = grabMonitor->getToolDirection();
	return grabMonitor->getInnerPt(left) + z*0.015*METERS;
}


btVector3 RavensRigidBodyGripperAction::getClosingDirection(bool left)
{return grabMonitor->getClosingDirection(left);}


btTransform RavensRigidBodyGripperAction::getTfm (bool left)
{return grabMonitor->getInverseFingerTfm(left).inverse();}


vector<dReal> RavensRigidBodyGripperAction::getCurrDOFVal() {
	vector<dReal> v;
	manip->robot->robot->GetDOFValues(v, indices);
	return v;
}

void RavensRigidBodyGripperAction::setOpenAction()  {
	if (hasPeg) s.togglePegFinger();
	setEndpoints(getCurrDOFVal(), OPEN_VAL);
	string message = "release ";
	jr->addMessageToFile(message.append(arm));
}


void RavensRigidBodyGripperAction::setCloseAction() {
	setEndpoints(getCurrDOFVal(), (hasPeg ? CLOSED_VAL_PEG :CLOSED_VAL));
	string message = "grab ";
	jr->addMessageToFile(message.append(arm));
}


void RavensRigidBodyGripperAction::toggleAction() {
	if (endVal == (hasPeg ? CLOSED_VAL_PEG :CLOSED_VAL))
		setOpenAction();
	else if (endVal == OPEN_VAL)
		setCloseAction();
}


void RavensRigidBodyGripperAction::grab (bool grabN, float threshold)
{grabMonitor->grab(grabN,threshold);}


void RavensRigidBodyGripperAction::step(float dt) {
	if (done()) return;

	// if there's a large force on the fingers
	// then we probably can't close any further
	if (endVal != OPEN_VAL) {
		grab(fracElapsed()  >= 0.5 ? true : false);
		//jr->addMessageToFile(arm.append(" grab"));
		if (grabMonitor->getNumGrabbed() > 0) {
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
