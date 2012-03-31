#include "graspingactions.h"
#include "graspingactions_impl.h"

Action::Ptr GraspingActionSpec::createAction() {
    stringstream ss;
    ss << specstr;
    string op; ss >> op; // consume type
    switch (type) {
    case GRAB: return createGrabAction(ss); break;
    case RELEASE: return createReleaseAction(ss); break;
    case MOVE: return createMoveAction(ss); break;
    }
    return Action::Ptr();
};

/*
void GraspingActionSpec::getSuccessors(vector<GraspingActionSpec> &out) {
    switch (type) {
    case GRAB: return succGrabAction(out); break;
    case RELEASE: return succReleaseAction(out); break;
    case MOVE: return succMoveAction(out); break;
    }
}*/

void GraspingActionSpec::readType() {
    stringstream ss;
    ss << specstr;
    string op; ss >> op;
    if (op == "grab")
        type = GRAB;
    else if (op == "release")
        type = RELEASE;
    else if (op == "move")
        type = MOVE;
    else if (op == "none")
        type = NONE;
    else {
        cout << "error: unknown action op " << op << " (spec:" << specstr << ")" << endl;
        type = NONE;
    }
}

Action::Ptr GraspingActionSpec::createGrabAction(stringstream &ss) {
    // format: grab <node index> <approach vec x> <approach vec y> <approach vec z>
    int nodeidx; ss >> nodeidx;
    btScalar vx, vy, vz; ss >> vx >> vy >> vz;
    return GraspClothNodeAction::Ptr(new GraspClothNodeAction(
                ctx.robot, ctx.manip, ctx.sbgripper, ctx.sb,
                nodeidx, btVector3(vx, vy, vz)));
}

Action::Ptr GraspingActionSpec::createReleaseAction(stringstream &ss) {
    // format: release
    FunctionAction::Ptr releaseAnchors(new FunctionAction(
                boost::bind(&PR2SoftBodyGripper::releaseAllAnchors, ctx.sbgripper)));
    GripperOpenCloseAction::Ptr openGripper(new GripperOpenCloseAction(
                ctx.robot, ctx.manip->manip, true));
    ActionChain::Ptr a(new ActionChain);
    *a << releaseAnchors << openGripper;
    return a;
}

Action::Ptr GraspingActionSpec::createMoveAction(stringstream &ss) {
    // format: move dx dy dz
    btScalar dx, dy, dz; ss >> dx >> dy >> dz;
    ManipIKInterpAction::Ptr a(new ManipIKInterpAction(ctx.robot, ctx.manip));
    a->setRelativeTrans(btTransform(btQuaternion::getIdentity(), btVector3(dx, dy, dz)));
    return a;
}
