#ifndef __FL_GRASPINGACTIONS_H__
#define __FL_GRASPINGACTIONS_H__

// Provides a friendly interface over the actions defined in graspingactions_impl.h

#include "simulation/environment.h"
#include "simulation/openravesupport.h"
#include "simulation/softbodies.h"
#include "robots/pr2.h"

struct GraspingActionContext {
    RaveRobotObject::Ptr robot;
    RaveRobotObject::Manipulator::Ptr manip;
    PR2SoftBodyGripper::Ptr sbgripper;
    BulletSoftObject::Ptr sb;
};

struct GraspingActionSpec {
    typedef boost::shared_ptr<GraspingActionSpec> Ptr;

    enum Type {
        NONE = 0,
        GRAB,
        RELEASE,
        MOVE, // relative movement
        MOVE_ABSOLUTE
    } type;

    const string specstr;
    GraspingActionContext ctx;

    GraspingActionSpec(const string &s) : specstr(s) { readType(); }
    GraspingActionSpec(const string &s, const GraspingActionContext &c) : specstr(s), ctx(c) { readType(); }
    void setContext(const GraspingActionContext &ctx_) { ctx = ctx_; }

    Action::Ptr createAction();
//    void getSuccessors(vector<GraspingActionSpec> &out);

private:
    void readType();

    Action::Ptr createGrabAction(stringstream &ss);
    Action::Ptr createReleaseAction(stringstream &ss);
    Action::Ptr createMoveAction(stringstream &ss);
};


#endif // __FL_GRASPINGACTIONS_H__
