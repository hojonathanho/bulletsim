#ifndef __FL_GRASPINGACTIONS_H__
#define __FL_GRASPINGACTIONS_H__

// Provides a friendly interface over the actions defined in graspingactions_impl.h

#include "simulation/environment.h"
#include "simulation/openravesupport.h"
#include "robots/pr2.h"
#include "cloth.h"

struct GraspingActionContext {
    Environment::Ptr env;
    RaveRobotObject::Ptr robot;
    RaveRobotObject::Manipulator::Ptr manip;
    PR2SoftBodyGripper::Ptr sbgripper;
    Cloth::Ptr cloth;
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

    string specstr;

    GraspingActionSpec(const string &s) : specstr(s) { readType(); }

    Action::Ptr createAction(GraspingActionContext &ctx) const;

    void genSuccessors(const GraspingActionContext &ctx, vector<GraspingActionSpec> &out) const;
    vector<GraspingActionSpec> genSuccessors(const GraspingActionContext &ctx) const;

private:
    void readType();
};

#endif // __FL_GRASPINGACTIONS_H__
