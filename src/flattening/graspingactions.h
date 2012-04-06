#ifndef __FL_GRASPINGACTIONS_H__
#define __FL_GRASPINGACTIONS_H__

// Provides a friendly interface over the actions defined in graspingactions_impl.h

#include "simulation/environment.h"
#include "simulation/openravesupport.h"
#include "robots/pr2.h"
#include "cloth.h"

#include <stdexcept>

class GraspingActionFailed : public std::runtime_error {
public:
    GraspingActionFailed(const string &msg) : std::runtime_error(msg) { }
};

struct GraspingActionContext {
    Environment::Ptr env;
    RaveRobotObject::Ptr robot;
    RaveRobotObject::Manipulator::Ptr manip;
    PR2SoftBodyGripper::Ptr sbgripper;
    Cloth::Ptr cloth;

    GraspingActionContext fork() const;
    void runAction(Action::Ptr a);
};

struct GraspingActionSpec {
    typedef boost::shared_ptr<GraspingActionSpec> Ptr;

    enum Type {
        NONE = 0, // format: none
        GRAB, // format: grab <node idx> <direction vec x> <direction vec y> <direction vec z>
        RELEASE, // format: release
        MOVE, // relative movement; format: move <dx> <dy> <dz>
        GRAB_AND_MOVE, // format: grab_and_move <grabstr> <movestr>
    } type;

    string specstr;

    GraspingActionSpec() : specstr("none"), type(NONE) { }
    GraspingActionSpec(const string &s) : specstr(s) { readType(); }
    Action::Ptr createAction(GraspingActionContext &ctx) const;

    void genSuccessors(const GraspingActionContext &ctx, vector<GraspingActionSpec> &out) const;
    vector<GraspingActionSpec> genSuccessors(const GraspingActionContext &ctx) const;

private:
    void readType();
};

#endif // __FL_GRASPINGACTIONS_H__
