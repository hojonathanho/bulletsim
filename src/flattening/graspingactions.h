#ifndef __FL_GRASPINGACTIONS_H__
#define __FL_GRASPINGACTIONS_H__

// Provides a friendly interface over the actions defined in graspingactions_impl.h

#include "simulation/environment.h"
#include "simulation/openravesupport.h"
#include "robots/pr2.h"
#include "cloth.h"
#include "genmanip.h"

#include <stdexcept>

class GraspingActionFailed : public std::runtime_error {
public:
    GraspingActionFailed(const string &msg) : std::runtime_error(msg) { }
};

struct GraspingActionContext {
    Environment::Ptr env;
    RaveRobotObject::Ptr robot;

    GenManip::Ptr gmanip;
    GenPR2SoftGripper::Ptr sbgripper;

    Cloth::Ptr cloth;
    struct TableDims {
        btTransform trans;
        btVector3 halfExtents;
        TableDims() { }
        TableDims(const btTransform &t, const btVector3 &he) : trans(t), halfExtents(he) { }
    } table;

    // for debugging
    Scene *scene;
    void enableDrawing(Scene *);
    void disableDrawing();

    GraspingActionContext fork() const;
    void runAction(TimedAction::Ptr a, bool debugDraw=false);

    GraspingActionContext() : scene(NULL) { }
    GraspingActionContext(
            Environment::Ptr env_,
            RaveRobotObject::Ptr robot_,
            GenManip::Ptr gmanip_,
            GenPR2SoftGripper::Ptr sbgripper_,
            Cloth::Ptr cloth_,
            const TableDims &table_) :
        env(env_), robot(robot_), gmanip(gmanip_),
        sbgripper(sbgripper_), cloth(cloth_),
        table(table_),
        scene(NULL) { }
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
    TimedAction::Ptr createAction(GraspingActionContext &ctx) const;

    void genSuccessors(const GraspingActionContext &ctx, vector<GraspingActionSpec> &out) const;
    vector<GraspingActionSpec> genSuccessors(const GraspingActionContext &ctx) const;

private:
    void readType();
};

// utility functions
btVector3 calcGraspDir(const GraspingActionContext &ctx, int node);

#endif // __FL_GRASPINGACTIONS_H__
