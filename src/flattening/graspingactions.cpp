#include "graspingactions.h"

#include "graspingactions_impl.h"
#include "edges.h"
#include "simulation/environment.h"
#include "simulation/config_bullet.h"
#include "clothutil.h"

GraspingActionContext GraspingActionContext::fork() const {
    BulletInstance::Ptr fork_bullet(new BulletInstance);
    fork_bullet->setGravity(BulletConfig::gravity);
    OSGInstance::Ptr fork_osg(new OSGInstance);
    Fork::Ptr fork(new Fork(env, fork_bullet, fork_osg));
    RaveRobotObject::Ptr fork_robot =
        boost::static_pointer_cast<RaveRobotObject>(fork->forkOf(robot));
    Cloth::Ptr fork_cloth =
        boost::static_pointer_cast<Cloth>(fork->forkOf(cloth));
    GenManip::Ptr fork_gmanip = gmanip->getForked(*fork, fork_robot);
//    RaveRobotObject::Manipulator::Ptr fork_manip =
        //fork_robot->getManipByIndex(manip->index);
    GenPR2SoftGripper::Ptr sbgripper(new GenPR2SoftGripper(fork_robot, fork_gmanip, true)); // TODO: leftGripper flag
    sbgripper->setGrabOnlyOnContact(true);
    sbgripper->setTarget(fork_cloth);
    return GraspingActionContext(fork->env, fork_robot, fork_gmanip, sbgripper, fork_cloth);
}

void GraspingActionContext::runAction(Action::Ptr a) {
    while (!a->done()) {
        a->step(BulletConfig::dt);
        env->step(BulletConfig::dt, BulletConfig::maxSubSteps, BulletConfig::internalTimeStep);
    }
    // let scene settle
    static const float SETTLE_TIME = 1;
    for (float t = 0; t < SETTLE_TIME; t += BulletConfig::dt)
        env->step(BulletConfig::dt, BulletConfig::maxSubSteps, BulletConfig::internalTimeStep);
}

static Action::Ptr createGrabAction(GraspingActionContext &ctx, stringstream &ss) {
    // format: grab <node index> <approach vec x> <approach vec y> <approach vec z>
    int nodeidx; ss >> nodeidx;
    btScalar vx, vy, vz; ss >> vx >> vy >> vz;
    return GraspClothNodeAction::Ptr(new GraspClothNodeAction(
                ctx.robot, ctx.gmanip, ctx.sbgripper, ctx.cloth,
                nodeidx, btVector3(vx, vy, vz)));
}

static Action::Ptr createReleaseAction(GraspingActionContext &ctx, stringstream &ss) {
    // format: release
    FunctionAction::Ptr releaseAnchors(new FunctionAction(
                boost::bind(&GenPR2SoftGripper::releaseAllAnchors, ctx.sbgripper)));
    GripperOpenCloseAction::Ptr openGripper(new GripperOpenCloseAction(
                ctx.robot, ctx.gmanip->baseManip()->manip, true));
    ActionChain::Ptr a(new ActionChain);
    *a << releaseAnchors << openGripper;
    return a;
}

static Action::Ptr createMoveAction(GraspingActionContext &ctx, stringstream &ss) {
    // format: move dx dy dz
    btScalar dx, dy, dz; ss >> dx >> dy >> dz;
    ManipMoveAction::Ptr a;
    switch (ctx.gmanip->type) {
    case GenManip::TYPE_FAKE:
        a.reset(new FakeManipMoveAction(ctx.gmanip->asFake())); break;
    case GenManip::TYPE_IK:
        a.reset(new ManipIKInterpAction(ctx.robot, ctx.gmanip->asIKManip())); break;
    default:
        cout << "error: gen manip type " << ctx.gmanip->type << " not recognized by createMoveAction" << endl;
        break;
    }
    a->setRelativeTrans(btTransform(btQuaternion::getIdentity(), btVector3(dx, dy, dz)));
    return a;
}

static Action::Ptr createGrabAndMoveAction(GraspingActionContext &ctx, stringstream &ss) {
    // format: grab_and_move <grabstr> <movestr>
    string op;
    ss >> op; BOOST_ASSERT(op == "grab");
    Action::Ptr grabaction = createGrabAction(ctx, ss);
    ss >> op; BOOST_ASSERT(op == "move");
    Action::Ptr moveaction = createMoveAction(ctx, ss);
    ActionChain::Ptr chain(new ActionChain);
    *chain << grabaction << moveaction;
    return chain;
}

static Action::Ptr createNoneAction() {
    return EmptyAction::Ptr(new EmptyAction);
}

Action::Ptr GraspingActionSpec::createAction(GraspingActionContext &ctx) const {
    stringstream ss;
    ss << specstr;
    string op; ss >> op; // consume type
    switch (type) {
    case GRAB: return createGrabAction(ctx, ss); break;
    case RELEASE: return createReleaseAction(ctx, ss); break;
    case MOVE: return createMoveAction(ctx, ss); break;
    case GRAB_AND_MOVE: return createGrabAndMoveAction(ctx, ss); break;
    }
    if (op != "none")
        cout << "warning: unrecognized action op " << op << endl;
    return createNoneAction();
};

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
    else if (op == "grab_and_move")
        type = GRAB_AND_MOVE;
    else {
        cout << "error: unknown action op " << op << " (spec:" << specstr << ")" << endl;
        type = NONE;
    }
}

static void genMoveSpecs(vector<GraspingActionSpec> &out) {
    // move in horizontal plane, in 8 possible directions
    stringstream ss;
    const btScalar d = 0.1 * METERS;
    for (int i = -1; i <= 1; ++i) {
        for (int j = -1; j <= 1; ++j) {
            if (i == 0 && j == 0) continue;
            if (i != 0 && j != 0) continue; // ignore diagonals for now
            ss.str(""); // clear out ss
            ss << "move " << i*d << ' ' << j*d << ' ' << 0;
            out.push_back(GraspingActionSpec(ss.str()));
        }
    }
}

// tries out a grasp and returns the number of anchors attached
static int tryGrasp(const GraspingActionContext &ctx, int node, const btVector3 &gripperdir) {
#if 0
    BulletInstance::Ptr fork_bullet(new BulletInstance);
    fork_bullet->setGravity(BulletConfig::gravity);
    OSGInstance::Ptr fork_osg(new OSGInstance);
    Fork::Ptr fork(new Fork(ctx.env, fork_bullet, fork_osg));
    RaveRobotObject::Ptr fork_robot =
        boost::static_pointer_cast<RaveRobotObject>(fork->forkOf(ctx.robot));
    Cloth::Ptr fork_cloth =
        boost::static_pointer_cast<Cloth>(fork->forkOf(ctx.cloth));
    RaveRobotObject::Manipulator::Ptr fork_manip =
        fork_robot->getManipByIndex(ctx.manip->index);
    GenPR2SoftGripper::Ptr sbgripper(new GenPR2SoftGripper(fork_robot, fork_manip->manip, true)); // TODO: leftGripper flag
    sbgripper->setGrabOnlyOnContact(true);
    sbgripper->setTarget(fork_cloth);
    GraspingActionContext forkctx = { fork->env, fork_robot, fork_manip, sbgripper, fork_cloth };
#endif
    GraspingActionContext forkctx = ctx.fork();

    int startNumAnchors = ctx.cloth->softBody->m_anchors.size();

    stringstream ss; ss << "grab " << node << ' ' << gripperdir.x() << ' ' << gripperdir.y() << ' ' << gripperdir.z();

    try {
        Action::Ptr a = GraspingActionSpec(ss.str()).createAction(forkctx);

        while (!a->done()) {
            a->step(BulletConfig::dt);
            forkctx.env->step(BulletConfig::dt,
                    BulletConfig::maxSubSteps, BulletConfig::internalTimeStep);
        }
    } catch (const GraspingActionFailed &) {
        return 0;
    }

    return forkctx.cloth->softBody->m_anchors.size() - startNumAnchors;
}

btVector3 calcGraspDir(const GraspingActionContext &ctx, int node) {
    btVector3 dir = calcGraspDir(*ctx.cloth, node);
    if (!ctx.cloth->idxOnEdge(node)) {
        // if not an edge node, there's ambiguity in the gripper direction
        // (either dir or -dir)

        // try to disambiguate based on layers of cloth
        // in each direction
        const btVector3 &pos = ctx.cloth->psb()->m_nodes[node].m_x;
        static const float RAY_LEN = 1*METERS;
        int nforward = softBody_facesCrossed(ctx.cloth->psb(), pos, pos + RAY_LEN*dir);
        int nbackward = softBody_facesCrossed(ctx.cloth->psb(), pos, pos - RAY_LEN*dir);
        if (nbackward > nforward)
            dir = -dir;
        cout << "ray testing: " << nbackward << ' ' << nforward << endl;

        // otherwise, choose the direction that attaches the most anchors to the softbody
        if (nbackward == nforward) {
            nforward = tryGrasp(ctx, node, dir);
            nbackward = tryGrasp(ctx, node, -dir);
            cout << "forward: " << nforward << " backward: " << nbackward << endl;
            if (nbackward > nforward)
                dir = -dir;
        }
    }
    return dir;
}

static void genGrabSpecs(const GraspingActionContext &ctx, vector<GraspingActionSpec> &out) {
    // for now: only try to grab the 4 corners of the cloth
    vector<int> nodes;
    /*
    nodes.push_back(0);
    nodes.push_back(ctx.cloth->idx(ctx.cloth->resx-1, 0));
    nodes.push_back(ctx.cloth->idx(0, ctx.cloth->resy-1));
    nodes.push_back(ctx.cloth->idx(ctx.cloth->resx-1, ctx.cloth->resy-1));
    */
    nodes.push_back(0+2);
    nodes.push_back(ctx.cloth->idx(ctx.cloth->resx-1-2, 0));
    nodes.push_back(ctx.cloth->idx(0, ctx.cloth->resy-1-2));
    nodes.push_back(ctx.cloth->idx(ctx.cloth->resx-1, ctx.cloth->resy-1-2));

    // look for some fold nodes
    vector<int> foldnodes;
    ctx.cloth->updateAccel();
    calcFoldNodes(*ctx.cloth, foldnodes);
    std::random_shuffle(foldnodes.begin(), foldnodes.end());
    static const int NUM_FOLD_NODES = 4;
    for (int i = 0; i < NUM_FOLD_NODES && i < foldnodes.size(); ++i) {
        if (std::find(nodes.begin(), nodes.end(), foldnodes[i]) == nodes.end())
            nodes.push_back(foldnodes[i]);
    }

    stringstream ss;
    for (int i = 0; i < nodes.size(); ++i) {
        ss.str("");
        btVector3 dir = calcGraspDir(ctx, nodes[i]);
        ss << "grab " << nodes[i] << ' ' << dir.x() << ' ' << dir.y() << ' ' << dir.z();
        out.push_back(GraspingActionSpec(ss.str()));
    }
}

static void genReleaseSpecs(vector<GraspingActionSpec> &out) {
    out.push_back(GraspingActionSpec("release"));
}

static void genGrabAndMoveSpecs(const GraspingActionContext &ctx, vector<GraspingActionSpec> &out) {
    stringstream ss;
    vector<GraspingActionSpec> grabout, moveout;
    genGrabSpecs(ctx, grabout);
    genMoveSpecs(moveout);
    for (int i = 0; i < grabout.size(); ++i) {
        for (int j = 0; j < moveout.size(); ++j) {
            ss.str("");
            ss << "grab_and_move " << grabout[i].specstr << ' ' << moveout[j].specstr;
            out.push_back(ss.str());
        }
    }
}

static void succGrabAction(const GraspingActionContext &ctx, vector<GraspingActionSpec> &out) {
    // after grabbing, we can only move
    genMoveSpecs(out);
}

static void succReleaseAction(const GraspingActionContext &ctx, vector<GraspingActionSpec> &out) {
    // after releasing, we can only grab & move
    genGrabAndMoveSpecs(ctx, out);
}

static void succMoveAction(const GraspingActionContext &ctx, vector<GraspingActionSpec> &out) {
    // after moving, we can release or move again
    genReleaseSpecs(out);
    genMoveSpecs(out);
}

static void succGrabAndMoveAction(const GraspingActionContext &ctx, vector<GraspingActionSpec> &out) {
    // after grabbing and moving, we can do whatever we do after just moving
    succMoveAction(ctx, out);
}

static void succNoneAction(const GraspingActionContext &ctx, vector<GraspingActionSpec> &out) {
    // after a "none" action (a dummy action that does nothing), we can only grab a node
    genGrabAndMoveSpecs(ctx, out);
}

void GraspingActionSpec::genSuccessors(const GraspingActionContext &ctx, vector<GraspingActionSpec> &out) const {
    switch (type) {
    case GRAB: return succGrabAction(ctx, out); break;
    case RELEASE: return succReleaseAction(ctx, out); break;
    case MOVE: return succMoveAction(ctx, out); break;
    case GRAB_AND_MOVE: return succGrabAndMoveAction(ctx, out); break;
    case NONE: return succNoneAction(ctx, out); break;
    }
}

vector<GraspingActionSpec> GraspingActionSpec::genSuccessors(const GraspingActionContext &ctx) const {
    vector<GraspingActionSpec> v;
    genSuccessors(ctx, v);
    return v;
}
