#include "simulation/simplescene.h"
#include "simulation/basicobjects.h"
#include "simulation/softbodies.h"
#include "simulation/config_bullet.h"
#include "simulation/config_viewer.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>

struct ClothSpec {
    btSoftBody *psb;
    int resx, resy;
};

static BulletSoftObject::Ptr createCloth(Scene &scene, btScalar s, int resx, int resy, const btVector3 &center) {
    btSoftBody *psb = btSoftBodyHelpers::CreatePatch(
        scene.env->bullet->softBodyWorldInfo,
        center + btVector3(-s,-s,0),
        center + btVector3(+s,-s,0),
        center + btVector3(-s,+s,0),
        center + btVector3(+s,+s,0),
        resx, resy,
        0, true);

    psb->m_cfg.piterations = 2;
    psb->m_cfg.collisions = btSoftBody::fCollision::CL_SS
        | btSoftBody::fCollision::CL_RS
        | btSoftBody::fCollision::CL_SELF;
    psb->m_cfg.kDF = 1.0;
    psb->getCollisionShape()->setMargin(0.05);
    btSoftBody::Material *pm = psb->appendMaterial();
    pm->m_kLST = 0.1;
    psb->generateBendingConstraints(2, pm);
    psb->randomizeConstraints();
    psb->setTotalMass(1, true);
    psb->generateClusters(0);

    return BulletSoftObject::Ptr(new BulletSoftObject(psb));
}


// an action that moves a node
// completely specified by ActionDesc
class NodeMoveAction : public Action {
    btSoftBody *psb;
    Environment::Ptr env;

    SphereObject::Ptr anchorpt; // an object that we can attach an an anchor to
    int anchoridx;
    bool anchorAppended;
    BulletObject::MoveAction::Ptr moveaction;

    void appendAnchor(const btTransform &trans) {
        if (anchorAppended) return;

        anchorpt.reset(new SphereObject(0, 0.005*METERS, trans, true));
        anchorpt->setColor(1, 0, 1, 1);
        anchorpt->rigidBody->setCollisionFlags(anchorpt->rigidBody->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);
        env->add(anchorpt);
        moveaction = anchorpt->createMoveAction();
        moveaction->setExecTime(execTime);

        psb->appendAnchor(spec.i, anchorpt->rigidBody.get());
        anchoridx = psb->m_anchors.size() - 1;
        if (anchoridx != 0) {
            cout << "WARNING: attaching multiple anchors to cloth in NodeMoveAction!" << endl;
            BOOST_ASSERT(false);
        }

        anchorAppended = true;
    }

    void removeAnchor() {
        if (!anchorAppended) return;

        env->remove(anchorpt);
        anchorpt.reset();
        moveaction.reset();

        psb->m_anchors[anchoridx].m_node->m_battach = 0;
        if (psb->m_anchors.size() != 1) {
            cout << "WARNING: number of anchors on cloth != 1" << endl;
            BOOST_ASSERT(false);
        }
        psb->m_anchors.clear();

        anchorAppended = false;
    }

public:
    struct Spec {
        int i; // index of node on cloth
        btVector3 v; // direction to move the node, magnitude signifies distance
        float t; // execution time for the action
        Spec() { }
        Spec(int i_, const btVector3 &v_, float t_) : i(i_), v(v_), t(t_) { }
    } spec;

    NodeMoveAction() : anchorAppended(false) { }

    // sets the environment and soft body to act on
    void setSoftBody(Environment::Ptr env_, btSoftBody *psb_) {
        env = env_; psb = psb_;
    }

    void readSpec() {
        setExecTime(spec.t);
    }

    void reset() {
        Action::reset();
        removeAnchor();
    }

    void step(float dt) {
        if (done()) return;

        if (timeElapsed == 0) {
            // first step
            const btVector3 &nodepos = psb->m_nodes[spec.i].m_x;
            btTransform starttrans(btQuaternion(0, 0, 0, 1), nodepos);
            btTransform endtrans(btQuaternion(0, 0, 0, 1), nodepos + spec.v);
            appendAnchor(starttrans);
            moveaction->setEndpoints(starttrans, endtrans);
        }
        moveaction->step(dt);
        stepTime(dt);

        if (done()) removeAnchor();
    }
};

// a fake list
// do NOT use two actions returned by at() at once! they will be
// the same NodeMoveAction object, just with different specs
class NodeActionList {
    NodeMoveAction ac;
    vector<NodeMoveAction::Spec> specs;

public:
    NodeActionList() { }

    void add(const NodeMoveAction::Spec &s) { specs.push_back(s); }
    void clear() { specs.clear(); }
    int size() const { return specs.size(); }

    NodeMoveAction &at(int i) {
        ac.reset();
        ac.spec = specs[i];
        ac.readSpec();
        return ac;
    }
    NodeMoveAction &operator[](int i) { return at(i); }
};

// action spec generation helpers
static void genCompleteSpecsForNode(NodeActionList &l, int idx, bool excludeNoOp=true) {
    static float ACTION_TIME = 1.; // 1 sec
    static btScalar d = 0.1*METERS;
    for (int i = -1; i <= 1; ++i)
        for (int j = -1; j <= 1; ++j)
            for (int k = -1; k <= 1; ++k)
                if (!excludeNoOp || i != 0 || j != 0 || k != 0)
                    l.add(NodeMoveAction::Spec(idx, d * btVector3(i, j, k).normalized(), ACTION_TIME));
}
static void genSpecsForNodeWithoutCorners(NodeActionList &l, int idx) {
    static float ACTION_TIME = 1.; // 1 sec
    static btScalar d = 0.1*METERS;

    l.add(NodeMoveAction::Spec(idx, d * btVector3(1, 0, 0), ACTION_TIME));
    l.add(NodeMoveAction::Spec(idx, d * btVector3(0, 1, 0), ACTION_TIME));
    l.add(NodeMoveAction::Spec(idx, d * btVector3(0, 0, 1), ACTION_TIME));

    l.add(NodeMoveAction::Spec(idx, d * btVector3(-1, 0, 0), ACTION_TIME));
    l.add(NodeMoveAction::Spec(idx, d * btVector3(0, -1, 0), ACTION_TIME));
    l.add(NodeMoveAction::Spec(idx, d * btVector3(0, 0, -1), ACTION_TIME));

    l.add(NodeMoveAction::Spec(idx, d * btVector3(1, 1, 0).normalized(), ACTION_TIME));
    l.add(NodeMoveAction::Spec(idx, d * btVector3(-1, 1, 0).normalized(), ACTION_TIME));
    l.add(NodeMoveAction::Spec(idx, d * btVector3(-1, -1, 0).normalized(), ACTION_TIME));
    l.add(NodeMoveAction::Spec(idx, d * btVector3(1, -1, 0).normalized(), ACTION_TIME));

    l.add(NodeMoveAction::Spec(idx, d * btVector3(1, 0, 1).normalized(), ACTION_TIME));
    l.add(NodeMoveAction::Spec(idx, d * btVector3(-1, 0, 1).normalized(), ACTION_TIME));
    l.add(NodeMoveAction::Spec(idx, d * btVector3(-1, 0, -1).normalized(), ACTION_TIME));
    l.add(NodeMoveAction::Spec(idx, d * btVector3(1, 0, -1).normalized(), ACTION_TIME));

    l.add(NodeMoveAction::Spec(idx, d * btVector3(0, 1, 1).normalized(), ACTION_TIME));
    l.add(NodeMoveAction::Spec(idx, d * btVector3(0, -1, 1).normalized(), ACTION_TIME));
    l.add(NodeMoveAction::Spec(idx, d * btVector3(0, -1, -1).normalized(), ACTION_TIME));
    l.add(NodeMoveAction::Spec(idx, d * btVector3(0, 1, -1).normalized(), ACTION_TIME));
}

static void genSpecsForCloth(NodeActionList &l, int resx, int resy) {
#define IDX(_x_, _y_) ((_y_)*resx+(_x_))

    // just the 4 corners for now
    genCompleteSpecsForNode(l, IDX(0, 0));
    genCompleteSpecsForNode(l, IDX(resx-1, 0));
    genCompleteSpecsForNode(l, IDX(0, resy-1));
    genCompleteSpecsForNode(l, IDX(resx-1, resy-1));

#undef IDX
}

int main(int argc, char *argv[]) {
    GeneralConfig::scale = 20.;
    ViewerConfig::cameraHomePosition = btVector3(100, 0, 100);
    BulletConfig::dt = BulletConfig::internalTimeStep = 0.02;
    BulletConfig::maxSubSteps = 0;

    Parser parser;

    parser.addGroup(GeneralConfig());
    parser.addGroup(BulletConfig());
    parser.addGroup(SceneConfig());
    parser.read(argc, argv);

    Scene scene;

    const float table_height = .5;
    const float table_thickness = .05;
    BoxObject::Ptr table(new BoxObject(0, GeneralConfig::scale * btVector3(.75,.75,table_thickness/2),
        btTransform(btQuaternion(0, 0, 0, 1), GeneralConfig::scale * btVector3(1.2, 0, table_height-table_thickness/2))));
    table->rigidBody->setFriction(1);
    scene.env->add(table);

    const int divs = 25;
    BulletSoftObject::Ptr cloth(
        createCloth(scene, GeneralConfig::scale * 0.25, divs, divs, GeneralConfig::scale * btVector3(0.9, 0, table_height+0.01)));
    scene.env->add(cloth);
    ClothSpec clothspec = { cloth->softBody.get(), divs, divs };

    NodeActionList actions;
    genSpecsForCloth(actions, clothspec.resx, clothspec.resy);


    scene.startViewer();
    scene.stepFor(BulletConfig::dt, 1);

    for (int i = 0; i < actions.size(); ++i) {
        NodeMoveAction &ac = actions[i];

        BulletInstance::Ptr bullet2(new BulletInstance);
        bullet2->setGravity(BulletConfig::gravity);
        OSGInstance::Ptr osg2(new OSGInstance);
        scene.osg->root->addChild(osg2->root.get());
        Fork::Ptr fork(new Fork(scene.env, bullet2, osg2));
        scene.registerFork(fork);
        BulletSoftObject::Ptr cloth2 = boost::static_pointer_cast<BulletSoftObject>(fork->forkOf(cloth));

        ac.setSoftBody(fork->env, cloth2->softBody.get());

        while (!ac.done()) {
            ac.step(BulletConfig::dt);
            fork->env->step(BulletConfig::dt, BulletConfig::maxSubSteps, BulletConfig::internalTimeStep);
            scene.draw();
        }
        scene.osg->root->removeChild(osg2->root.get());
        scene.unregisterFork(fork);
        //scene.runAction(ac, BulletConfig::dt);
    }

    return 0;
}
