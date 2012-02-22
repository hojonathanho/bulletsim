#include "simulation/simplescene.h"
#include "simulation/basicobjects.h"
#include "simulation/softbodies.h"
#include "simulation/config_bullet.h"
#include "simulation/config_viewer.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>

static BulletSoftObject::Ptr createCloth(Scene &scene, btScalar s, const btVector3 &center) {
    const int divs = 25;

    btSoftBody *psb = btSoftBodyHelpers::CreatePatch(
        scene.env->bullet->softBodyWorldInfo,
        center + btVector3(-s,-s,0),
        center + btVector3(+s,-s,0),
        center + btVector3(-s,+s,0),
        center + btVector3(+s,+s,0),
        divs, divs,
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
class NodeMoveAction : public Action {
private:
    btSoftBody *psb;
    Environment::Ptr env;

    int idx; // index of the node in psb
    btVector3 dir; // direction to move the node, magnitude signifies distance

    SphereObject::Ptr anchorpt; // an object that we can attach an an anchor to
    int anchoridx;
    bool anchorAppended;
    BulletObject::MoveAction::Ptr moveaction;

    void appendAnchor(const btTransform &trans) {
        if (anchorAppended) return;

        anchorpt.reset(new SphereObject(0, 0.005*METERS, trans, true));
        anchorpt->rigidBody->setCollisionFlags(anchorpt->rigidBody->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);
        env->add(anchorpt);
        moveaction = anchorpt->createMoveAction();
        moveaction->setExecTime(execTime);

        psb->appendAnchor(idx, anchorpt->rigidBody.get());
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
    typedef boost::shared_ptr<NodeMoveAction> Ptr;

    NodeMoveAction(int nodeidx) : idx(nodeidx), anchorAppended(false) { }
    void setSoftBody(Environment::Ptr env_, btSoftBody *psb_) {
        env = env_; psb = psb_;
    }
    void setMovement(const btVector3 &dir_) { dir = dir_; }

    void reset() {
        moveaction->reset();
        Action::reset();
        removeAnchor();
    }

    void step(float dt) {
        if (done()) return;

        if (timeElapsed == 0) {
            // first step
            const btVector3 &nodepos = psb->m_nodes[idx].m_x;
            btTransform starttrans(btQuaternion(0, 0, 0, 1), nodepos);
            btTransform endtrans(btQuaternion(0, 0, 0, 1), nodepos + dir);
            appendAnchor(starttrans);
            moveaction->setEndpoints(starttrans, endtrans);
        }
        moveaction->step(dt);
        stepTime(dt);

        if (done()) removeAnchor();
    }
};

class NodeActionList {
    vector<NodeMoveAction::Ptr> actions;
    btSoftBody *psb;
    Environment::Ptr env;

    void genSingle(int idx, const btVector3 &vec) {
        NodeMoveAction::Ptr a(new NodeMoveAction(idx));
        a->setSoftBody(env, psb);
        a->setMovement(vec);
        a->setExecTime(1);
        actions.push_back(a);
    }

    void genActionsForNode(int idx) {
        const btScalar d = 0.1*METERS;

        genSingle(idx, btVector3(d, 0, 0));
        genSingle(idx, btVector3(-d, 0, 0));
        genSingle(idx, btVector3(0, d, 0));
        genSingle(idx, btVector3(0, -d, 0));
        genSingle(idx, btVector3(0, 0, d));
        genSingle(idx, btVector3(0, 0, -d));

        genSingle(idx, d * btVector3(1, 1, 0).normalized());
        genSingle(idx, d * btVector3(-1, 1, 0).normalized());
        genSingle(idx, d * btVector3(-1, -1, 0).normalized());
        genSingle(idx, d * btVector3(1, -1, 0).normalized());

        genSingle(idx, d * btVector3(1, 0, 1).normalized());
        genSingle(idx, d * btVector3(-1, 0, 1).normalized());
        genSingle(idx, d * btVector3(-1, 0, -1).normalized());
        genSingle(idx, d * btVector3(1, 0, -1).normalized());

        genSingle(idx, d * btVector3(0, 1, 1).normalized());
        genSingle(idx, d * btVector3(0, -1, 1).normalized());
        genSingle(idx, d * btVector3(0, -1, -1).normalized());
        genSingle(idx, d * btVector3(0, 1, -1).normalized());
    }

    void generate() {
        actions.clear();
        for (int i = 0; i < psb->m_nodes.size(); ++i) {
            genActionsForNode(i);
        }
        cout << "made " << actions.size() << " actions!" << endl;
    }

    // for iterating through actions
    int nextActionPos;

public:
    NodeActionList(Environment::Ptr env_, btSoftBody *psb_) : env(env_), psb(psb_) {
        generate();
        nextActionPos = 0;
    }

    void reset() { nextActionPos = 0; }
    NodeMoveAction::Ptr next() {
        if (nextActionPos < actions.size())
            return actions[nextActionPos++];
        return NodeMoveAction::Ptr();
    }
};

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

    BulletSoftObject::Ptr cloth(
        createCloth(scene, GeneralConfig::scale * 0.25, GeneralConfig::scale * btVector3(0.9, 0, table_height+0.01)));
    scene.env->add(cloth);
    cout << "STARTING WITH: " << cloth->softBody->m_anchors.size() << endl;

    scene.startViewer();

    NodeActionList actions(scene.env, cloth->softBody.get());
    scene.setDrawing(false);

    cout << "running actions" << endl;
    NodeMoveAction::Ptr a;
    while (a = actions.next())
        scene.runAction(a, BulletConfig::dt);
    cout << "done running actions" << endl;

//    scene.startLoop();

    return 0;
}
