#include "simulation/simplescene.h"
#include "simulation/basicobjects.h"
#include "simulation/softbodies.h"
#include "simulation/config_bullet.h"
#include "simulation/config_viewer.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>

static BulletSoftObject::Ptr createCloth(Scene &scene, btScalar s, const btVector3 &center) {
    const int divs = 45;

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
    int idx; // index of the node in psb
    btVector3 dir; // direction to move the node, magnitude signifies distance
    bool activated;
    Environment::Ptr env;
    SphereObject::Ptr anchorpt; // an object that we can attach an an anchor to
    BulletObject::MoveAction::Ptr moveaction;

public:
    NodeMoveAction(Environment::Ptr env_, btSoftBody *psb_) :
            env(env_), psb(psb_), activated(true) {
        anchorpt.reset(new SphereObject(0, 0.01*METERS, btTransform::getIdentity(), true));
        env->add(anchorpt);
        moveaction = anchorpt->createMoveAction();
    }

    void activate(bool a) {
        // activation
        if (!activated && a)
            env->bullet->dynamicsWorld->addRigidBody(anchorpt->rigidBody.get());

        // deactivation
        if (activated && !a)
            env->bullet->dynamicsWorld->removeRigidBody(anchorpt->rigidBody.get());

        activated = a;
    }

    void setNode(int i) { idx = i; }

    void setMovement(const btVector3 &dir_) { dir = dir_; }

    void reset() {
        moveaction.reset();
        Action::reset();
        activated = false;
        psb->m_anchors.clear(); // assumes this is the only anchor in psb
    }

    void step(float dt) {
        if (done()) return;

        if (timeElapsed == 0) {
            // first step
            const btVector3 &nodepos = psb->m_nodes[idx].m_x;
            btTransform starttrans(btQuaternion(0, 0, 0, 1), nodepos);
            btTransform endtrans(btQuaternion(0, 0, 0, 1), nodepos + dir);

            anchorpt->motionState->setKinematicPos(starttrans);
            psb->appendAnchor(idx, anchorpt->rigidBody.get());
            moveaction->setEndpoints(starttrans, endtrans);
        }

        stepTime(dt);

        moveaction->step(dt);
    }

    void setExecTime(float t) {
        moveaction->setExecTime(t);
        Action::setExecTime(t);
    }
};

int main(int argc, char *argv[]) {
    GeneralConfig::scale = 20.;
    ViewerConfig::cameraHomePosition = btVector3(100, 0, 100);
    BulletConfig::dt = BulletConfig::internalTimeStep = 0.01;
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

    scene.startViewer();

    NodeMoveAction a(scene.env, cloth->softBody.get());
    a.setNode(0);
    a.setMovement(btVector3(0, 0, 0.1) * METERS);
    a.setExecTime(10);

    scene.setDrawing(false);

    cout << "running action" << endl;
    scene.runAction(a, BulletConfig::dt);
    cout << "done running action" << endl;

    return 0;
}
