#include <Wm5Core.h>
#include <Wm5Mathematics.h>
#include "simulation/simplescene.h"
#include "simulation/basicobjects.h"
#include "simulation/softbodies.h"
#include "simulation/config_bullet.h"
#include "simulation/config_viewer.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <cstdlib>
#include <boost/date_time/posix_time/posix_time.hpp>

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
    psb->m_cfg.kDF = 0.6;
    psb->m_cfg.kSSHR_CL = 1.0; // so the cloth doesn't penetrate itself
    psb->getCollisionShape()->setMargin(0.05);
    btSoftBody::Material *pm = psb->appendMaterial();
    pm->m_kLST = 1;
    pm->m_kAST = 1;
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
        moveaction->setExecTime(execTime * spec.movingFrac);

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
        float movingFrac;
        Spec() { }
        Spec(int i_, const btVector3 &v_, float t_) : i(i_), v(v_), t(t_), movingFrac(0.05) { }
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
    static float ACTION_TIME = 0.5;
    static btScalar d = 0.5*METERS;
    for (int i = -1; i <= 1; ++i)
        for (int j = -1; j <= 1; ++j)
            for (int k = -1; k <= 1; ++k)
                if (!excludeNoOp || i != 0 || j != 0 || k != 0)
                    l.add(NodeMoveAction::Spec(idx, d * btVector3(i, j, k).normalized(), ACTION_TIME));
}
static void genSpecsForNodeWithoutCorners(NodeActionList &l, int idx) {
    static float ACTION_TIME = 0.5;
    static btScalar d = 0.5*METERS;

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

// runs some random actions on a cloth
static void randomizeCloth(Scene &scene, btSoftBody *psb, int resx, int resy, int numSteps=20) {
    NodeActionList actions;
    genSpecsForCloth(actions, resx, resy);
    for (int i = 0; i < numSteps; ++i) {
        NodeMoveAction &a = actions[rand() % actions.size()];
        a.setSoftBody(scene.env, psb);
        scene.runAction(a, BulletConfig::dt);
    }
}

static void liftClothMiddle(Scene &scene, ClothSpec &cs, bool disableDrawing=true) {
    bool d = scene.drawingOn;
    scene.setDrawing(!disableDrawing);

    // add forces
//    const int steps = 30;
    const int steps = 50;
    const btVector3 force(0, 0, 0.5);
    for (int i = 0; i < steps; ++i) {
        for (int x = 0; x < cs.resx; ++x)
            cs.psb->addForce(force, (cs.resy / 2)*cs.resx + x);
        scene.step(BulletConfig::dt);
    }

    // let the cloth stabilize
    const int restingsteps = 400;
    for (int i = 0; i < restingsteps; ++i) {
        scene.step(BulletConfig::dt);
    }
    //scene.stepFor(BulletConfig::dt, 10);

    // clear velocities
    for (int i = 0; i < cs.psb->m_nodes.size(); ++i) {
        cs.psb->m_nodes[i].m_v.setZero();
    }

    scene.setDrawing(d);
}

static btVector3 softBodyCM(btSoftBody *psb) {
    // assumes all node masses are the same..
    btVector3 sum(0, 0, 0);
    for (int i = 0; i < psb->m_nodes.size(); ++i)
        sum += psb->m_nodes[i].m_x;
    return sum * 1./psb->m_nodes.size();
}

static btScalar projectedConvexHullArea(btSoftBody *psb) {
    // projects onto the xy-plane
    Wm5::Vector2f *vertices = new Wm5::Vector2f[psb->m_nodes.size()];
    for (int i = 0; i < psb->m_nodes.size(); ++i) {
        vertices[i].X() = psb->m_nodes[i].m_x.x();
        vertices[i].Y() = psb->m_nodes[i].m_x.y();
    }

    Wm5::ConvexHullf *hull = new Wm5::ConvexHull2f(psb->m_nodes.size(), vertices, 0.001f, false, Wm5::Query::QT_REAL);
    const int nSimplices = hull->GetNumSimplices();
    const int *indices = hull->GetIndices();
    Wm5::Vector2f *hullvertices = new Wm5::Vector2f[nSimplices];
    for (int i = 0; i < nSimplices; ++i)
        hullvertices[i] = vertices[indices[i]];

    btScalar area = (btScalar) Wm5::Polygon2f(nSimplices, hullvertices).ComputeArea();

//    delete [] hullvertices;
    delete hull;
    delete [] vertices;

    return area;
}

static void greedy(Scene &scene, BulletSoftObject::Ptr initCloth, NodeActionList &candidateActions, int steps, NodeActionList &out) {
    // greedily find a list of actions that flattens the cloth

    struct {
        BulletInstance::Ptr bullet;
        OSGInstance::Ptr osg;
        Fork::Ptr fork;
        BulletSoftObject::Ptr cloth;
    } stepState;

    stepState.bullet.reset(new BulletInstance);
    stepState.bullet->setGravity(BulletConfig::gravity);
    stepState.osg.reset(new OSGInstance);
    stepState.fork.reset(new Fork(scene.env, stepState.bullet, stepState.osg));
    stepState.cloth = boost::static_pointer_cast<BulletSoftObject>(stepState.fork->forkOf(initCloth));
    BOOST_ASSERT(stepState.cloth);

    boost::posix_time::ptime begTick0(boost::posix_time::microsec_clock::local_time());
    for (int step = 0; step < steps; ++step) {
        cout << "step " << (step+1) << "/" << steps << '\n';

        // at each step, try all actions and find the one that flattens the cloth
        struct {
            BulletInstance::Ptr bullet;
            OSGInstance::Ptr osg;
            Fork::Ptr fork;
            BulletSoftObject::Ptr cloth;
            NodeMoveAction::Spec actionspec;
            float val;
        } optimal;

        int numActionsRun = 0;
        btVector3 startingCM = softBodyCM(stepState.cloth->softBody.get());
        cout << "starting cloth height: " << startingCM.z() << endl;

#if 0
        optimal.bullet = stepState.bullet;
        optimal.osg = stepState.osg;
        optimal.fork = stepState.fork;
        optimal.cloth = stepState.cloth;
        optimal.actionspec = candidateActions[rand() % candidateActions.size()].spec;
#endif
        for (int i = 0; i < candidateActions.size(); ++i) {
            NodeMoveAction &ac = candidateActions[i];

            // ignore actions that drag down the cloth (for now)
            if (ac.spec.v.z() != 0) continue;
            // ignore actions that drag a node toward the center
            if (ac.spec.v.dot(stepState.cloth->softBody->m_nodes[ac.spec.i].m_x - startingCM) < 0) continue;

            cout << "\taction " << (i+1) << "/" << candidateActions.size() << '\n';

            BulletInstance::Ptr bullet2(new BulletInstance);
            bullet2->setGravity(BulletConfig::gravity);
            OSGInstance::Ptr osg2(new OSGInstance);
            scene.osg->root->addChild(osg2->root.get());
            Fork::Ptr fork(new Fork(stepState.fork->env, bullet2, osg2));
            scene.registerFork(fork);
            BulletSoftObject::Ptr cloth2 = boost::static_pointer_cast<BulletSoftObject>(fork->forkOf(stepState.cloth));

            ac.setSoftBody(fork->env, cloth2->softBody.get());

            while (!ac.done()) {
                ac.step(BulletConfig::dt);
                fork->env->step(BulletConfig::dt, BulletConfig::maxSubSteps, BulletConfig::internalTimeStep);
                scene.draw();
            }

            float avgHeight = softBodyCM(cloth2->softBody.get()).z();
            cout << "\t\taverage node height: " << avgHeight << '\n';
            float hullArea = projectedConvexHullArea(cloth2->softBody.get());
            cout << "\t\tarea of convex hull of projected vertices: " << hullArea << '\n';

            float val = -hullArea;

            if (numActionsRun == 0 || val < optimal.val) {
                optimal.val = val;

                optimal.bullet = bullet2;
                optimal.osg = osg2;
                optimal.fork = fork;
                optimal.cloth = cloth2;

                optimal.actionspec = ac.spec;
            }

            scene.osg->root->removeChild(osg2->root.get());
            scene.unregisterFork(fork);

            ++numActionsRun;
        }

        //cout << "optimal got cloth height: " << optimal.val << endl;
        cout << "optimal val: " << optimal.val << endl;
        out.add(optimal.actionspec);
/*

        scene.idle(true);
        out[out.size()-1].setSoftBody(scene.env, initCloth->softBody.get());
        scene.runAction(out[out.size()-1], BulletConfig::dt);
        scene.idle(true);
        */

        stepState.bullet = optimal.bullet;
        stepState.osg = optimal.osg;
        stepState.fork = optimal.fork;
        stepState.cloth = optimal.cloth;
    }
    // keep the last step in the viewer
    scene.osg->root->addChild(stepState.osg->root.get());
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
    table->rigidBody->setFriction(0.5);
    scene.env->add(table);

    const int divs = 31;
    BulletSoftObject::Ptr cloth(
        createCloth(scene, GeneralConfig::scale * 0.25, divs, divs, GeneralConfig::scale * btVector3(0.9, 0, table_height+0.01)));
    scene.env->add(cloth);
    ClothSpec clothspec = { cloth->softBody.get(), divs, divs };

    NodeActionList actions;
    genSpecsForCloth(actions, clothspec.resx, clothspec.resy);

    scene.startViewer();
    scene.stepFor(BulletConfig::dt, 1);

    cout << "lifting..." << endl;
    liftClothMiddle(scene, clothspec);
    scene.idle(true);
    cout << "done lifting."<< endl;
//    randomizeCloth(scene, cloth->softBody.get(), clothspec.resx, clothspec.resy);

    NodeActionList optimal;
//    scene.setDrawing(false);
    greedy(scene, cloth, actions, 10, optimal);
    scene.setDrawing(true);
    scene.idle(true);

    // replay optimal actions on cloth
    for (int i = 0; i < optimal.size(); ++i) {
        NodeMoveAction &a = optimal[i];
        a.setSoftBody(scene.env, cloth->softBody.get());
        while (!a.done()) {
            a.step(BulletConfig::dt);
            scene.env->step(BulletConfig::dt, BulletConfig::maxSubSteps, BulletConfig::internalTimeStep);
            scene.draw();
            scene.idle(true);
        }
    }
    scene.idle(true);

    return 0;
#if 0
    boost::posix_time::ptime begTick0(boost::posix_time::microsec_clock::local_time());
    int numActionsRun = 0;
    for (int i = 0; i < actions.size(); ++i) {
        NodeMoveAction &ac = actions[i];

        if (ac.spec.v.z() < 0) continue;

        cout << "running action: " << ac.spec.v.x() << ' ' << ac.spec.v.y() << ' ' << ac.spec.v.z() << endl;

        BulletInstance::Ptr bullet2(new BulletInstance);
        bullet2->setGravity(BulletConfig::gravity);
        OSGInstance::Ptr osg2(new OSGInstance);
        scene.osg->root->addChild(osg2->root.get());
        Fork::Ptr fork(new Fork(scene.env, bullet2, osg2));
        scene.registerFork(fork);
        BulletSoftObject::Ptr cloth2 = boost::static_pointer_cast<BulletSoftObject>(fork->forkOf(cloth));

        ac.setSoftBody(fork->env, cloth2->softBody.get());

        boost::posix_time::ptime begTick(boost::posix_time::microsec_clock::local_time());
        while (!ac.done()) {
            ac.step(BulletConfig::dt);
            fork->env->step(BulletConfig::dt, BulletConfig::maxSubSteps, BulletConfig::internalTimeStep);
            scene.draw();
        }
        cout << "average node height: " << softBodyCM(cloth2->softBody.get()).z() << endl;
        boost::posix_time::ptime endTick(boost::posix_time::microsec_clock::local_time());
        std::cout << boost::posix_time::to_simple_string(endTick - begTick) << std::endl;
        /*
        for (int z = 0; z < 400; ++z) {
            fork->env->step(BulletConfig::dt, BulletConfig::maxSubSteps, BulletConfig::internalTimeStep);
            scene.draw();
        }*/
//        scene.idle(true);

        scene.osg->root->removeChild(osg2->root.get());
        scene.unregisterFork(fork);

        ++numActionsRun;
    }
    boost::posix_time::ptime endTick0(boost::posix_time::microsec_clock::local_time());
    std::cout << "total time: " << boost::posix_time::to_simple_string(endTick0 - begTick0) << std::endl;
    std::cout << " (" << numActionsRun << " actions, " << boost::posix_time::to_simple_string((endTick0 - begTick0)/(float)numActionsRun) << " / action)" << std::endl;

    return 0;
#endif
}
