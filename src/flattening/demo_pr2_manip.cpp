#include "simulation/simplescene.h"
#include "simulation/config_bullet.h"
#include "simulation/config_viewer.h"
#include "simulation/plotting.h"
#include "simulation/util.h"
#include "robots/pr2.h"
#include <cstdlib>

#include "make_bodies.h"
#include "clothutil.h"
#include "nodeactions.h"
#include "graspingactions.h"
#include "graspingactions_impl.h"
#include "edges.h"
#include "facepicker.h"
#include "search_general.h"

class CustomScene : Scene {
    Cloth::Ptr cloth;
    PR2Manager::Ptr pr2m;
    PR2SoftBodyGripper::Ptr sbgripperleft;

    PlotPoints::Ptr foldnodeplot;
    PlotLines::Ptr folddirplot;
    PlotSpheres::Ptr pickplot;
    SoftBodyFacePicker::Ptr facepicker;

    vector<int> foldnodes;

    void runGripperAction(PR2SoftBodyGripperAction &a) {
        a.reset();
        a.toggleAction();
        try {
            runAction(a, BulletConfig::dt);
        } catch (...) { }
    }

    btSoftBody::Node *pickedNode;
    void pickCallback(int faceidx) {
        // we have to pick 1 of the 3 nodes on the face
        // just choose one that is a foldnode
        for (int i = 0; i < 3; ++i) {
            int n = cloth->psb()->m_faces[faceidx].m_n[i] - &cloth->psb()->m_nodes[0];
            if (std::find(foldnodes.begin(), foldnodes.end(), n) != foldnodes.end()) {
                pickedNode = cloth->psb()->m_faces[faceidx].m_n[i];
                return;
            }
        }
        pickedNode = cloth->psb()->m_faces[faceidx].m_n[0]; // arbitrary
    }

    void drawPick() {
        if (!pickedNode) return;

        btVector3 p = pickedNode->m_x;

        osg::ref_ptr<osg::Vec3Array> centers(new osg::Vec3Array);
        centers->push_back(osg::Vec3(p.x(), p.y(), p.z()));
        osg::ref_ptr<osg::Vec4Array> cols(new osg::Vec4Array);
        cols->push_back(osg::Vec4(1, 0, 0, 0.5));
        std::vector<float> radii;
        radii.push_back(0.01*METERS);
        pickplot->plot(centers, cols, radii);
    }

    btTransform savedTrans;
    void saveManipTrans(RaveRobotObject::Manipulator::Ptr manip) {
        savedTrans = manip->getTransform();
        cout << "saved!" << endl;

        vector<btVector3> pts;

        pts.push_back(btVector3(0, 0, 0));

        btTransform t = manip->getTransform();
        btTransform s = btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0.02)*METERS);
        btTransform final = t * s;
        pts.push_back(final.getOrigin());
        plotLines->setPoints(pts);
    }
    void moveArmToSaved(RaveRobotObject::Ptr robot, RaveRobotObject::Manipulator::Ptr manip) {
        ManipIKInterpAction a(robot, manip);
        a.setExecTime(1);
//        savedTrans.setOrigin(cloth->softBody->m_nodes[cloth->softBody->m_nodes.size()-10].m_x);
        a.setPR2TipTargetTrans(savedTrans);
        cout << "moving arm to saved trans" << endl;
        try { runAction(a, BulletConfig::dt); } catch (...) { }
        cout << "done." << endl;
    }

    int tryGrasp(int node, const btVector3 &gripperdir) {
        BulletInstance::Ptr fork_bullet(new BulletInstance);
        fork_bullet->setGravity(BulletConfig::gravity);
        OSGInstance::Ptr fork_osg(new OSGInstance);
        Fork::Ptr fork(new Fork(env, fork_bullet, fork_osg));
        RaveRobotObject::Ptr fork_pr2 =
            boost::static_pointer_cast<RaveRobotObject>(
                fork->forkOf(pr2m->pr2));
        Cloth::Ptr fork_cloth =
            boost::static_pointer_cast<Cloth>(
                    fork->forkOf(cloth));
        RaveRobotObject::Manipulator::Ptr fork_pr2Left =
            fork_pr2->getManipByIndex(pr2m->pr2Left->index);
        osg->root->addChild(fork_osg->root);

        PR2SoftBodyGripper::Ptr sbgripper(new PR2SoftBodyGripper(fork_pr2, fork_pr2Left->manip, true)); // TODO: leftGripper flag
        sbgripper->setGrabOnlyOnContact(true);
        sbgripper->setTarget(fork_cloth);

        GraspingActionContext ctx = { fork->env, fork_pr2, fork_pr2Left, sbgripper, fork_cloth };
        stringstream ss; ss << "grab " << node << ' ' << gripperdir.x() << ' ' << gripperdir.y() << ' ' << gripperdir.z();
        Action::Ptr a = GraspingActionSpec(ss.str()).createAction(ctx);

        while (!a->done()) {
            a->step(BulletConfig::dt);
            fork->env->step(BulletConfig::dt,
                    BulletConfig::maxSubSteps, BulletConfig::internalTimeStep);
            draw();
        }
        osg->root->removeChild(fork_osg->root);

        return fork_cloth->softBody->m_anchors.size();
    }

    void printSuccs(const GraspingActionContext &ctx, const GraspingActionSpec &s) {
        vector<GraspingActionSpec> v;
        s.genSuccessors(ctx, v);
        cout << "successors to: " << s.specstr << '\n';
        for (int i = 0; i < v.size(); ++i)
            cout << '\t' << v[i].specstr << '\n';
    }

    void graspPickedNode() {
        if (!pickedNode) return;
        const int node = pickedNode - &cloth->softBody->m_nodes[0];

        /*
        btVector3 folddir = calcFoldLineDir(*cloth, node, foldnodes);
        folddir.setZ(0);
        btVector3 gripperdir = folddir.cross(btVector3(0, 0, 1));
        */
        btVector3 gripperdir = calcGraspDir(*cloth, node);
        if (!cloth->idxOnEdge(node)) {
            // if not an edge node, there's ambiguity in the gripper direction
            // (either gripperdir or -gripperdir)
            // choose the one that attaches the most anchors to the softbody
            int nforward = tryGrasp(node, gripperdir);
            int nbackward = tryGrasp(node, -gripperdir);
            cout << "forward: " << nforward << " backward: " << nbackward << endl;

            if (nbackward > nforward)
                gripperdir = -gripperdir;
        }

        GraspingActionContext ctx = { env, pr2m->pr2, pr2m->pr2Left, sbgripperleft, cloth };
        stringstream ss; ss << "grab " << node << ' ' << gripperdir.x() << ' ' << gripperdir.y() << ' ' << gripperdir.z();
        GraspingActionSpec spec(ss.str());
        Action::Ptr a = spec.createAction(ctx);
        printSuccs(ctx, spec);
        try { runAction(a, BulletConfig::dt); } catch (...) { }
        cout << "done." << endl;

        GraspingActionSpec spec2("release");
        try { runAction(spec2.createAction(ctx), BulletConfig::dt); } catch(...) { }
        printSuccs(ctx, spec2);
    }

    GraspingActionSpec prevActionSpec;
    void greedyFlattenSingle() {
        GraspingActionContext ctx = { env, pr2m->pr2, pr2m->pr2Left, sbgripperleft, cloth };
        GraspingActionSpec spec = flattenCloth_greedy_single(ctx, prevActionSpec);
        try { runAction(spec.createAction(ctx), BulletConfig::dt);} catch (...) { }
        prevActionSpec = spec;
    }

    void liftCloth() {
        const int steps = 20;
    //    const int steps = 50;
        const btVector3 force(0, 0, 0.5*100);
        for (int i = 0; i < steps; ++i) {
            for (int y = 0; y < cloth->resy; ++y) {
                //cloth->psb()->addForce(force, y*cloth->resx + 0);
                cloth->psb()->addForce(force, y*cloth->resx + cloth->resx-1);
            }
            step(BulletConfig::dt);
        }

        // let the cloth stabilize
        const int restingsteps = 400;
        for (int i = 0; i < restingsteps; ++i) {
            step(BulletConfig::dt);
        }
        //scene.stepFor(BulletConfig::dt, 10);

        // clear velocities
        for (int i = 0; i < cloth->psb()->m_nodes.size(); ++i) {
            cloth->psb()->m_nodes[i].m_v.setZero();
        }
    }

    void markFolds() {
        cloth->updateAccel();
        foldnodes.clear();
        calcFoldNodes(*cloth, foldnodes);

        vector<btVector3> pts;
        vector<btVector4> colors;
        for (int i = 0; i < foldnodes.size(); ++i) {
            pts.push_back(cloth->psb()->m_nodes[foldnodes[i]].m_x);
            colors.push_back(btVector4(0, 0, 0, 1));
        }
        foldnodeplot->setPoints(pts, colors);

        pts.clear();
        for (int i = 0; i < foldnodes.size(); ++i) {
            btVector3 dir = calcFoldLineDir(*cloth, foldnodes[i], foldnodes);
            btVector3 c = cloth->psb()->m_nodes[foldnodes[i]].m_x;
            btScalar l = 0.01*METERS;
            pts.push_back(c - l*dir); pts.push_back(c + l*dir);
        }
        folddirplot->setPoints(pts);
    }

public:
    void run() {
        foldnodeplot.reset(new PlotPoints());
        env->add(foldnodeplot);
        folddirplot.reset(new PlotLines(3));
        env->add(folddirplot);
        pickplot.reset(new PlotSpheres());
        env->add(pickplot);
        pickedNode = NULL;

        // load the robot
        pr2m.reset(new PR2Manager(*this));

        // create the table
        const float table_height = .5;
        const float table_thickness = .05;
        const btVector3 table_extents = GeneralConfig::scale * btVector3(.75,.75,table_thickness/2);
        const btTransform table_trans = btTransform(btQuaternion(0, 0, 0, 1), GeneralConfig::scale * btVector3(0.8, 0, table_height-table_thickness/2));
        BoxObject::Ptr table(new BoxObject(0, table_extents, table_trans));
        table->rigidBody->setFriction(0.1);
        env->add(table);
        cout << "table margin: " << table->rigidBody->getCollisionShape()->getMargin() << endl;

        // put the table in openrave
        /*
        OpenRAVE::KinBodyPtr raveTable = OpenRAVE::RaveCreateKinBody(rave->env);
        raveTable->SetName("table");
        vector<OpenRAVE::AABB> v;
        v.push_back(OpenRAVE::AABB(util::toRaveTransform(table_trans, 1./pr2m->pr2->scale).trans, 1./pr2m->pr2->scale * util::toRaveVector(table_extents)));
        raveTable->InitFromBoxes(v, true);
        rave->env->AddKinBody(raveTable);
        */

#if 0
        OpenRAVE::ViewerBasePtr raveViewer = OpenRAVE::RaveCreateViewer(rave->env, "qtcoin");
        rave->env->AddViewer(raveViewer);
        raveViewer->main(true);
#endif

        const int resx = 45/2, resy = 31/2;
//        const btScalar lenx = GeneralConfig::scale * 0.7, leny = GeneralConfig::scale * 0.5;
        const btScalar lenx = GeneralConfig::scale * 0.7/2, leny = GeneralConfig::scale * 0.5/2;
//        const btVector3 clothcenter = GeneralConfig::scale * btVector3(0.5, 0, table_height+0.01);
        const btVector3 clothcenter = GeneralConfig::scale * btVector3(0.5, 0.1, table_height+0.01);
//        cloth = makeSelfCollidingTowel(clothcenter, lenx, leny, resx, resy, env->bullet->softBodyWorldInfo);
        cloth.reset(new Cloth(resx, resy, lenx, leny, clothcenter, env->bullet->softBodyWorldInfo));
        env->add(cloth);

        facepicker.reset(new SoftBodyFacePicker(*this, viewer.getCamera(), cloth->softBody.get()));
        facepicker->setPickCallback(boost::bind(&CustomScene::pickCallback, this, _1));

        sbgripperleft.reset(new PR2SoftBodyGripper(pr2m->pr2, pr2m->pr2Left->manip, true));
        sbgripperleft->setGrabOnlyOnContact(true);
        sbgripperleft->setTarget(cloth);

        PR2SoftBodyGripperAction leftAction(pr2m->pr2, pr2m->pr2Left->manip, sbgripperleft);
        leftAction.setTarget(cloth);
        leftAction.setExecTime(1.);
        addVoidKeyCallback('a', boost::bind(&CustomScene::runGripperAction, this, leftAction));
        addVoidKeyCallback('z', boost::bind(&CustomScene::saveManipTrans, this, pr2m->pr2Left));
        addVoidKeyCallback('x', boost::bind(&CustomScene::moveArmToSaved, this, pr2m->pr2, pr2m->pr2Left));
        addVoidKeyCallback('c', boost::bind(&CustomScene::graspPickedNode, this));
        addVoidKeyCallback('f', boost::bind(&CustomScene::greedyFlattenSingle, this));
        addVoidKeyCallback('g', boost::bind(&CustomScene::liftCloth, this));

        addPreDrawCallback(boost::bind(&CustomScene::markFolds, this));
        addPreDrawCallback(boost::bind(&CustomScene::drawPick, this));

        startViewer();
        startFixedTimestepLoop(BulletConfig::dt);
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

    CustomScene().run();

    return 0;
}
