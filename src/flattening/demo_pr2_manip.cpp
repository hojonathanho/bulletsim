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
#include "config_flattening.h"

class CustomScene : Scene {
    Cloth::Ptr cloth;
    PR2Manager::Ptr pr2m;
    GenManip::Ptr gleft, gright;
    GenPR2SoftGripper::Ptr sbgripperleft;
    PlotAxes::Ptr leftManipAxes, rightManipAxes;

    PlotPoints::Ptr foldnodeplot;
    PlotLines::Ptr folddirplot;
    PlotSpheres::Ptr pickplot;
    SoftBodyFacePicker::Ptr facepicker;

    vector<int> foldnodes;

    void runGripperAction(GenPR2SoftGripperAction &a) {
        a.reset();
        a.toggleAction();
        try { runAction(a, BulletConfig::dt); } catch (...) { }
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
        GraspingActionContext ctx(env, pr2m->pr2, gleft, sbgripperleft, cloth);
        btVector3 gripperdir = calcGraspDir(ctx, node);
        stringstream ss; ss << "grab " << node << ' ' << gripperdir.x() << ' ' << gripperdir.y() << ' ' << gripperdir.z();
        GraspingActionSpec spec(ss.str());
        Action::Ptr a(spec.createAction(ctx));
        try { runAction(a, BulletConfig::dt); } catch (...) { }
        cout << "done." << endl;
    }

    GraspingActionSpec prevActionSpec;
    void greedyFlattenSingle() {
        GraspingActionContext ctx(env, pr2m->pr2, gleft, sbgripperleft, cloth);
        GraspingActionSpec spec = flattenCloth_greedy_single(ctx, prevActionSpec);
        try { runAction(spec.createAction(ctx), BulletConfig::dt); } catch (...) { }
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

    void drawManipAxes() {
        leftManipAxes->setup(gleft->getTransform(), 0.05*METERS);
        rightManipAxes->setup(gright->getTransform(), 0.05*METERS);
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

        leftManipAxes.reset(new PlotAxes);
        env->add(leftManipAxes);
        rightManipAxes.reset(new PlotAxes);
        env->add(rightManipAxes);

        // load the robot
        pr2m.reset(new PR2Manager(*this));
        if (FlatteningConfig::useFakeGripper) {
            TelekineticGripper::Ptr fakeLeft(new TelekineticGripper(pr2m->pr2Left));
            fakeLeft->setTransform(pr2m->pr2Left->getTransform());
            env->add(fakeLeft);
            gleft.reset(new GenManip(fakeLeft));

            TelekineticGripper::Ptr fakeRight(new TelekineticGripper(pr2m->pr2Right));
            fakeRight->setTransform(pr2m->pr2Right->getTransform());
            gright.reset(new GenManip(fakeRight));
            env->add(fakeRight);

            pr2m->pr2->setTransform(btTransform(btQuaternion::getIdentity(), btVector3(0, 0, -100))); // out of view

        } else {
            gleft.reset(new GenManip(pr2m->pr2Left));
            gright.reset(new GenManip(pr2m->pr2Right));
        }

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
        const btVector3 clothcenter = GeneralConfig::scale * btVector3(0.3, 0.1, table_height+0.01);
//        cloth = makeSelfCollidingTowel(clothcenter, lenx, leny, resx, resy, env->bullet->softBodyWorldInfo);
        cloth.reset(new Cloth(resx, resy, lenx, leny, clothcenter, env->bullet->softBodyWorldInfo));
        env->add(cloth);

        facepicker.reset(new SoftBodyFacePicker(*this, viewer.getCamera(), cloth->softBody.get()));
        facepicker->setPickCallback(boost::bind(&CustomScene::pickCallback, this, _1));

        sbgripperleft.reset(new GenPR2SoftGripper(pr2m->pr2, gleft, true));
        sbgripperleft->setGrabOnlyOnContact(true);
        sbgripperleft->setTarget(cloth);

        GenPR2SoftGripperAction leftAction(pr2m->pr2, gleft->baseManip()->manip, sbgripperleft);
        leftAction.setTarget(cloth);
        leftAction.setExecTime(1.);
        addVoidKeyCallback('a', boost::bind(&CustomScene::runGripperAction, this, leftAction));
        addVoidKeyCallback('c', boost::bind(&CustomScene::graspPickedNode, this));
        addVoidKeyCallback('f', boost::bind(&CustomScene::greedyFlattenSingle, this));
        addVoidKeyCallback('g', boost::bind(&CustomScene::liftCloth, this));

        addPreDrawCallback(boost::bind(&CustomScene::markFolds, this));
        addPreDrawCallback(boost::bind(&CustomScene::drawPick, this));
        addPreDrawCallback(boost::bind(&CustomScene::drawManipAxes, this));

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
    parser.addGroup(FlatteningConfig());
    parser.read(argc, argv);

    CustomScene().run();

    return 0;
}
