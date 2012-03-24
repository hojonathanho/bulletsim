#include "simulation/simplescene.h"
#include "simulation/config_bullet.h"
#include "simulation/config_viewer.h"
#include "simulation/plotting.h"
#include "robots/pr2.h"
#include <cstdlib>

#include "make_bodies.h"
#include "clothutil.h"
#include "nodeactions.h"
#include "clothgrasping.h"
#include "edges.h"
#include "facepicker.h"

class CustomScene : Scene {
    Cloth::Ptr cloth;
    PR2Manager::Ptr pr2m;

    PlotPoints::Ptr foldnodeplot;
    PlotLines::Ptr folddirplot;
    PlotSpheres::Ptr pickplot;
    SoftBodyFacePicker::Ptr facepicker;

    vector<int> foldnodes;

    void runGripperAction(PR2SoftBodyGripperAction &a) {
        a.reset();
        a.toggleAction();
        runAction(a, BulletConfig::dt);
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
        runAction(a, BulletConfig::dt);
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
        BulletSoftObject::Ptr fork_cloth =
            boost::static_pointer_cast<BulletSoftObject>(
                    fork->forkOf(cloth));
        RaveRobotObject::Manipulator::Ptr fork_pr2Left =
            fork_pr2->getManipByIndex(pr2m->pr2Left->index);
        osg->root->addChild(fork_osg->root);

        Action::Ptr a(new GraspClothNodeAction(
                    fork_pr2, fork_pr2Left, fork_cloth->softBody.get(),
                    node, gripperdir));
        while (!a->done()) {
            a->step(BulletConfig::dt);
            fork->env->step(BulletConfig::dt,
                    BulletConfig::maxSubSteps, BulletConfig::internalTimeStep);
            draw();
        }
        osg->root->removeChild(fork_osg->root);

        return fork_cloth->softBody->m_anchors.size();
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
            // if not an edge node, there's ambiguitiy in the gripper direction
            // (either gripperdir or -gripperdir)
            // choose the one that attaches the most anchors to the softbody
            int nforward = tryGrasp(node, gripperdir);
            int nbackward = tryGrasp(node, -gripperdir);
            cout << "forward: " << nforward << " backward: " << nbackward << endl;

            if (nbackward > nforward)
                gripperdir = -gripperdir;
        }

        runAction(Action::Ptr(new GraspClothNodeAction(
                        pr2m->pr2, pr2m->pr2Left, cloth->psb(),
                        node, gripperdir)),
                BulletConfig::dt);
        cout << "done." << endl;
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

        const float table_height = .5;
        const float table_thickness = .05;
        BoxObject::Ptr table(new BoxObject(0, GeneralConfig::scale * btVector3(.75,.75,table_thickness/2),
            btTransform(btQuaternion(0, 0, 0, 1), GeneralConfig::scale * btVector3(0.8, 0, table_height-table_thickness/2))));
        table->rigidBody->setFriction(0.1);
        env->add(table);
        cout << "table margin: " << table->rigidBody->getCollisionShape()->getMargin() << endl;

        //const int resx = (int)(45*1.5), resy = (int)(31*1.5);
        const int resx = 45, resy = 31;
        const btScalar lenx = GeneralConfig::scale * 0.7, leny = GeneralConfig::scale * 0.5;
        const btVector3 clothcenter = GeneralConfig::scale * btVector3(0.5, 0, table_height+0.01);
//        cloth = makeSelfCollidingTowel(clothcenter, lenx, leny, resx, resy, env->bullet->softBodyWorldInfo);
        cloth.reset(new Cloth(resx, resy, lenx, leny, clothcenter, env->bullet->softBodyWorldInfo));
        env->add(cloth);

        facepicker.reset(new SoftBodyFacePicker(*this, viewer.getCamera(), cloth->softBody.get()));
        facepicker->setPickCallback(boost::bind(&CustomScene::pickCallback, this, _1));

        pr2m.reset(new PR2Manager(*this));

        PR2SoftBodyGripperAction leftAction(pr2m->pr2, pr2m->pr2Left->manip, true);
        leftAction.setTarget(cloth->psb());
        leftAction.setExecTime(1.);
        addVoidKeyCallback('a', boost::bind(&CustomScene::runGripperAction, this, leftAction));
        addVoidKeyCallback('z', boost::bind(&CustomScene::saveManipTrans, this, pr2m->pr2Left));
        addVoidKeyCallback('x', boost::bind(&CustomScene::moveArmToSaved, this, pr2m->pr2, pr2m->pr2Left));
        addVoidKeyCallback('c', boost::bind(&CustomScene::graspPickedNode, this));

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
