#include "simulation/simplescene.h"
#include "simulation/config_bullet.h"
#include "simulation/config_viewer.h"
#include "robots/pr2.h"
#include <cstdlib>

#include "make_bodies.h"
#include "clothutil.h"
#include "nodeactions.h"
#include "clothgrasping.h"

class CustomScene : Scene {
    BulletSoftObject::Ptr cloth;
    PR2Manager::Ptr pr2m;

    void runGripperAction(PR2SoftBodyGripperAction &a) {
        a.reset();
        a.toggleAction();
        runAction(a, BulletConfig::dt);
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

    void doStuff() {
//        savedTrans.setOrigin(cloth->softBody->m_nodes[cloth->softBody->m_nodes.size()-10].m_x);
        runAction(Action::Ptr(new GraspClothNodeAction(pr2m->pr2, pr2m->pr2Left, cloth->softBody.get(), cloth->softBody->m_nodes.size()-10, btVector3(1, -1, 0))), BulletConfig::dt);
        cout << "done." << endl;
    }

public:
    void run() {
        const float table_height = .5;
        const float table_thickness = .05;
        BoxObject::Ptr table(new BoxObject(0, GeneralConfig::scale * btVector3(.75,.75,table_thickness/2),
            btTransform(btQuaternion(0, 0, 0, 1), GeneralConfig::scale * btVector3(0.8, 0, table_height-table_thickness/2))));
        table->rigidBody->setFriction(0.1);
        env->add(table);

        //const int resx = (int)(45*1.5), resy = (int)(31*1.5);
        const int resx = 45, resy = 31;
        const btScalar lenx = GeneralConfig::scale * 0.7, leny = GeneralConfig::scale * 0.5;
        const btVector3 clothcenter = GeneralConfig::scale * btVector3(0.5, 0, table_height+0.01);
        cloth = makeSelfCollidingTowel(clothcenter, lenx, leny, resx, resy, env->bullet->softBodyWorldInfo);
        env->add(cloth);
        ClothSpec clothspec = { cloth->softBody.get(), resx, resy };

        pr2m.reset(new PR2Manager(*this));

        PR2SoftBodyGripperAction leftAction(pr2m->pr2, pr2m->pr2Left->manip, true);
        leftAction.setTarget(cloth->softBody.get());
        leftAction.setExecTime(1.);
        addVoidKeyCallback('a', boost::bind(&CustomScene::runGripperAction, this, leftAction));

        addVoidKeyCallback('z', boost::bind(&CustomScene::saveManipTrans, this, pr2m->pr2Left));
        addVoidKeyCallback('x', boost::bind(&CustomScene::moveArmToSaved, this, pr2m->pr2, pr2m->pr2Left));
        addVoidKeyCallback('c', boost::bind(&CustomScene::doStuff, this));

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
