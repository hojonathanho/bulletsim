#include "simplescene.h"
#include "softbodies.h"
#include "userconfig.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>

class GripperAction : public Action {
    RaveRobotKinematicObject::Ptr robot;
    dReal startVal, endVal;
    vector<int> indices;
    vector<dReal> vals;

public:
    typedef boost::shared_ptr<GripperAction> Ptr;
    GripperAction(RaveRobotKinematicObject::Ptr robot_, const string &jointName, float time) :
            robot(robot_), Action(time) {
        int idx = robot->robot->GetJointIndex(jointName);
        indices.push_back(idx);
        vals.push_back(0);
    }
    void setEndpoints(dReal start, dReal end) { startVal = start; endVal = end; }
    void setOpenAction() { setEndpoints(0.f, 0.54f); } // 0.54 is the max joint value for the pr2
    void setCloseAction() { setEndpoints(0.54f, 0.f); }

    void step(float dt) {
        if (done()) return;
        stepTime(dt);

        float frac = fracElapsed();
        vals[0] = (1.f - frac)*startVal + frac*endVal;
        robot->setDOFValues(indices, vals);
    }
};


struct CustomScene : public Scene {
    GripperAction::Ptr leftAction, rightAction;
    BulletSoftObject::Ptr createCloth(btScalar s, const btVector3 &center, int divs);
    void run();
};

class CustomKeyHandler : public osgGA::GUIEventHandler {
    CustomScene &scene;
public:
    CustomKeyHandler(CustomScene &scene_) : scene(scene_) { }
    bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&);
};

bool CustomKeyHandler::handle(const osgGA::GUIEventAdapter &ea,osgGA::GUIActionAdapter &) {
    switch (ea.getEventType()) {
    case osgGA::GUIEventAdapter::KEYDOWN:
        switch (ea.getKey()) {
        case 'a':
            scene.leftAction->reset();
            scene.leftAction->setCloseAction();
            scene.runAction(scene.leftAction, CFG.bullet.dt);
            break;
        case 's':
            scene.rightAction->reset();
            scene.rightAction->setCloseAction();
            scene.runAction(scene.rightAction, CFG.bullet.dt);
            break;
        }
        break;
    }
    return false;
}

BulletSoftObject::Ptr CustomScene::createCloth(btScalar s, const btVector3 &center, int divs) {
    btSoftBody *psb = btSoftBodyHelpers::CreatePatch(
        env->bullet->softBodyWorldInfo,
        center + btVector3(-s,-s,0),
        center + btVector3(+s,-s,0),
        center + btVector3(-s,+s,0),
        center + btVector3(+s,+s,0),
        divs, divs,
        0, true);

    psb->m_cfg.piterations = 4;
    psb->m_cfg.collisions = btSoftBody::fCollision::CL_SS
        + btSoftBody::fCollision::CL_RS
        + btSoftBody::fCollision::CL_SELF;
    psb->m_cfg.kDF = 0.9;
    psb->getCollisionShape()->setMargin(0.03);
    btSoftBody::Material *pm = psb->appendMaterial();
//    pm->m_kLST = 0.4;
//    pm->m_kAST = 0.4;
    psb->generateBendingConstraints(2, pm);
    psb->randomizeConstraints();
    psb->setTotalMass(1, true);
    psb->generateClusters(0);

    return BulletSoftObject::Ptr(new BulletSoftObject(psb));
}

void CustomScene::run() {
    viewer.addEventHandler(new CustomKeyHandler(*this));

    const float dt = CFG.bullet.dt;
    const float table_height = .5;
    const float table_thickness = .05;
    boost::shared_ptr<btDefaultMotionState> ms(new btDefaultMotionState(
        btTransform(btQuaternion(0, 0, 0, 1),
                    CFG.scene.scale * btVector3(1, 0, table_height-table_thickness/2))));
    boost::shared_ptr<BulletObject> table(
        new BoxObject(0, CFG.scene.scale * btVector3(.75,.75,table_thickness/2),ms));

    env->add(table);
    env->add(createCloth(CFG.scene.scale * 0.25, CFG.scene.scale * btVector3(0.75, 0, 1), 31));

    leftAction.reset(new GripperAction(pr2, "l_gripper_l_finger_joint", 1));
    rightAction.reset(new GripperAction(pr2, "r_gripper_l_finger_joint", 1));


    //setSyncTime(true);
    startViewer();
    stepFor(dt, 2);

    leftAction->setOpenAction();
    runAction(leftAction, dt);

    rightAction->setOpenAction();
    runAction(rightAction, dt);

    //startLoop();
    startFixedTimestepLoop(dt);
}

int main(int argc, char *argv[]) {
    Config::read(argc, argv);
    //CFG.scene.mouseDragScale = 0.1;
    CFG.scene.scale = 20.;
    CFG.viewer.cameraHomePosition = btVector3(100, 0, 100);
    CFG.bullet.dt = 0.01;
    CFG.bullet.internalTimeStep = 0.01;
    CFG.bullet.maxSubSteps = 0;

    CustomScene().run();
    return 0;
}
